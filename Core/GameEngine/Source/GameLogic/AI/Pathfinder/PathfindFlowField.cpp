/*
**	Command & Conquer Generals Zero Hour(tm)
**	Copyright 2026 TheSuperHackers
**
**	This program is free software: you can redistribute it and/or modify
**	it under the terms of the GNU General Public License as published by
**	the Free Software Foundation, either version 3 of the License, or
**	(at your option) any later version.
**
**	This program is distributed in the hope that it will be useful,
**	but WITHOUT ANY WARRANTY; without even the implied warranty of
**	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**	GNU General Public License for more details.
**
**	You should have received a copy of the GNU General Public License
**	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "PreRTS.h"
#include "GameLogic/Pathfinder/PathfindConstants.h"
#include "GameLogic/Pathfinder/PathfindFlowField.h"

namespace
{
	constexpr UnsignedInt FLOW_COST_UNREACHABLE = 0xFFFFFFFFu;
	constexpr UnsignedInt SOFT_CLEARANCE_PENALTY = 6; // Max step cost = 14 + 6 = 20 < DIAL_BUCKET_COUNT (32)

	// Fixed deterministic neighbor order: 4 orthogonal (0..3), then 4 diagonal (4..7).
	constexpr Int DIR_DX[8] = {  1, -1,  0,  0,  1,  1, -1, -1 };
	constexpr Int DIR_DY[8] = {  0,  0,  1, -1,  1, -1,  1, -1 };

	// Opposite direction index: when expanding from u -> v along d, unit at v flows toward u via OPPOSITE_DIR[d].
	constexpr signed char OPPOSITE_DIR[8] = { 1, 0, 3, 2, 7, 6, 5, 4 };

	// For each diagonal d in [4..7], the two orthogonal indices that must both be passable to prevent corner cutting.
	constexpr Int DIAG_ORTHO_A[4] = { 0, 0, 1, 1 };
	constexpr Int DIAG_ORTHO_B[4] = { 2, 3, 2, 3 };

	inline UnsignedInt packCell(Int x, Int y)
	{
		return (static_cast<UnsignedInt>(x) << 16) | (static_cast<UnsignedInt>(y) & 0xFFFFu);
	}

	inline Int unpackX(UnsignedInt packed)
	{
		return static_cast<Int>(packed >> 16);
	}

	inline Int unpackY(UnsignedInt packed)
	{
		return static_cast<Int>(packed & 0xFFFFu);
	}
}

PathfindFlowFieldManager::PathfindFlowFieldManager()
	: m_mapVersion(1)
	, m_lruClock(0)
{
	for (Int b = 0; b < DIAL_BUCKET_COUNT; ++b)
	{
		m_buckets[b].reserve(256);
	}
}

void PathfindFlowFieldManager::reset()
{
	m_mapVersion = 1;
	m_lruClock = 0;

	for (Int i = 0; i < CACHE_CAPACITY; ++i)
	{
		m_cache[i].mapVersion = 0;
		m_cache[i].lastUsedSeq = 0;
		m_cache[i].width = 0;
		m_cache[i].height = 0;
		m_cache[i].flowDir.clear();
	}

	for (Int i = 0; i < DEMAND_TABLE_SIZE; ++i)
	{
		m_demand[i].count = 0;
		m_demand[i].lastFrame = 0;
		m_demand[i].mapVersion = 0;
	}
}

void PathfindFlowFieldManager::invalidateCache()
{
	++m_mapVersion;
	if (m_mapVersion == 0)
	{
		m_mapVersion = 1;
		for (Int i = 0; i < CACHE_CAPACITY; ++i)
		{
			m_cache[i].mapVersion = 0;
		}
		for (Int i = 0; i < DEMAND_TABLE_SIZE; ++i)
		{
			m_demand[i].mapVersion = 0;
		}
	}
}

std::vector<UnsignedByte> &PathfindFlowFieldManager::getPassabilityScratch(size_t totalCells)
{
	if (m_passabilityScratch.size() < totalCells)
	{
		m_passabilityScratch.resize(totalCells);
	}
	return m_passabilityScratch;
}

Bool PathfindFlowFieldManager::recordDemandAndShouldUse(const FlowFieldKey &key, UnsignedInt currentFrame)
{
	// Fast path: if an up-to-date flow field is already cached for this key, use it immediately.
	for (Int i = 0; i < CACHE_CAPACITY; ++i)
	{
		if (m_cache[i].mapVersion == m_mapVersion && m_cache[i].key == key)
		{
			return true;
		}
	}

	Int matchSlot = -1;
	Int evictSlot = 0;
	UnsignedInt oldestFrame = 0xFFFFFFFFu;

	for (Int i = 0; i < DEMAND_TABLE_SIZE; ++i)
	{
		DemandSlot &slot = m_demand[i];
		if (slot.mapVersion == m_mapVersion && slot.count > 0 && slot.key == key)
		{
			matchSlot = i;
			break;
		}

		if (slot.mapVersion != m_mapVersion || slot.count == 0)
		{
			evictSlot = i;
			oldestFrame = 0;
		}
		else if (slot.lastFrame < oldestFrame)
		{
			oldestFrame = slot.lastFrame;
			evictSlot = i;
		}
	}

	if (matchSlot >= 0)
	{
		DemandSlot &slot = m_demand[matchSlot];
		if (currentFrame >= slot.lastFrame && (currentFrame - slot.lastFrame) > DEMAND_WINDOW_FRAMES)
		{
			slot.count = 1;
		}
		else
		{
			++slot.count;
		}
		slot.lastFrame = currentFrame;
		return slot.count >= ACTIVATION_THRESHOLD;
	}

	DemandSlot &slot = m_demand[evictSlot];
	slot.key = key;
	slot.mapVersion = m_mapVersion;
	slot.lastFrame = currentFrame;
	slot.count = 1;
	return slot.count >= ACTIVATION_THRESHOLD;
}

const PathfindFlowFieldManager::FlowFieldEntry *PathfindFlowFieldManager::findCached(const FlowFieldKey &key)
{
	for (Int i = 0; i < CACHE_CAPACITY; ++i)
	{
		if (m_cache[i].mapVersion == m_mapVersion && m_cache[i].key == key)
		{
			m_cache[i].lastUsedSeq = ++m_lruClock;
			return &m_cache[i];
		}
	}
	return nullptr;
}

PathfindFlowFieldManager::FlowFieldEntry *PathfindFlowFieldManager::allocateSlot(const FlowFieldKey &key)
{
	Int victimIndex = 0;
	UnsignedInt oldestSeq = 0xFFFFFFFFu;

	for (Int i = 0; i < CACHE_CAPACITY; ++i)
	{
		if (m_cache[i].mapVersion == m_mapVersion && m_cache[i].key == key)
		{
			return &m_cache[i];
		}

		if (m_cache[i].mapVersion != m_mapVersion)
		{
			return &m_cache[i];
		}

		if (m_cache[i].lastUsedSeq < oldestSeq)
		{
			oldestSeq = m_cache[i].lastUsedSeq;
			victimIndex = i;
		}
	}

	return &m_cache[victimIndex];
}

const PathfindFlowFieldManager::FlowFieldEntry *PathfindFlowFieldManager::buildFlowField(
	const FlowFieldKey &key,
	Int width,
	Int height,
	const IRegion2D &bounds,
	Int seedX,
	Int seedY,
	const UnsignedByte *passabilityGrid)
{
	if (width <= 0 || height <= 0 || passabilityGrid == nullptr)
	{
		return nullptr;
	}

	if (seedX < bounds.lo.x || seedX > bounds.hi.x || seedY < bounds.lo.y || seedY > bounds.hi.y)
	{
		return nullptr;
	}

	const size_t totalCells = static_cast<size_t>(width) * static_cast<size_t>(height);
	const size_t seedIndex = static_cast<size_t>(seedX) * static_cast<size_t>(height) + static_cast<size_t>(seedY);
	if (passabilityGrid[seedIndex] == PASS_BLOCKED)
	{
		return nullptr;
	}

	FlowFieldEntry *entry = allocateSlot(key);
	entry->key = key;
	entry->mapVersion = m_mapVersion;
	entry->lastUsedSeq = ++m_lruClock;
	entry->seedX = seedX;
	entry->seedY = seedY;
	entry->width = width;
	entry->height = height;
	entry->bounds = bounds;
	entry->flowDir.assign(totalCells, FLOW_DIR_NONE);

	m_costScratch.assign(totalCells, FLOW_COST_UNREACHABLE);

	size_t bucketHead[DIAL_BUCKET_COUNT];
	for (Int b = 0; b < DIAL_BUCKET_COUNT; ++b)
	{
		m_buckets[b].clear();
		bucketHead[b] = 0;
	}

	m_costScratch[seedIndex] = 0;
	entry->flowDir[seedIndex] = FLOW_DIR_GOAL;
	m_buckets[0].push_back(packCell(seedX, seedY));

	UnsignedInt currentCost = 0;
	size_t queuedEntries = 1;
	const UnsignedInt bucketMask = DIAL_BUCKET_COUNT - 1;

	while (queuedEntries > 0)
	{
		UnsignedInt bucketIdx = currentCost & bucketMask;
		while (bucketHead[bucketIdx] >= m_buckets[bucketIdx].size())
		{
			m_buckets[bucketIdx].clear();
			bucketHead[bucketIdx] = 0;
			++currentCost;
			bucketIdx = currentCost & bucketMask;
		}

		const UnsignedInt packed = m_buckets[bucketIdx][bucketHead[bucketIdx]++];
		--queuedEntries;

		if (bucketHead[bucketIdx] == m_buckets[bucketIdx].size())
		{
			m_buckets[bucketIdx].clear();
			bucketHead[bucketIdx] = 0;
		}

		const Int ux = unpackX(packed);
		const Int uy = unpackY(packed);
		const size_t uIdx = static_cast<size_t>(ux) * static_cast<size_t>(height) + static_cast<size_t>(uy);

		// Skip stale queue entries superseded by a cheaper path.
		if (m_costScratch[uIdx] < currentCost)
		{
			continue;
		}

		// Expand 4 orthogonal neighbors first and record passability for diagonal corner-cutting checks.
		Bool orthoPassable[4] = { false, false, false, false };
		for (Int d = 0; d < 4; ++d)
		{
			const Int vx = ux + DIR_DX[d];
			const Int vy = uy + DIR_DY[d];
			if (vx < bounds.lo.x || vx > bounds.hi.x || vy < bounds.lo.y || vy > bounds.hi.y)
			{
				continue;
			}

			const size_t vIdx = static_cast<size_t>(vx) * static_cast<size_t>(height) + static_cast<size_t>(vy);
			const UnsignedByte passState = passabilityGrid[vIdx];
			if (passState == PASS_BLOCKED)
			{
				continue;
			}

			orthoPassable[d] = true;
			const UnsignedInt stepCost = COST_ORTHOGONAL + ((passState == PASS_SOFT_PENALTY) ? SOFT_CLEARANCE_PENALTY : 0u);
			const UnsignedInt newCost = currentCost + stepCost;

			if (newCost < m_costScratch[vIdx])
			{
				m_costScratch[vIdx] = newCost;
				entry->flowDir[vIdx] = OPPOSITE_DIR[d];
				m_buckets[newCost & bucketMask].push_back(packCell(vx, vy));
				++queuedEntries;
			}
		}

		// Expand 4 diagonal neighbors only when both adjacent orthogonal cells are passable (no corner cutting).
		for (Int d = 4; d < 8; ++d)
		{
			if (!orthoPassable[DIAG_ORTHO_A[d - 4]] || !orthoPassable[DIAG_ORTHO_B[d - 4]])
			{
				continue;
			}

			const Int vx = ux + DIR_DX[d];
			const Int vy = uy + DIR_DY[d];
			if (vx < bounds.lo.x || vx > bounds.hi.x || vy < bounds.lo.y || vy > bounds.hi.y)
			{
				continue;
			}

			const size_t vIdx = static_cast<size_t>(vx) * static_cast<size_t>(height) + static_cast<size_t>(vy);
			const UnsignedByte passState = passabilityGrid[vIdx];
			if (passState == PASS_BLOCKED)
			{
				continue;
			}

			const UnsignedInt stepCost = COST_DIAGONAL + ((passState == PASS_SOFT_PENALTY) ? SOFT_CLEARANCE_PENALTY : 0u);
			const UnsignedInt newCost = currentCost + stepCost;

			if (newCost < m_costScratch[vIdx])
			{
				m_costScratch[vIdx] = newCost;
				entry->flowDir[vIdx] = OPPOSITE_DIR[d];
				m_buckets[newCost & bucketMask].push_back(packCell(vx, vy));
				++queuedEntries;
			}
		}
	}

	return entry;
}

Bool PathfindFlowFieldManager::tracePath(
	const FlowFieldEntry &entry,
	Int startX,
	Int startY,
	Int goalX,
	Int goalY,
	std::vector<ICoord2D> &outWaypoints)
{
	outWaypoints.clear();

	if (entry.width <= 0 || entry.height <= 0 || entry.flowDir.empty())
	{
		return false;
	}

	if (startX < entry.bounds.lo.x || startX > entry.bounds.hi.x ||
		startY < entry.bounds.lo.y || startY > entry.bounds.hi.y)
	{
		return false;
	}

	const Int height = entry.height;
	const size_t startIdx = static_cast<size_t>(startX) * static_cast<size_t>(height) + static_cast<size_t>(startY);
	if (entry.flowDir[startIdx] == FLOW_DIR_NONE)
	{
		return false;
	}

	Int curX = startX;
	Int curY = startY;
	signed char prevDir = -1;
	const Int maxSteps = (entry.width + entry.height) * 2 + 64;

	for (Int step = 0; step < maxSteps; ++step)
	{
		if (curX == goalX && curY == goalY)
		{
			ICoord2D pt;
			pt.x = goalX;
			pt.y = goalY;
			if (outWaypoints.empty() || outWaypoints.back().x != pt.x || outWaypoints.back().y != pt.y)
			{
				outWaypoints.push_back(pt);
			}
			return true;
		}

		const size_t curIdx = static_cast<size_t>(curX) * static_cast<size_t>(height) + static_cast<size_t>(curY);
		const signed char dir = entry.flowDir[curIdx];

		if (dir == FLOW_DIR_GOAL)
		{
			// Reached the cluster seed; append seed corner and final exact goal cell.
			ICoord2D seedPt;
			seedPt.x = curX;
			seedPt.y = curY;
			if (outWaypoints.empty() || outWaypoints.back().x != seedPt.x || outWaypoints.back().y != seedPt.y)
			{
				outWaypoints.push_back(seedPt);
			}

			ICoord2D goalPt;
			goalPt.x = goalX;
			goalPt.y = goalY;
			outWaypoints.push_back(goalPt);
			return true;
		}

		if (dir < 0 || dir >= 8)
		{
			outWaypoints.clear();
			return false;
		}

		if (prevDir >= 0 && dir != prevDir)
		{
			ICoord2D corner;
			corner.x = curX;
			corner.y = curY;
			outWaypoints.push_back(corner);
		}

		curX += DIR_DX[dir];
		curY += DIR_DY[dir];
		prevDir = dir;

		if (curX < entry.bounds.lo.x || curX > entry.bounds.hi.x ||
			curY < entry.bounds.lo.y || curY > entry.bounds.hi.y)
		{
			outWaypoints.clear();
			return false;
		}
	}

	outWaypoints.clear();
	return false;
}
