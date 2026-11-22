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
#include "GameLogic/Pathfinder/PathfindCell.h"
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
	: m_poolStarted(false)
	, m_poolShutdown(false)
	, m_batchEpoch(0)
	, m_activeJobs(nullptr)
	, m_activeJobCount(0)
	, m_activeWidth(0)
	, m_activeHeight(0)
	, m_activeSnapshot(nullptr)
	, m_nextJobIndex(0)
	, m_completedJobs(0)
	, m_mapVersion(1)
	, m_lruClock(0)
{
	// Always keep scratch[0] ready for the main thread even before workers spawn.
	m_workerScratches.resize(1);
	for (Int b = 0; b < DIAL_BUCKET_COUNT; ++b)
	{
		m_workerScratches[0].buckets[b].reserve(256);
	}
}

PathfindFlowFieldManager::~PathfindFlowFieldManager()
{
	shutdownWorkers();
}

void PathfindFlowFieldManager::ensureWorkersStarted()
{
	if (m_poolStarted)
	{
		return;
	}
	m_poolStarted = true;

	unsigned int hwThreads = std::thread::hardware_concurrency();
	if (hwThreads <= 1)
	{
		return;
	}

	unsigned int bgCount = hwThreads - 1;
	if (bgCount > static_cast<unsigned int>(MAX_WORKER_THREADS))
	{
		bgCount = static_cast<unsigned int>(MAX_WORKER_THREADS);
	}

	// Resize scratches BEFORE spawning threads so references remain stable forever.
	m_workerScratches.resize(static_cast<size_t>(bgCount) + 1);
	for (size_t s = 0; s < m_workerScratches.size(); ++s)
	{
		for (Int b = 0; b < DIAL_BUCKET_COUNT; ++b)
		{
			m_workerScratches[s].buckets[b].reserve(256);
		}
	}

	m_workers.reserve(bgCount);
	for (unsigned int w = 0; w < bgCount; ++w)
	{
		const size_t scratchIndex = static_cast<size_t>(w) + 1;
		m_workers.emplace_back(&PathfindFlowFieldManager::workerLoop, this, scratchIndex);
	}
}

void PathfindFlowFieldManager::shutdownWorkers()
{
	if (m_workers.empty())
	{
		return;
	}

	{
		std::lock_guard<std::mutex> lock(m_poolMutex);
		m_poolShutdown = true;
		++m_batchEpoch;
	}
	m_cvWork.notify_all();

	for (size_t i = 0; i < m_workers.size(); ++i)
	{
		if (m_workers[i].joinable())
		{
			m_workers[i].join();
		}
	}
	m_workers.clear();
	m_poolStarted = false;
	m_poolShutdown = false;
}

void PathfindFlowFieldManager::workerLoop(size_t scratchIndex)
{
	UnsignedInt seenEpoch = 0;
	for (;;)
	{
		{
			std::unique_lock<std::mutex> lock(m_poolMutex);
			m_cvWork.wait(lock, [this, seenEpoch]()
			{
				return m_poolShutdown || m_batchEpoch != seenEpoch;
			});

			if (m_poolShutdown)
			{
				return;
			}
			seenEpoch = m_batchEpoch;
		}

		executeBatchJobs(scratchIndex);
	}
}

UnsignedByte PathfindFlowFieldManager::decodePassabilityFromSnapshot(
	UnsignedByte snap,
	Bool isCrusher,
	LocomotorSurfaceTypeMask acceptableSurfaces)
{
	Bool valid = false;
	if (isCrusher && (snap & SNAP_FENCE) != 0)
	{
		valid = true;
	}
	else
	{
		LocomotorSurfaceTypeMask cellSurfaces = NO_SURFACES;
		switch (static_cast<PathfindCell::CellType>(snap & SNAP_TYPE_MASK))
		{
			case PathfindCell::CELL_CLEAR:
				cellSurfaces = LOCOMOTORSURFACE_GROUND | LOCOMOTORSURFACE_AIR;
				break;
			case PathfindCell::CELL_WATER:
				cellSurfaces = LOCOMOTORSURFACE_WATER | LOCOMOTORSURFACE_AIR;
				break;
			case PathfindCell::CELL_CLIFF:
				cellSurfaces = LOCOMOTORSURFACE_CLIFF | LOCOMOTORSURFACE_AIR;
				break;
			case PathfindCell::CELL_RUBBLE:
				cellSurfaces = LOCOMOTORSURFACE_RUBBLE | LOCOMOTORSURFACE_AIR;
				break;
			case PathfindCell::CELL_OBSTACLE:
			case PathfindCell::CELL_BRIDGE_IMPASSABLE:
			case PathfindCell::CELL_IMPASSABLE:
				cellSurfaces = LOCOMOTORSURFACE_AIR;
				break;
			default:
				cellSurfaces = NO_SURFACES;
				break;
		}
		valid = ((cellSurfaces & acceptableSurfaces) != 0);
	}

	if (!valid)
	{
		return PASS_BLOCKED;
	}

	return ((snap & SNAP_PINCHED) != 0) ? PASS_SOFT_PENALTY : PASS_CLEAR;
}

void PathfindFlowFieldManager::executeBatchJobs(size_t scratchIndex)
{
	WorkerScratch &scratch = m_workerScratches[scratchIndex];
	const size_t totalCells = static_cast<size_t>(m_activeWidth) * static_cast<size_t>(m_activeHeight);

	for (;;)
	{
		const size_t jobIdx = m_nextJobIndex.fetch_add(1, std::memory_order_relaxed);
		if (jobIdx >= m_activeJobCount)
		{
			break;
		}

		const ParallelBuildJob &job = m_activeJobs[jobIdx];
		ParallelJobResult &result = m_parallelResults[jobIdx];
		result.key = job.key;
		result.bounds = job.bounds;
		result.seedX = job.seedX;
		result.seedY = job.seedY;
		result.width = m_activeWidth;
		result.height = m_activeHeight;
		result.valid = false;

		if (scratch.passability.size() < totalCells)
		{
			scratch.passability.resize(totalCells);
		}
		std::fill(scratch.passability.begin(), scratch.passability.begin() + totalCells, static_cast<UnsignedByte>(PASS_BLOCKED));

		const Int height = m_activeHeight;
		const Bool isCrusher = job.key.isCrusher;
		const LocomotorSurfaceTypeMask surfaces = job.key.acceptableSurfaces;

		for (Int x = job.bounds.lo.x; x <= job.bounds.hi.x; ++x)
		{
			const size_t colOffset = static_cast<size_t>(x) * static_cast<size_t>(height);
			const UnsignedByte *srcCol = m_activeSnapshot + colOffset;
			UnsignedByte *dstCol = scratch.passability.data() + colOffset;
			for (Int y = job.bounds.lo.y; y <= job.bounds.hi.y; ++y)
			{
				dstCol[y] = decodePassabilityFromSnapshot(srcCol[y], isCrusher, surfaces);
			}
		}

		result.valid = runDialDijkstra(
			m_activeWidth,
			m_activeHeight,
			job.bounds,
			job.seedX,
			job.seedY,
			scratch.passability.data(),
			scratch,
			result.flowDir);

		const size_t doneCount = m_completedJobs.fetch_add(1, std::memory_order_acq_rel) + 1;
		if (doneCount == m_activeJobCount)
		{
			std::lock_guard<std::mutex> lock(m_poolMutex);
			m_cvDone.notify_one();
		}
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

std::vector<UnsignedByte> &PathfindFlowFieldManager::getSnapshotScratch(size_t totalCells)
{
	if (m_snapshotScratch.size() < totalCells)
	{
		m_snapshotScratch.resize(totalCells);
	}
	return m_snapshotScratch;
}

UnsignedInt PathfindFlowFieldManager::peekRecentDemand(const FlowFieldKey &key, UnsignedInt currentFrame) const
{
	for (Int i = 0; i < DEMAND_TABLE_SIZE; ++i)
	{
		const DemandSlot &slot = m_demand[i];
		if (slot.mapVersion == m_mapVersion && slot.count > 0 && slot.key == key)
		{
			if (currentFrame >= slot.lastFrame && (currentFrame - slot.lastFrame) <= DEMAND_WINDOW_FRAMES)
			{
				return slot.count;
			}
			return 0;
		}
	}
	return 0;
}

Bool PathfindFlowFieldManager::hasCached(const FlowFieldKey &key) const
{
	for (Int i = 0; i < CACHE_CAPACITY; ++i)
	{
		if (m_cache[i].mapVersion == m_mapVersion && m_cache[i].key == key)
		{
			return true;
		}
	}
	return false;
}

Bool PathfindFlowFieldManager::recordDemandAndShouldUse(const FlowFieldKey &key, UnsignedInt currentFrame)
{
	// Fast path: if an up-to-date flow field is already cached for this key, use it immediately.
	if (hasCached(key))
	{
		return true;
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

Bool PathfindFlowFieldManager::runDialDijkstra(
	Int width,
	Int height,
	const IRegion2D &bounds,
	Int seedX,
	Int seedY,
	const UnsignedByte *passabilityGrid,
	WorkerScratch &scratch,
	std::vector<signed char> &outFlowDir)
{
	if (width <= 0 || height <= 0 || passabilityGrid == nullptr)
	{
		return false;
	}

	if (seedX < bounds.lo.x || seedX > bounds.hi.x || seedY < bounds.lo.y || seedY > bounds.hi.y)
	{
		return false;
	}

	const size_t totalCells = static_cast<size_t>(width) * static_cast<size_t>(height);
	const size_t seedIndex = static_cast<size_t>(seedX) * static_cast<size_t>(height) + static_cast<size_t>(seedY);
	if (passabilityGrid[seedIndex] == PASS_BLOCKED)
	{
		return false;
	}

	outFlowDir.assign(totalCells, FLOW_DIR_NONE);
	scratch.costs.assign(totalCells, FLOW_COST_UNREACHABLE);

	size_t bucketHead[DIAL_BUCKET_COUNT];
	for (Int b = 0; b < DIAL_BUCKET_COUNT; ++b)
	{
		scratch.buckets[b].clear();
		bucketHead[b] = 0;
	}

	scratch.costs[seedIndex] = 0;
	outFlowDir[seedIndex] = FLOW_DIR_GOAL;
	scratch.buckets[0].push_back(packCell(seedX, seedY));

	UnsignedInt currentCost = 0;
	size_t queuedEntries = 1;
	const UnsignedInt bucketMask = DIAL_BUCKET_COUNT - 1;

	while (queuedEntries > 0)
	{
		UnsignedInt bucketIdx = currentCost & bucketMask;
		while (bucketHead[bucketIdx] >= scratch.buckets[bucketIdx].size())
		{
			scratch.buckets[bucketIdx].clear();
			bucketHead[bucketIdx] = 0;
			++currentCost;
			bucketIdx = currentCost & bucketMask;
		}

		const UnsignedInt packed = scratch.buckets[bucketIdx][bucketHead[bucketIdx]++];
		--queuedEntries;

		if (bucketHead[bucketIdx] == scratch.buckets[bucketIdx].size())
		{
			scratch.buckets[bucketIdx].clear();
			bucketHead[bucketIdx] = 0;
		}

		const Int ux = unpackX(packed);
		const Int uy = unpackY(packed);
		const size_t uIdx = static_cast<size_t>(ux) * static_cast<size_t>(height) + static_cast<size_t>(uy);

		if (scratch.costs[uIdx] < currentCost)
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

			if (newCost < scratch.costs[vIdx])
			{
				scratch.costs[vIdx] = newCost;
				outFlowDir[vIdx] = OPPOSITE_DIR[d];
				scratch.buckets[newCost & bucketMask].push_back(packCell(vx, vy));
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

			if (newCost < scratch.costs[vIdx])
			{
				scratch.costs[vIdx] = newCost;
				outFlowDir[vIdx] = OPPOSITE_DIR[d];
				scratch.buckets[newCost & bucketMask].push_back(packCell(vx, vy));
				++queuedEntries;
			}
		}
	}

	return true;
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
	FlowFieldEntry *entry = allocateSlot(key);
	if (!runDialDijkstra(width, height, bounds, seedX, seedY, passabilityGrid, m_workerScratches[0], entry->flowDir))
	{
		return nullptr;
	}

	entry->key = key;
	entry->mapVersion = m_mapVersion;
	entry->lastUsedSeq = ++m_lruClock;
	entry->seedX = seedX;
	entry->seedY = seedY;
	entry->width = width;
	entry->height = height;
	entry->bounds = bounds;
	return entry;
}

Int PathfindFlowFieldManager::buildFlowFieldsParallel(
	const ParallelBuildJob *jobs,
	size_t jobCount,
	Int width,
	Int height,
	const UnsignedByte *cellSnapshot)
{
	if (jobs == nullptr || jobCount == 0 || width <= 0 || height <= 0 || cellSnapshot == nullptr)
	{
		return 0;
	}

	if (jobCount > static_cast<size_t>(CACHE_CAPACITY))
	{
		jobCount = static_cast<size_t>(CACHE_CAPACITY);
	}

	if (m_parallelResults.size() < jobCount)
	{
		m_parallelResults.resize(jobCount);
	}

	m_activeJobs = jobs;
	m_activeJobCount = jobCount;
	m_activeWidth = width;
	m_activeHeight = height;
	m_activeSnapshot = cellSnapshot;
	m_nextJobIndex.store(0, std::memory_order_relaxed);
	m_completedJobs.store(0, std::memory_order_relaxed);

	if (jobCount > 1)
	{
		ensureWorkersStarted();
	}

	if (jobCount > 1 && !m_workers.empty())
	{
		{
			std::lock_guard<std::mutex> lock(m_poolMutex);
			++m_batchEpoch;
		}
		m_cvWork.notify_all();

		// Main thread also participates using scratch[0] so zero cores sit idle.
		executeBatchJobs(0);

		std::unique_lock<std::mutex> lock(m_poolMutex);
		m_cvDone.wait(lock, [this]()
		{
			return m_completedJobs.load(std::memory_order_acquire) >= m_activeJobCount;
		});
	}
	else
	{
		executeBatchJobs(0);
	}

	// Commit results on the main thread in exact deterministic job index order [0..jobCount-1].
	Int committed = 0;
	for (size_t i = 0; i < jobCount; ++i)
	{
		ParallelJobResult &res = m_parallelResults[i];
		if (!res.valid)
		{
			continue;
		}

		FlowFieldEntry *slot = allocateSlot(res.key);
		slot->key = res.key;
		slot->mapVersion = m_mapVersion;
		slot->lastUsedSeq = ++m_lruClock;
		slot->seedX = res.seedX;
		slot->seedY = res.seedY;
		slot->width = res.width;
		slot->height = res.height;
		slot->bounds = res.bounds;
		slot->flowDir.swap(res.flowDir);
		++committed;
	}

	m_activeJobs = nullptr;
	m_activeJobCount = 0;
	m_activeSnapshot = nullptr;
	return committed;
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
