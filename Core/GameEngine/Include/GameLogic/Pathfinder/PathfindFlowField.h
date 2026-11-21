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

#pragma once

#include <vector>
#include "Common/GameType.h"
#include "GameLogic/LocomotorSet.h"

typedef UnsignedShort zoneStorageType;

/**
 * Deterministic LRU-cached Flow Field engine for mass unit movement (100 - 1,000+ units).
 *
 * When multiple units target the same destination cluster and terrain zone, a single
 * exact O(1)-bucket Dial's Dijkstra sweep builds a reverse flow direction grid for the
 * entire playable bounds. Subsequent units trace paths in O(PathLength) without running
 * individual A* searches or allocating PathfindCellInfo nodes.
 */
class PathfindFlowFieldManager
{
public:
	enum
	{
		CLUSTER_SHIFT = 2,                ///< 4x4 cell destination bucket
		ACTIVATION_THRESHOLD = 4,         ///< Requests within window before building a flow field
		DEMAND_WINDOW_FRAMES = 60,        ///< Logic frame window for demand aggregation (~2 sec)
		CACHE_CAPACITY = 16,              ///< Number of cached flow fields in LRU cache
		DEMAND_TABLE_SIZE = 32,           ///< Demand tracker table capacity
		DIAL_BUCKET_COUNT = 32            ///< Power-of-2 bucket count (> max step cost of 24)
	};

	enum PassabilityState : UnsignedByte
	{
		PASS_BLOCKED = 0,
		PASS_CLEAR = 1,
		PASS_SOFT_PENALTY = 2             ///< Pinched or obstacle-adjacent clearance penalty
	};

	enum FlowDirection : signed char
	{
		FLOW_DIR_NONE = -1,
		FLOW_DIR_GOAL = -2
	};

	struct FlowFieldKey
	{
		Int goalClusterX;
		Int goalClusterY;
		zoneStorageType goalZone;
		LocomotorSurfaceTypeMask acceptableSurfaces;
		Bool isCrusher;
		Bool isHuman;

		FlowFieldKey()
			: goalClusterX(0)
			, goalClusterY(0)
			, goalZone(0)
			, acceptableSurfaces(0)
			, isCrusher(false)
			, isHuman(false)
		{
		}

		Bool operator==(const FlowFieldKey &other) const
		{
			return goalClusterX == other.goalClusterX
				&& goalClusterY == other.goalClusterY
				&& goalZone == other.goalZone
				&& acceptableSurfaces == other.acceptableSurfaces
				&& isCrusher == other.isCrusher
				&& isHuman == other.isHuman;
		}
	};

	struct FlowFieldEntry
	{
		FlowFieldKey key;
		UnsignedInt mapVersion;
		UnsignedInt lastUsedSeq;
		Int seedX;
		Int seedY;
		Int width;
		Int height;
		IRegion2D bounds;
		std::vector<signed char> flowDir; ///< Column-major [x * height + y] matching Pathfinder::m_map

		FlowFieldEntry()
			: mapVersion(0)
			, lastUsedSeq(0)
			, seedX(0)
			, seedY(0)
			, width(0)
			, height(0)
		{
			bounds.lo.x = bounds.lo.y = bounds.hi.x = bounds.hi.y = 0;
		}
	};

	PathfindFlowFieldManager();

	void reset();
	void invalidateCache();
	UnsignedInt getMapVersion() const { return m_mapVersion; }

	/// Records demand for a key at the current frame. Returns true if a cached field exists
	/// or demand has reached ACTIVATION_THRESHOLD and a flow field should be built.
	Bool recordDemandAndShouldUse(const FlowFieldKey &key, UnsignedInt currentFrame);

	/// Looks up an up-to-date cached flow field, refreshing its LRU sequence.
	const FlowFieldEntry *findCached(const FlowFieldKey &key);

	/// Builds (or replaces an LRU slot with) a deterministic flow field using Dial's 32-bucket Dijkstra.
	/// passabilityGrid is column-major [x * height + y] with PassabilityState values.
	const FlowFieldEntry *buildFlowField(
		const FlowFieldKey &key,
		Int width,
		Int height,
		const IRegion2D &bounds,
		Int seedX,
		Int seedY,
		const UnsignedByte *passabilityGrid);

	/// Traces a compressed corner waypoint path from (startX, startY) to (goalX, goalY) using
	/// a built flow field. Waypoints are recorded at direction changes and end at (goalX, goalY).
	static Bool tracePath(
		const FlowFieldEntry &entry,
		Int startX,
		Int startY,
		Int goalX,
		Int goalY,
		std::vector<ICoord2D> &outWaypoints);

	/// Reusable scratch buffer for building the compact passability grid without per-build allocations.
	std::vector<UnsignedByte> &getPassabilityScratch(size_t totalCells);

private:
	struct DemandSlot
	{
		FlowFieldKey key;
		UnsignedInt lastFrame;
		UnsignedInt count;
		UnsignedInt mapVersion;

		DemandSlot() : lastFrame(0), count(0), mapVersion(0) {}
	};

	FlowFieldEntry *allocateSlot(const FlowFieldKey &key);

	FlowFieldEntry m_cache[CACHE_CAPACITY];
	DemandSlot m_demand[DEMAND_TABLE_SIZE];

	std::vector<UnsignedByte> m_passabilityScratch;
	std::vector<UnsignedInt> m_costScratch;
	std::vector<UnsignedInt> m_buckets[DIAL_BUCKET_COUNT];

	UnsignedInt m_mapVersion;
	UnsignedInt m_lruClock;
};
