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

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include "Common/GameType.h"
#include "GameLogic/LocomotorSet.h"

typedef UnsignedShort zoneStorageType;

/**
 * Deterministic LRU-cached Flow Field engine with lockstep-safe multithreaded batch building.
 *
 * When multiple units target the same destination cluster and terrain zone, a single
 * exact O(1)-bucket Dial's Dijkstra sweep builds a reverse flow direction grid for the
 * entire playable bounds. When multiple distinct Flow Fields are demanded in the same
 * frame (e.g. multi-front battles or post-structure-destruction invalidation), a persistent
 * worker thread pool computes them concurrently from an immutable cell snapshot and commits
 * results on the main thread in exact deterministic job order.
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
		DIAL_BUCKET_COUNT = 32,           ///< Power-of-2 bucket count (> max step cost of 24)
		MAX_WORKER_THREADS = 7            ///< Max background workers (+ main thread = up to 8 cores)
	};

	enum PassabilityState : UnsignedByte
	{
		PASS_BLOCKED = 0,
		PASS_CLEAR = 1,
		PASS_SOFT_PENALTY = 2             ///< Pinched or obstacle-adjacent clearance penalty
	};

	enum SnapshotCellBits : UnsignedByte
	{
		SNAP_TYPE_MASK = 0x0F,            ///< PathfindCell::CellType (0..6)
		SNAP_PINCHED   = 0x10,            ///< Cell is pinched
		SNAP_FENCE     = 0x20             ///< Cell is a crushable fence obstacle
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

	struct ParallelBuildJob
	{
		FlowFieldKey key;
		IRegion2D bounds;
		Int seedX;
		Int seedY;
	};

	PathfindFlowFieldManager();
	~PathfindFlowFieldManager();

	void reset();
	void invalidateCache();
	UnsignedInt getMapVersion() const { return m_mapVersion; }
	UnsignedInt getWorkerCount() const { return static_cast<UnsignedInt>(m_workers.size()); }

	/// Records demand for a key at the current frame. Returns true if a cached field exists
	/// or demand has reached ACTIVATION_THRESHOLD and a flow field should be built.
	Bool recordDemandAndShouldUse(const FlowFieldKey &key, UnsignedInt currentFrame);

	/// Returns current active demand count for key within DEMAND_WINDOW_FRAMES without mutating it.
	UnsignedInt peekRecentDemand(const FlowFieldKey &key, UnsignedInt currentFrame) const;

	/// Looks up an up-to-date cached flow field, refreshing its LRU sequence.
	const FlowFieldEntry *findCached(const FlowFieldKey &key);

	/// Non-mutating cache check used during queue pre-scan.
	Bool hasCached(const FlowFieldKey &key) const;

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

	/// Builds a batch of distinct flow fields in parallel across worker threads from a shared
	/// immutable cell snapshot (encoded with SnapshotCellBits), then commits results into the
	/// LRU cache in exact deterministic job order [0..jobCount-1].
	/// Returns number of valid flow fields committed.
	Int buildFlowFieldsParallel(
		const ParallelBuildJob *jobs,
		size_t jobCount,
		Int width,
		Int height,
		const UnsignedByte *cellSnapshot);

	/// Encodes a ground map cell into a compact snapshot byte for lock-free worker reads.
	static inline UnsignedByte encodeSnapshotCell(UnsignedByte cellType, Bool isPinched, Bool isFence)
	{
		return static_cast<UnsignedByte>((cellType & SNAP_TYPE_MASK)
			| (isPinched ? SNAP_PINCHED : 0)
			| (isFence ? SNAP_FENCE : 0));
	}

	/// Traces a compressed corner waypoint path from (startX, startY) to (goalX, goalY) using
	/// a built flow field. Waypoints are recorded at direction changes and end at (goalX, goalY).
	static Bool tracePath(
		const FlowFieldEntry &entry,
		Int startX,
		Int startY,
		Int goalX,
		Int goalY,
		std::vector<ICoord2D> &outWaypoints);

	/// Reusable scratch buffers for building grids without per-frame heap allocations.
	std::vector<UnsignedByte> &getPassabilityScratch(size_t totalCells);
	std::vector<UnsignedByte> &getSnapshotScratch(size_t totalCells);

private:
	struct DemandSlot
	{
		FlowFieldKey key;
		UnsignedInt lastFrame;
		UnsignedInt count;
		UnsignedInt mapVersion;

		DemandSlot() : lastFrame(0), count(0), mapVersion(0) {}
	};

	struct WorkerScratch
	{
		std::vector<UnsignedByte> passability;
		std::vector<UnsignedInt> costs;
		std::vector<UnsignedInt> buckets[DIAL_BUCKET_COUNT];
		char cacheLinePad[64];
	};

	struct ParallelJobResult
	{
		FlowFieldKey key;
		IRegion2D bounds;
		Int seedX;
		Int seedY;
		Int width;
		Int height;
		Bool valid;
		std::vector<signed char> flowDir;

		ParallelJobResult()
			: seedX(0)
			, seedY(0)
			, width(0)
			, height(0)
			, valid(false)
		{
			bounds.lo.x = bounds.lo.y = bounds.hi.x = bounds.hi.y = 0;
		}
	};

	FlowFieldEntry *allocateSlot(const FlowFieldKey &key);
	void ensureWorkersStarted();
	void shutdownWorkers();
	void workerLoop(size_t scratchIndex);
	void executeBatchJobs(size_t scratchIndex);

	static UnsignedByte decodePassabilityFromSnapshot(
		UnsignedByte snap,
		Bool isCrusher,
		LocomotorSurfaceTypeMask acceptableSurfaces);

	static Bool runDialDijkstra(
		Int width,
		Int height,
		const IRegion2D &bounds,
		Int seedX,
		Int seedY,
		const UnsignedByte *passabilityGrid,
		WorkerScratch &scratch,
		std::vector<signed char> &outFlowDir);

	FlowFieldEntry m_cache[CACHE_CAPACITY];
	DemandSlot m_demand[DEMAND_TABLE_SIZE];

	std::vector<UnsignedByte> m_passabilityScratch;
	std::vector<UnsignedByte> m_snapshotScratch;
	std::vector<WorkerScratch> m_workerScratches;
	std::vector<ParallelJobResult> m_parallelResults;

	std::vector<std::thread> m_workers;
	std::mutex m_poolMutex;
	std::condition_variable m_cvWork;
	std::condition_variable m_cvDone;
	Bool m_poolStarted;
	Bool m_poolShutdown;
	UnsignedInt m_batchEpoch;

	const ParallelBuildJob *m_activeJobs;
	size_t m_activeJobCount;
	Int m_activeWidth;
	Int m_activeHeight;
	const UnsignedByte *m_activeSnapshot;
	std::atomic<size_t> m_nextJobIndex;
	std::atomic<size_t> m_completedJobs;

	UnsignedInt m_mapVersion;
	UnsignedInt m_lruClock;
};
