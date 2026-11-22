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

#include <gtest/gtest.h>
#include <chrono>
#include <cstdio>
#include <vector>

#include "Lib/BaseType.h"
#include "Common/Debug.h"
#include "Common/GameDefines.h"
#include "Common/GameType.h"
#include "GameLogic/Pathfinder/PathfindCell.h"
#include "GameLogic/Pathfinder/PathfindCellInfo.h"
#include "GameLogic/Pathfinder/PathfindCellList.h"
#include "GameLogic/Pathfinder/PathfindConstants.h"

#if !RETAIL_COMPATIBLE_PATHFINDING
#include "GameLogic/Pathfinder/PathfindFlowField.h"
#include "GameLogic/Pathfinder/PathfindOpenHeap.h"
#include "GameLogic/Pathfinder/PathfindRequestQueue.h"

namespace
{
	struct ScopedCellInfoPool
	{
		ScopedCellInfoPool() { PathfindCellInfo::allocateCellInfos(); }
		~ScopedCellInfoPool() { PathfindCellInfo::releaseCellInfos(); }
	};
}

TEST(PathfindOverhaul, OpenHeapOrderAndDeterministicTies)
{
	ScopedCellInfoPool pool;

	constexpr Int NUM_CELLS = 1000;
	std::vector<PathfindCell> cells(NUM_CELLS);
	PathfindOpenHeap heap;

	UnsignedInt rng = 0x12345678u;
	for (Int i = 0; i < NUM_CELLS; ++i)
	{
		rng = rng * 1664525u + 1013904223u;
		ICoord2D pos;
		pos.x = i;
		pos.y = 0;
		ASSERT_TRUE(cells[i].allocateInfo(pos));
		cells[i].startPathfind(nullptr);
		// Intentionally create frequent ties by clamping costs to [0..63]
		cells[i].setTotalCost((rng >> 16) & 63u);
		cells[i].putOnSortedOpenList(heap);
	}

	EXPECT_EQ(heap.size(), static_cast<size_t>(NUM_CELLS));

	UnsignedInt prevCost = 0;
	Int prevXForSameCost = -1;
	Int popped = 0;

	while (!heap.empty())
	{
		PathfindCell *top = heap.getHead();
		ASSERT_NE(top, nullptr);
		const UnsignedInt cost = top->getTotalCost();
		const Int x = top->getXIndex();
		top->removeFromOpenList(heap);
		top->releaseInfo();

		EXPECT_GE(cost, prevCost);
		if (cost == prevCost)
		{
			// Ties must break deterministically in FIFO insertion order (ascending x)
			EXPECT_GT(x, prevXForSameCost);
		}
		prevCost = cost;
		prevXForSameCost = x;
		++popped;
	}

	EXPECT_EQ(popped, NUM_CELLS);
}

TEST(PathfindOverhaul, OpenHeapArbitraryRemoveAndDecreaseKey)
{
	ScopedCellInfoPool pool;

	constexpr Int NUM_CELLS = 500;
	std::vector<PathfindCell> cells(NUM_CELLS);
	PathfindOpenHeap heap;

	for (Int i = 0; i < NUM_CELLS; ++i)
	{
		ICoord2D pos;
		pos.x = i;
		pos.y = 1;
		ASSERT_TRUE(cells[i].allocateInfo(pos));
		cells[i].startPathfind(nullptr);
		cells[i].setTotalCost(static_cast<UnsignedInt>((NUM_CELLS - i) * 10));
		cells[i].putOnSortedOpenList(heap);
	}

	// Remove every 5th cell from arbitrary heap positions, lower its cost, and re-insert
	for (Int i = 0; i < NUM_CELLS; i += 5)
	{
		cells[i].removeFromOpenList(heap);
		cells[i].setTotalCost(static_cast<UnsignedInt>(i));
		cells[i].putOnSortedOpenList(heap);
	}

	// Pop half and verify ascending order, then test releaseAll on the rest
	UnsignedInt prevCost = 0;
	for (Int k = 0; k < NUM_CELLS / 2; ++k)
	{
		PathfindCell *top = heap.getHead();
		ASSERT_NE(top, nullptr);
		const UnsignedInt cost = top->getTotalCost();
		EXPECT_GE(cost, prevCost);
		prevCost = cost;
		top->removeFromOpenList(heap);
		top->releaseInfo();
	}

	const Int released = PathfindCell::releaseOpenList(heap);
	EXPECT_EQ(released, NUM_CELLS - (NUM_CELLS / 2));
	EXPECT_TRUE(heap.empty());
}

TEST(PathfindOverhaul, OpenHeapBenchmarkVsLegacyLinkedList)
{
	ScopedCellInfoPool pool;

	constexpr Int NUM_CELLS = 2000;
	std::vector<PathfindCell> cells(NUM_CELLS);
	std::vector<UnsignedInt> costs(NUM_CELLS);

	UnsignedInt rng = 0xCAFEBABEu;
	for (Int i = 0; i < NUM_CELLS; ++i)
	{
		rng = rng * 1664525u + 1013904223u;
		costs[i] = (rng >> 16) % 4096u;
	}

	// Benchmark new 4-ary heap
	PathfindOpenHeap heap;
	auto t0 = std::chrono::high_resolution_clock::now();
	for (Int i = 0; i < NUM_CELLS; ++i)
	{
		ICoord2D pos = { i, 0 };
		cells[i].allocateInfo(pos);
		cells[i].startPathfind(nullptr);
		cells[i].setTotalCost(costs[i]);
		heap.push(&cells[i]);
	}
	while (!heap.empty())
	{
		PathfindCell *top = heap.getHead();
		heap.remove(top);
		top->releaseInfo();
	}
	auto t1 = std::chrono::high_resolution_clock::now();

	// Benchmark legacy doubly-linked list insertion sort
	PathfindCellList legacyList;
	auto t2 = std::chrono::high_resolution_clock::now();
	for (Int i = 0; i < NUM_CELLS; ++i)
	{
		ICoord2D pos = { i, 0 };
		cells[i].allocateInfo(pos);
		cells[i].startPathfind(nullptr);
		cells[i].setTotalCost(costs[i]);
		if (legacyList.canReverseSort(cells[i]))
			cells[i].reverseInsertionSort(legacyList);
		else
			cells[i].forwardInsertionSort(legacyList);
	}
	// Clean legacy list by walking head
	while (!legacyList.empty())
	{
		PathfindCell *cur = legacyList.getHead();
		PathfindCell *next = cur->getNextOpen();
		cur->setOpenFlag(false);
		// Clear linked list pointers via startPathfind reset before release
		cur->startPathfind(nullptr);
		cur->releaseInfo();
		if (next)
		{
			// Detach head in test cleanup
			legacyList.reset();
			// Walk remaining via next chain
			while (next)
			{
				PathfindCell *after = next->getNextOpen();
				next->startPathfind(nullptr);
				next->releaseInfo();
				next = after;
			}
		}
		else
		{
			legacyList.reset();
		}
	}
	auto t3 = std::chrono::high_resolution_clock::now();

	const double heapUs = std::chrono::duration<double, std::micro>(t1 - t0).count();
	const double legacyUs = std::chrono::duration<double, std::micro>(t3 - t2).count();
	std::printf("[BENCH] 2,000-node OpenSet: 4-ary Heap = %.1f us vs Legacy LinkedList = %.1f us (%.1fx faster)\n",
		heapUs, legacyUs, legacyUs / (heapUs > 0.1 ? heapUs : 0.1));

	EXPECT_LT(heapUs, legacyUs);
}

TEST(PathfindOverhaul, RequestQueueUnboundedDeduplicatedFIFO)
{
	PathfindRequestQueue queue;

	// Enqueue 5,000 distinct unit IDs (well beyond old 512 limit)
	constexpr UnsignedInt INITIAL_COUNT = 5000;
	for (UnsignedInt i = 1; i <= INITIAL_COUNT; ++i)
	{
		ASSERT_TRUE(queue.enqueue(static_cast<ObjectID>(i)));
	}
	EXPECT_EQ(queue.size(), INITIAL_COUNT);

	// Re-enqueue duplicates; size must not grow
	for (UnsignedInt i = 1; i <= INITIAL_COUNT; i += 2)
	{
		ASSERT_TRUE(queue.enqueue(static_cast<ObjectID>(i)));
	}
	EXPECT_EQ(queue.size(), INITIAL_COUNT);

	// Dequeue first 2,500 in exact FIFO order (tests backward-shift hash deletion)
	for (UnsignedInt i = 1; i <= 2500; ++i)
	{
		ObjectID id = INVALID_ID;
		ASSERT_TRUE(queue.dequeue(id));
		EXPECT_EQ(id, static_cast<ObjectID>(i));
		EXPECT_FALSE(queue.contains(static_cast<ObjectID>(i)));
	}
	EXPECT_EQ(queue.size(), 2500u);

	// Re-enqueue previously dequeued IDs + 3,000 brand-new IDs (tests wrap-around & growth)
	for (UnsignedInt i = 1; i <= 1000; ++i)
	{
		ASSERT_TRUE(queue.enqueue(static_cast<ObjectID>(i)));
	}
	for (UnsignedInt i = INITIAL_COUNT + 1; i <= INITIAL_COUNT + 3000; ++i)
	{
		ASSERT_TRUE(queue.enqueue(static_cast<ObjectID>(i)));
	}
	EXPECT_EQ(queue.size(), 6500u);

	// Verify exact remaining FIFO sequence: 2501..5000, then 1..1000, then 5001..8000
	for (UnsignedInt expected = 2501; expected <= 5000; ++expected)
	{
		ObjectID id = INVALID_ID;
		ASSERT_TRUE(queue.dequeue(id));
		EXPECT_EQ(id, static_cast<ObjectID>(expected));
	}
	for (UnsignedInt expected = 1; expected <= 1000; ++expected)
	{
		ObjectID id = INVALID_ID;
		ASSERT_TRUE(queue.dequeue(id));
		EXPECT_EQ(id, static_cast<ObjectID>(expected));
	}
	for (UnsignedInt expected = 5001; expected <= 8000; ++expected)
	{
		ObjectID id = INVALID_ID;
		ASSERT_TRUE(queue.dequeue(id));
		EXPECT_EQ(id, static_cast<ObjectID>(expected));
	}
	EXPECT_TRUE(queue.empty());
}

TEST(PathfindOverhaul, FlowFieldMazeNavigationAndCacheInvalidation)
{
	PathfindFlowFieldManager mgr;
	constexpr Int W = 64;
	constexpr Int H = 64;
	IRegion2D bounds;
	bounds.lo.x = 0;
	bounds.lo.y = 0;
	bounds.hi.x = W - 1;
	bounds.hi.y = H - 1;

	std::vector<UnsignedByte> grid(W * H, PathfindFlowFieldManager::PASS_CLEAR);

	// Vertical wall at x=20 with opening only at y=55..60
	for (Int y = 0; y < 55; ++y)
	{
		grid[20 * H + y] = PathfindFlowFieldManager::PASS_BLOCKED;
	}
	// Second vertical wall at x=40 with opening only at y=4..9
	for (Int y = 10; y < H; ++y)
	{
		grid[40 * H + y] = PathfindFlowFieldManager::PASS_BLOCKED;
	}
	// Completely walled-off unreachable room at x=2..8, y=2..8
	for (Int x = 2; x <= 8; ++x)
	{
		grid[x * H + 2] = PathfindFlowFieldManager::PASS_BLOCKED;
		grid[x * H + 8] = PathfindFlowFieldManager::PASS_BLOCKED;
	}
	for (Int y = 2; y <= 8; ++y)
	{
		grid[2 * H + y] = PathfindFlowFieldManager::PASS_BLOCKED;
		grid[8 * H + y] = PathfindFlowFieldManager::PASS_BLOCKED;
	}

	PathfindFlowFieldManager::FlowFieldKey key;
	key.goalClusterX = 55 >> PathfindFlowFieldManager::CLUSTER_SHIFT;
	key.goalClusterY = 55 >> PathfindFlowFieldManager::CLUSTER_SHIFT;
	key.goalZone = 1;
	key.acceptableSurfaces = LOCOMOTORSURFACE_GROUND;
	key.isCrusher = false;
	key.isHuman = true;

	// Verify demand threshold activation (3 requests -> false, 4th -> true)
	EXPECT_FALSE(mgr.recordDemandAndShouldUse(key, 100));
	EXPECT_FALSE(mgr.recordDemandAndShouldUse(key, 105));
	EXPECT_FALSE(mgr.recordDemandAndShouldUse(key, 110));
	EXPECT_TRUE(mgr.recordDemandAndShouldUse(key, 115));

	const PathfindFlowFieldManager::FlowFieldEntry *entry = mgr.buildFlowField(key, W, H, bounds, 55, 55, grid.data());
	ASSERT_NE(entry, nullptr);
	EXPECT_EQ(mgr.findCached(key), entry);

	// Trace from (5, 25) -> must navigate around both maze walls to (55, 55)
	std::vector<ICoord2D> path;
	ASSERT_TRUE(PathfindFlowFieldManager::tracePath(*entry, 5, 25, 55, 55, path));
	ASSERT_FALSE(path.empty());
	EXPECT_EQ(path.back().x, 55);
	EXPECT_EQ(path.back().y, 55);

	// Verify every waypoint lies on a passable cell
	for (size_t i = 0; i < path.size(); ++i)
	{
		EXPECT_NE(grid[path[i].x * H + path[i].y], PathfindFlowFieldManager::PASS_BLOCKED);
	}

	// Trace from inside the sealed room (5, 5) must fail cleanly
	std::vector<ICoord2D> blockedPath;
	EXPECT_FALSE(PathfindFlowFieldManager::tracePath(*entry, 5, 5, 55, 55, blockedPath));

	// Cache invalidation must evict stale entry immediately
	mgr.invalidateCache();
	EXPECT_EQ(mgr.findCached(key), nullptr);
}

TEST(PathfindOverhaul, FlowFieldBenchmark1000UnitsOn400x400Map)
{
	PathfindFlowFieldManager mgr;
	constexpr Int W = 400;
	constexpr Int H = 400;
	IRegion2D bounds;
	bounds.lo.x = 0;
	bounds.lo.y = 0;
	bounds.hi.x = W - 1;
	bounds.hi.y = H - 1;

	std::vector<UnsignedByte> grid(W * H, PathfindFlowFieldManager::PASS_CLEAR);

	// Add realistic base walls and choke points across the 400x400 map
	for (Int y = 0; y < 340; ++y)
	{
		if (y < 140 || y > 170)
		{
			grid[130 * H + y] = PathfindFlowFieldManager::PASS_BLOCKED;
		}
	}
	for (Int y = 60; y < H; ++y)
	{
		if (y < 230 || y > 260)
		{
			grid[270 * H + y] = PathfindFlowFieldManager::PASS_BLOCKED;
		}
	}

	PathfindFlowFieldManager::FlowFieldKey key;
	key.goalClusterX = 360 >> PathfindFlowFieldManager::CLUSTER_SHIFT;
	key.goalClusterY = 360 >> PathfindFlowFieldManager::CLUSTER_SHIFT;
	key.goalZone = 1;
	key.acceptableSurfaces = LOCOMOTORSURFACE_GROUND;
	key.isCrusher = false;
	key.isHuman = true;

	auto tBuildStart = std::chrono::high_resolution_clock::now();
	const PathfindFlowFieldManager::FlowFieldEntry *entry = mgr.buildFlowField(key, W, H, bounds, 360, 360, grid.data());
	auto tBuildEnd = std::chrono::high_resolution_clock::now();
	ASSERT_NE(entry, nullptr);

	// Trace 1,000 units distributed across the left/middle map heading to (360, 360)
	std::vector<ICoord2D> waypoints;
	waypoints.reserve(64);
	Int tracedCount = 0;
	UnsignedInt rng = 0xDEADBEEFu;

	auto tTraceStart = std::chrono::high_resolution_clock::now();
	for (Int u = 0; u < 1000; ++u)
	{
		rng = rng * 1664525u + 1013904223u;
		Int sx = static_cast<Int>((rng >> 16) % 120u) + 5;
		rng = rng * 1664525u + 1013904223u;
		Int sy = static_cast<Int>((rng >> 16) % 380u) + 10;
		Int gx = 360 + (u % 3) - 1;
		Int gy = 360 + ((u / 3) % 3) - 1;

		if (PathfindFlowFieldManager::tracePath(*entry, sx, sy, gx, gy, waypoints))
		{
			++tracedCount;
		}
	}
	auto tTraceEnd = std::chrono::high_resolution_clock::now();

	const double buildMs = std::chrono::duration<double, std::milli>(tBuildEnd - tBuildStart).count();
	const double traceMs = std::chrono::duration<double, std::milli>(tTraceEnd - tTraceStart).count();
	std::printf("[BENCH] 400x400 Map (160,000 cells): FlowField Build = %.2f ms, 1,000 Unit Traces = %.2f ms (%.2f us/unit)\n",
		buildMs, traceMs, (traceMs * 1000.0) / 1000.0);

	EXPECT_EQ(tracedCount, 1000);
	EXPECT_LT(buildMs + traceMs, 100.0);
}

TEST(PathfindOverhaul, ParallelFlowFieldsBitExactDeterminismAndSpeedup)
{
	constexpr Int W = 400;
	constexpr Int H = 400;
	constexpr size_t JOB_COUNT = 8;

	IRegion2D bounds;
	bounds.lo.x = 0;
	bounds.lo.y = 0;
	bounds.hi.x = W - 1;
	bounds.hi.y = H - 1;

	std::vector<UnsignedByte> snapshot(W * H,
		PathfindFlowFieldManager::encodeSnapshotCell(PathfindCell::CELL_CLEAR, false, false));

	// Build multi-chokepoint obstacles + pinched borders + crushable fences
	for (Int y = 0; y < H; ++y)
	{
		if (y < 120 || (y > 160 && y < 280) || y > 320)
		{
			snapshot[140 * H + y] = PathfindFlowFieldManager::encodeSnapshotCell(PathfindCell::CELL_OBSTACLE, false, false);
		}
		if (y > 40 && y < 360 && (y < 190 || y > 220))
		{
			snapshot[260 * H + y] = PathfindFlowFieldManager::encodeSnapshotCell(PathfindCell::CELL_OBSTACLE, false, true);
		}
	}

	PathfindFlowFieldManager::ParallelBuildJob jobs[JOB_COUNT];
	const Int seedsX[JOB_COUNT] = { 30, 370, 30, 370, 200, 200, 80, 320 };
	const Int seedsY[JOB_COUNT] = { 30, 30, 370, 370, 40, 360, 200, 200 };

	for (size_t j = 0; j < JOB_COUNT; ++j)
	{
		jobs[j].bounds = bounds;
		jobs[j].seedX = seedsX[j];
		jobs[j].seedY = seedsY[j];
		jobs[j].key.goalClusterX = seedsX[j] >> PathfindFlowFieldManager::CLUSTER_SHIFT;
		jobs[j].key.goalClusterY = seedsY[j] >> PathfindFlowFieldManager::CLUSTER_SHIFT;
		jobs[j].key.goalZone = 1;
		jobs[j].key.acceptableSurfaces = LOCOMOTORSURFACE_GROUND;
		jobs[j].key.isCrusher = (j & 1) != 0;
		jobs[j].key.isHuman = true;
	}

	// 1. Build all 8 serially as ground truth
	PathfindFlowFieldManager serialMgr;
	std::vector<std::vector<signed char>> serialDirs(JOB_COUNT);
	std::vector<UnsignedByte> passGrid(W * H);

	auto tSerial0 = std::chrono::high_resolution_clock::now();
	for (size_t j = 0; j < JOB_COUNT; ++j)
	{
		const Bool crusher = jobs[j].key.isCrusher;
		for (size_t idx = 0; idx < static_cast<size_t>(W * H); ++idx)
		{
			const UnsignedByte snap = snapshot[idx];
			const UnsignedByte type = snap & PathfindFlowFieldManager::SNAP_TYPE_MASK;
			const Bool fence = (snap & PathfindFlowFieldManager::SNAP_FENCE) != 0;
			const Bool pass = (type == PathfindCell::CELL_CLEAR) || (crusher && fence);
			passGrid[idx] = pass ? PathfindFlowFieldManager::PASS_CLEAR : PathfindFlowFieldManager::PASS_BLOCKED;
		}
		const PathfindFlowFieldManager::FlowFieldEntry *e = serialMgr.buildFlowField(
			jobs[j].key, W, H, bounds, jobs[j].seedX, jobs[j].seedY, passGrid.data());
		ASSERT_NE(e, nullptr);
		serialDirs[j] = e->flowDir;
	}
	auto tSerial1 = std::chrono::high_resolution_clock::now();

	// 2. Build all 8 in parallel across worker pool (warm up workers once first, then measure steady-state)
	PathfindFlowFieldManager parallelMgr;
	ASSERT_EQ(parallelMgr.buildFlowFieldsParallel(jobs, JOB_COUNT, W, H, snapshot.data()), static_cast<Int>(JOB_COUNT));
	parallelMgr.invalidateCache();

	auto tPar0 = std::chrono::high_resolution_clock::now();
	const Int committed = parallelMgr.buildFlowFieldsParallel(jobs, JOB_COUNT, W, H, snapshot.data());
	auto tPar1 = std::chrono::high_resolution_clock::now();
	ASSERT_EQ(committed, static_cast<Int>(JOB_COUNT));

	// 3. Verify 100% bit-for-bit determinism across all 8 x 160,000 = 1,280,000 cells!
	for (size_t j = 0; j < JOB_COUNT; ++j)
	{
		const PathfindFlowFieldManager::FlowFieldEntry *pe = parallelMgr.findCached(jobs[j].key);
		ASSERT_NE(pe, nullptr);
		ASSERT_EQ(pe->flowDir.size(), serialDirs[j].size());
		EXPECT_EQ(pe->flowDir, serialDirs[j]) << "Mismatch on job " << j;
	}

	const double serialMs = std::chrono::duration<double, std::milli>(tSerial1 - tSerial0).count();
	const double parallelMs = std::chrono::duration<double, std::milli>(tPar1 - tPar0).count();
	std::printf("[BENCH] 8 Flow Fields (400x400): Serial = %.2f ms vs Parallel (%u threads) = %.2f ms (%.2fx faster)\n",
		serialMs, parallelMgr.getWorkerCount() + 1u, parallelMs, serialMs / (parallelMs > 0.01 ? parallelMs : 0.01));
}

TEST(PathfindOverhaul, PartitionBitsetScanSpeedupBenchmark)
{
	// Simulates 1,000 units scanning a 15-cell radius across a 200x200 grid of 216-byte PartitionCells (~8.6 MB)
	// comparing our 5 KB L1-cache occupancy bitset + compact head table vs naive 216-byte cell loads.
	struct FatCell
	{
		void *firstCoi;
		char shroudThreatCashPadding[212];
	};
	static_assert(sizeof(FatCell) == 216, "Match PartitionCell cache footprint");

	constexpr Int GRID_W = 200;
	constexpr Int GRID_H = 200;
	constexpr Int TOTAL = GRID_W * GRID_H;
	constexpr Int RADIUS = 15;

	std::vector<FatCell> fatCells(TOTAL);
	std::vector<UnsignedInt> nonEmptyBits((TOTAL + 31) >> 5, 0u);
	std::vector<void *> coiHeads(TOTAL, nullptr);

	UnsignedInt rng = 0x517cc1b7u;
	for (Int i = 0; i < TOTAL; ++i)
	{
		rng = rng * 1664525u + 1013904223u;
		const Bool occupied = ((rng >> 24) < 13u); // ~5% cells occupied (typical 1,000 units on 40,000 cells)
		void *ptr = occupied ? reinterpret_cast<void *>(static_cast<uintptr_t>(i + 1)) : nullptr;
		fatCells[i].firstCoi = ptr;
		coiHeads[i] = ptr;
		if (occupied)
		{
			nonEmptyBits[i >> 5] |= (1u << (i & 31));
		}
	}

	std::vector<ICoord2D> offsets;
	for (Int dy = -RADIUS; dy <= RADIUS; ++dy)
	{
		for (Int dx = -RADIUS; dx <= RADIUS; ++dx)
		{
			if (dx * dx + dy * dy <= RADIUS * RADIUS)
			{
				ICoord2D o = { dx, dy };
				offsets.push_back(o);
			}
		}
	}

	std::vector<ICoord2D> queries(1000);
	for (Int q = 0; q < 1000; ++q)
	{
		rng = rng * 1664525u + 1013904223u;
		queries[q].x = static_cast<Int>((rng >> 16) % static_cast<UnsignedInt>(GRID_W));
		rng = rng * 1664525u + 1013904223u;
		queries[q].y = static_cast<Int>((rng >> 16) % static_cast<UnsignedInt>(GRID_H));
	}

	// Naive 216-byte cell scan
	volatile uintptr_t naiveSum = 0;
	auto t0 = std::chrono::high_resolution_clock::now();
	for (Int q = 0; q < 1000; ++q)
	{
		const Int cx0 = queries[q].x;
		const Int cy0 = queries[q].y;
		for (size_t oi = 0; oi < offsets.size(); ++oi)
		{
			const Int cx = cx0 + offsets[oi].x;
			const Int cy = cy0 + offsets[oi].y;
			if (cx < 0 || cy < 0 || cx >= GRID_W || cy >= GRID_H)
				continue;
			void *head = fatCells[cy * GRID_W + cx].firstCoi;
			if (head)
				naiveSum += reinterpret_cast<uintptr_t>(head);
		}
	}
	auto t1 = std::chrono::high_resolution_clock::now();

	// L1-cache bitset + compact COI head scan
	volatile uintptr_t fastSum = 0;
	const UnsignedInt uW = static_cast<UnsignedInt>(GRID_W);
	const UnsignedInt uH = static_cast<UnsignedInt>(GRID_H);
	auto t2 = std::chrono::high_resolution_clock::now();
	for (Int q = 0; q < 1000; ++q)
	{
		const Int cx0 = queries[q].x;
		const Int cy0 = queries[q].y;
		for (size_t oi = 0; oi < offsets.size(); ++oi)
		{
			const Int cx = cx0 + offsets[oi].x;
			const Int cy = cy0 + offsets[oi].y;
			if (static_cast<UnsignedInt>(cx) >= uW || static_cast<UnsignedInt>(cy) >= uH)
				continue;
			const Int idx = cy * GRID_W + cx;
			if ((nonEmptyBits[idx >> 5] & (1u << (idx & 31))) == 0)
				continue;
			fastSum += reinterpret_cast<uintptr_t>(coiHeads[idx]);
		}
	}
	auto t3 = std::chrono::high_resolution_clock::now();

	EXPECT_EQ(fastSum, naiveSum);
	const double naiveMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
	const double fastMs = std::chrono::duration<double, std::milli>(t3 - t2).count();
	std::printf("[BENCH] 1,000 Partition GCO Range Scans (200x200 grid): Naive = %.2f ms vs L1-Bitset = %.2f ms (%.2fx faster)\n",
		naiveMs, fastMs, naiveMs / (fastMs > 0.01 ? fastMs : 0.01));
}
#endif
