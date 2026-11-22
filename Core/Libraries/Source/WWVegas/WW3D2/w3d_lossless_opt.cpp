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

#include "w3d_lossless_opt.h"

#include "WWMath/vector3.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace W3DLosslessOpt
{
namespace
{
inline uint32_t CanonicalFloatBits(float f) noexcept
{
	if (f == 0.0f)
	{
		return 0u; // Canonicalize -0.0f to +0.0f so (a - b).Length2() == 0 matches identically
	}
	uint32_t bits = 0u;
	std::memcpy(&bits, &f, sizeof(bits));
	return bits;
}

struct ExactVec3Key
{
	uint32_t x;
	uint32_t y;
	uint32_t z;

	bool operator==(const ExactVec3Key& o) const noexcept
	{
		return x == o.x && y == o.y && z == o.z;
	}
};

struct ExactVec3KeyHash
{
	size_t operator()(const ExactVec3Key& k) const noexcept
	{
		uint64_t h = static_cast<uint64_t>(k.x) * 0x9E3779B185EBCA87ull;
		h ^= static_cast<uint64_t>(k.y) * 0xC2B2AE3D27D4EB4Full;
		h ^= static_cast<uint64_t>(k.z) * 0x165667B19E3779F9ull;
		h ^= (h >> 33);
		h *= 0xFF51AFD7ED558CCDull;
		h ^= (h >> 33);
		return static_cast<size_t>(h);
	}
};

// ============================================================================
// Tom Forsyth Vertex Cache Constants
// ============================================================================
constexpr unsigned int kForsythCacheSize = 32u;
constexpr float kCacheDecayPower = 1.5f;
constexpr float kLastTriScore = 0.75f;
constexpr float kValenceBoostScale = 2.0f;
constexpr float kValenceBoostPower = 0.5f;

inline float ComputeVertexScore(int cachePos, unsigned int activeTris) noexcept
{
	if (activeTris == 0u)
	{
		return -1.0f;
	}

	float score = 0.0f;
	if (cachePos >= 0)
	{
		if (cachePos < 3)
		{
			score = kLastTriScore;
		}
		else
		{
			constexpr float scaler = 1.0f / static_cast<float>(kForsythCacheSize - 3u);
			const float normPos = 1.0f - static_cast<float>(cachePos - 3) * scaler;
			score = std::pow(normPos, kCacheDecayPower);
		}
	}

	score += kValenceBoostScale * std::pow(static_cast<float>(activeTris), -kValenceBoostPower);
	return score;
}

// ============================================================================
// Persistent Worker Pool for CPU Shadow Volume Extrusion
// ============================================================================
class ShadowWorkerPool
{
public:
	static ShadowWorkerPool& Instance()
	{
		static ShadowWorkerPool pool;
		return pool;
	}

	unsigned int ThreadCount() const noexcept
	{
		return m_numWorkers + 1u;
	}

	void ParallelFor(size_t count, const std::function<void(size_t, size_t)>& fn, size_t minItemsPerTask)
	{
		if (count == 0)
		{
			return;
		}
		const size_t minChunk = (std::max)(size_t(1), minItemsPerTask);
		if (m_numWorkers == 0 || count <= minChunk)
		{
			fn(0, count);
			return;
		}

		// Serialize outer entry cleanly (always called from main render thread)
		std::lock_guard<std::mutex> callGuard(m_callMutex);

		EnsureStarted();

		const size_t totalThreads = static_cast<size_t>(m_numWorkers) + 1u;
		const size_t chunk = (std::max)(minChunk, (count + totalThreads - 1u) / totalThreads);
		const size_t numTasks = (count + chunk - 1u) / chunk;
		if (numTasks <= 1u)
		{
			fn(0, count);
			return;
		}

		{
			std::lock_guard<std::mutex> lk(m_mutex);
			m_currentFn = &fn;
			m_totalCount = count;
			m_chunkSize = chunk;
			m_nextStart.store(0, std::memory_order_relaxed);
			m_remainingWorkers.store(m_numWorkers, std::memory_order_relaxed);
			++m_generation;
		}
		m_cvWork.notify_all();

		// Main thread also participates in stealing chunks
		ExecuteChunks(fn, count, chunk);

		// Wait for all worker threads to complete this generation
		std::unique_lock<std::mutex> lk(m_mutex);
		m_cvDone.wait(lk, [this]() { return m_remainingWorkers.load(std::memory_order_acquire) == 0u; });
		m_currentFn = nullptr;
	}

private:
	ShadowWorkerPool()
	{
		const unsigned int hw = std::thread::hardware_concurrency();
		const unsigned int desiredTotal = (hw > 1u) ? (std::min)(hw, 8u) : 1u;
		m_numWorkers = (desiredTotal > 1u) ? (desiredTotal - 1u) : 0u;
	}

	~ShadowWorkerPool()
	{
		if (!m_started)
		{
			return;
		}
		{
			std::lock_guard<std::mutex> lk(m_mutex);
			m_stop = true;
			++m_generation;
		}
		m_cvWork.notify_all();
		for (auto& t : m_workers)
		{
			if (t.joinable())
			{
				t.join();
			}
		}
	}

	void EnsureStarted()
	{
		if (m_started || m_numWorkers == 0u)
		{
			return;
		}
		m_started = true;
		m_workers.reserve(m_numWorkers);
		for (unsigned int i = 0; i < m_numWorkers; ++i)
		{
			m_workers.emplace_back([this]() { WorkerLoop(); });
		}
	}

	void ExecuteChunks(const std::function<void(size_t, size_t)>& fn, size_t totalCount, size_t chunkSize)
	{
		for (;;)
		{
			const size_t start = m_nextStart.fetch_add(chunkSize, std::memory_order_relaxed);
			if (start >= totalCount)
			{
				break;
			}
			const size_t end = (std::min)(start + chunkSize, totalCount);
			fn(start, end);
		}
	}

	void WorkerLoop()
	{
		uint64_t seenGen = 0;
		for (;;)
		{
			const std::function<void(size_t, size_t)>* fn = nullptr;
			size_t totalCount = 0;
			size_t chunkSize = 1;
			{
				std::unique_lock<std::mutex> lk(m_mutex);
				m_cvWork.wait(lk, [this, seenGen]() { return m_generation != seenGen; });
				if (m_stop)
				{
					return;
				}
				seenGen = m_generation;
				fn = m_currentFn;
				totalCount = m_totalCount;
				chunkSize = m_chunkSize;
			}

			if (fn)
			{
				ExecuteChunks(*fn, totalCount, chunkSize);
			}

			if (m_remainingWorkers.fetch_sub(1u, std::memory_order_acq_rel) == 1u)
			{
				std::lock_guard<std::mutex> lk(m_mutex);
				m_cvDone.notify_one();
			}
		}
	}

	unsigned int m_numWorkers = 0;
	bool m_started = false;
	bool m_stop = false;
	uint64_t m_generation = 0;

	std::mutex m_callMutex;
	std::mutex m_mutex;
	std::condition_variable m_cvWork;
	std::condition_variable m_cvDone;
	std::vector<std::thread> m_workers;

	const std::function<void(size_t, size_t)>* m_currentFn = nullptr;
	size_t m_totalCount = 0;
	size_t m_chunkSize = 1;
	std::atomic<size_t> m_nextStart{ 0 };
	std::atomic<unsigned int> m_remainingWorkers{ 0 };
};
} // namespace

int WeldVerticesExact(
	const Vector3* verts,
	int numVerts,
	uint16_t* outParentVerts)
{
	if (!verts || !outParentVerts || numVerts <= 0)
	{
		return 0;
	}

	std::unordered_map<ExactVec3Key, uint16_t, ExactVec3KeyHash> firstOccurrence;
	firstOccurrence.reserve(static_cast<size_t>(numVerts));

	int uniqueCount = 0;
	for (int i = 0; i < numVerts; ++i)
	{
		const ExactVec3Key key{
			CanonicalFloatBits(verts[i].X),
			CanonicalFloatBits(verts[i].Y),
			CanonicalFloatBits(verts[i].Z)
		};
		auto it = firstOccurrence.find(key);
		if (it != firstOccurrence.end())
		{
			outParentVerts[i] = it->second;
		}
		else
		{
			const uint16_t idx = static_cast<uint16_t>(i);
			firstOccurrence.emplace(key, idx);
			outParentVerts[i] = idx;
			++uniqueCount;
		}
	}
	return uniqueCount;
}

void BuildPolygonNeighborsExact(
	int numPolys,
	const uint16_t* rawTriIndices,
	const uint16_t* parentVerts,
	const Vector3* polyNormals,
	ShadowPolyNeighbor* outNeighbors)
{
	if (numPolys <= 0 || !rawTriIndices || !polyNormals || !outNeighbors)
	{
		return;
	}

	constexpr int kNoNeighbor = -1;
	constexpr uint64_t kEmptyKey = ~uint64_t(0);

	// Initialize output table
	for (int i = 0; i < numPolys; ++i)
	{
		outNeighbors[i].myIndex = i;
		outNeighbors[i].status = 0;
		for (int a = 0; a < 3; ++a)
		{
			outNeighbors[i].neighbor[a].neighborIndex = kNoNeighbor;
			outNeighbors[i].neighbor[a].neighborEdgeIndex[0] = 0;
			outNeighbors[i].neighbor[a].neighborEdgeIndex[1] = 0;
		}
	}

	auto fetchPoly = [rawTriIndices, parentVerts](int polyId, int outPoly[3]) noexcept
	{
		const uint16_t i0 = rawTriIndices[polyId * 3 + 0];
		const uint16_t i1 = rawTriIndices[polyId * 3 + 1];
		const uint16_t i2 = rawTriIndices[polyId * 3 + 2];
		if (parentVerts)
		{
			outPoly[0] = static_cast<int>(parentVerts[i0]);
			outPoly[1] = static_cast<int>(parentVerts[i1]);
			outPoly[2] = static_cast<int>(parentVerts[i2]);
		}
		else
		{
			outPoly[0] = static_cast<int>(i0);
			outPoly[1] = static_cast<int>(i1);
			outPoly[2] = static_cast<int>(i2);
		}
	};

	auto hashDirectedEdge = [](uint64_t key) noexcept -> size_t
	{
		uint64_t h = key ^ (key >> 33);
		h *= 0xFF51AFD7ED558CCDull;
		h ^= (h >> 33);
		return static_cast<size_t>(h);
	};

	// Build flat open-addressing hash map from directed half-edge (u -> v) to polygon indices
	const size_t numHalfEdges = static_cast<size_t>(numPolys) * 3u;
	size_t tableSize = 16u;
	while (tableSize < numHalfEdges * 2u)
	{
		tableSize <<= 1u;
	}
	const size_t tableMask = tableSize - 1u;

	std::vector<uint64_t> slotKey(tableSize, kEmptyKey);
	std::vector<int> slotHead(tableSize, -1);
	std::vector<int> entryPoly(numHalfEdges, -1);
	std::vector<int> entryNext(numHalfEdges, -1);

	for (int j = 0; j < numPolys; ++j)
	{
		int poly[3];
		fetchPoly(j, poly);

		for (int e = 0; e < 3; ++e)
		{
			const uint32_t u = static_cast<uint32_t>(poly[e]);
			const uint32_t v = static_cast<uint32_t>(poly[(e == 2) ? 0 : (e + 1)]);
			const uint64_t key = (static_cast<uint64_t>(u) << 32) | static_cast<uint64_t>(v);
			size_t slot = hashDirectedEdge(key) & tableMask;
			while (slotKey[slot] != kEmptyKey && slotKey[slot] != key)
			{
				slot = (slot + 1u) & tableMask;
			}
			if (slotKey[slot] == kEmptyKey)
			{
				slotKey[slot] = key;
			}
			const int entryIdx = j * 3 + e;
			entryPoly[static_cast<size_t>(entryIdx)] = j;
			entryNext[static_cast<size_t>(entryIdx)] = slotHead[slot];
			slotHead[slot] = entryIdx;
		}
	}

	// Query reverse directed half-edges (v -> u) for each polygon i and apply exact legacy edge validator
	std::vector<int> candidates;
	candidates.reserve(16);

	for (int i = 0; i < numPolys; ++i)
	{
		int poly[3];
		fetchPoly(i, poly);
		const Vector3& vNorm = polyNormals[i];

		candidates.clear();
		for (int e = 0; e < 3; ++e)
		{
			const uint32_t u = static_cast<uint32_t>(poly[e]);
			const uint32_t v = static_cast<uint32_t>(poly[(e == 2) ? 0 : (e + 1)]);
			// Neighbor with opposite winding must own the reverse directed edge (v -> u)
			const uint64_t revKey = (static_cast<uint64_t>(v) << 32) | static_cast<uint64_t>(u);
			size_t slot = hashDirectedEdge(revKey) & tableMask;
			while (slotKey[slot] != kEmptyKey && slotKey[slot] != revKey)
			{
				slot = (slot + 1u) & tableMask;
			}
			if (slotKey[slot] == revKey)
			{
				for (int entry = slotHead[slot]; entry != -1; entry = entryNext[static_cast<size_t>(entry)])
				{
					const int j = entryPoly[static_cast<size_t>(entry)];
					if (j != i)
					{
						candidates.push_back(j);
					}
				}
			}
		}

		if (candidates.empty())
		{
			continue;
		}

		// Sort and deduplicate candidate indices so neighbor slot assignment matches legacy ascending j order exactly
		if (candidates.size() > 1u)
		{
			std::sort(candidates.begin(), candidates.end());
			candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
		}

		for (int j : candidates)
		{
			int otherPoly[3];
			fetchPoly(j, otherPoly);

			int index1 = -1;
			int index2 = -1;
			int index1Pos[2] = { 0, 0 };

			for (int a = 0; a < 3; ++a)
			{
				for (int b = 0; b < 3; ++b)
				{
					if (poly[a] == otherPoly[b])
					{
						if (index1 == -1)
						{
							index1 = poly[a];
							index1Pos[0] = a;
							index1Pos[1] = b;
						}
						else if (index2 == -1)
						{
							const int diff1 = a - index1Pos[0];
							const int diff2 = b - index1Pos[1];
							const uint32_t s1 = (static_cast<uint32_t>(diff1) & 0x80000000u) ^ ((static_cast<uint32_t>(std::abs(diff1)) & 2u) << 30);
							const uint32_t s2 = (static_cast<uint32_t>(diff2) & 0x80000000u) ^ ((static_cast<uint32_t>(std::abs(diff2)) & 2u) << 30);
							if (s1 != s2)
							{
								const Vector3& vOtherNorm = polyNormals[j];
								if (std::fabs(Vector3::Dot_Product(vOtherNorm, vNorm) + 1.0f) <= 0.01f)
								{
									continue;
								}
								index2 = poly[a];
							}
							else
							{
								continue;
							}
						}
						else
						{
							// Same polygon facing opposite direction (3 shared vertices)
							index1 = index2 = -1;
							continue;
						}
					}
				}
			}

			if (index1 != -1 && index2 != -1)
			{
				for (int a = 0; a < 3; ++a)
				{
					if (outNeighbors[i].neighbor[a].neighborIndex == kNoNeighbor)
					{
						outNeighbors[i].neighbor[a].neighborIndex = j;
						outNeighbors[i].neighbor[a].neighborEdgeIndex[0] = index1;
						outNeighbors[i].neighbor[a].neighborEdgeIndex[1] = index2;
						break;
					}
				}
			}
		}
	}
}

void ChainSilhouetteEdgesExact(
	int* silhouetteIndices,
	int indicesPerMesh)
{
	if (!silhouetteIndices || indicesPerMesh <= 4)
	{
		return;
	}

	const int edgeCount = indicesPerMesh / 2;
	int maxVert = 0;
	for (int i = 0; i < indicesPerMesh; ++i)
	{
		if (silhouetteIndices[i] > maxVert)
		{
			maxVert = silhouetteIndices[i];
		}
	}
	if (maxVert < 0)
	{
		return;
	}

	// Thread-local reusable buffers avoid per-frame heap allocations on worker threads and main thread
	thread_local std::vector<int> head;
	thread_local std::vector<int> nextEdge;
	thread_local std::vector<uint8_t> used;
	thread_local std::vector<int> ordered;

	const size_t vertTableSize = static_cast<size_t>(maxVert) + 1u;
	head.assign(vertTableSize, -1);
	nextEdge.assign(static_cast<size_t>(edgeCount), -1);
	used.assign(static_cast<size_t>(edgeCount), 0u);
	ordered.clear();
	ordered.reserve(static_cast<size_t>(indicesPerMesh));

	// Insert in reverse order so the linked list for each start vertex visits lowest edge index first
	// (matching deterministic greedy traversal order).
	for (int e = edgeCount - 1; e >= 0; --e)
	{
		const int u = silhouetteIndices[e * 2];
		if (u >= 0 && static_cast<size_t>(u) < vertTableSize)
		{
			nextEdge[e] = head[static_cast<size_t>(u)];
			head[static_cast<size_t>(u)] = e;
		}
	}

	int scanCursor = 0;
	while (static_cast<int>(ordered.size()) < indicesPerMesh)
	{
		while (scanCursor < edgeCount && used[static_cast<size_t>(scanCursor)])
		{
			++scanCursor;
		}
		if (scanCursor >= edgeCount)
		{
			break;
		}

		int curEdge = scanCursor;
		while (curEdge != -1)
		{
			used[static_cast<size_t>(curEdge)] = 1u;
			const int u = silhouetteIndices[curEdge * 2];
			const int v = silhouetteIndices[curEdge * 2 + 1];
			ordered.push_back(u);
			ordered.push_back(v);

			int nextCandidate = -1;
			if (v >= 0 && static_cast<size_t>(v) < vertTableSize)
			{
				int h = head[static_cast<size_t>(v)];
				while (h != -1 && used[static_cast<size_t>(h)])
				{
					h = nextEdge[static_cast<size_t>(h)];
				}
				head[static_cast<size_t>(v)] = h;
				if (h != -1)
				{
					nextCandidate = h;
					head[static_cast<size_t>(v)] = nextEdge[static_cast<size_t>(h)];
				}
			}
			curEdge = nextCandidate;
		}
	}

	if (static_cast<int>(ordered.size()) == indicesPerMesh)
	{
		std::memcpy(silhouetteIndices, ordered.data(), static_cast<size_t>(indicesPerMesh) * sizeof(int));
	}
}

void OptimizeVertexCacheForsyth(
	const uint16_t* inIndices,
	uint16_t* outIndices,
	unsigned int triCount,
	unsigned int vertexCount)
{
	if (!inIndices || !outIndices || triCount == 0u)
	{
		return;
	}
	if (triCount <= 2u || vertexCount == 0u)
	{
		if (inIndices != outIndices)
		{
			std::memcpy(outIndices, inIndices, static_cast<size_t>(triCount) * 3u * sizeof(uint16_t));
		}
		return;
	}

	// Ensure vertexCount covers the max referenced index
	unsigned int maxIdx = 0u;
	for (unsigned int i = 0u; i < triCount * 3u; ++i)
	{
		if (inIndices[i] > maxIdx)
		{
			maxIdx = inIndices[i];
		}
	}
	const unsigned int numVerts = (std::max)(vertexCount, maxIdx + 1u);

	// 1. Count triangle valence per vertex
	std::vector<unsigned int> vertTriCount(numVerts, 0u);
	for (unsigned int i = 0u; i < triCount * 3u; ++i)
	{
		++vertTriCount[inIndices[i]];
	}

	// 2. Build CSR adjacency offsets: vertex -> incident triangle list
	std::vector<unsigned int> vertOffsets(numVerts + 1u, 0u);
	for (unsigned int v = 0u; v < numVerts; ++v)
	{
		vertOffsets[v + 1u] = vertOffsets[v] + vertTriCount[v];
	}
	std::vector<unsigned int> vertActiveTris = vertTriCount;
	std::vector<unsigned int> vertCursor = vertOffsets;
	std::vector<unsigned int> adjTris(vertOffsets[numVerts], 0u);

	for (unsigned int t = 0u; t < triCount; ++t)
	{
		const uint16_t v0 = inIndices[t * 3u + 0u];
		const uint16_t v1 = inIndices[t * 3u + 1u];
		const uint16_t v2 = inIndices[t * 3u + 2u];
		adjTris[vertCursor[v0]++] = t;
		adjTris[vertCursor[v1]++] = t;
		adjTris[vertCursor[v2]++] = t;
	}

	// 3. Initial vertex & triangle scores
	std::vector<int> vertCachePos(numVerts, -1);
	std::vector<float> vertScore(numVerts, 0.0f);
	for (unsigned int v = 0u; v < numVerts; ++v)
	{
		vertScore[v] = ComputeVertexScore(-1, vertActiveTris[v]);
	}

	std::vector<float> triScore(triCount, 0.0f);
	std::vector<uint8_t> triAdded(triCount, 0u);
	unsigned int bestTri = 0u;
	float bestTriScore = -1.0f;

	for (unsigned int t = 0u; t < triCount; ++t)
	{
		const float s = vertScore[inIndices[t * 3u]] + vertScore[inIndices[t * 3u + 1u]] + vertScore[inIndices[t * 3u + 2u]];
		triScore[t] = s;
		if (s > bestTriScore)
		{
			bestTriScore = s;
			bestTri = t;
		}
	}

	// Work into a temporary buffer if inIndices == outIndices
	std::vector<uint16_t> tempOut;
	uint16_t* dst = outIndices;
	if (inIndices == outIndices)
	{
		tempOut.resize(static_cast<size_t>(triCount) * 3u);
		dst = tempOut.data();
	}

	// LRU cache (+3 temporary slots for newly inserted triangle vertices)
	int cache[kForsythCacheSize + 3u];
	for (unsigned int c = 0u; c < kForsythCacheSize + 3u; ++c)
	{
		cache[c] = -1;
	}
	unsigned int cacheUsed = 0u;
	unsigned int scanCursor = 0u;

	for (unsigned int outIdx = 0u; outIdx < triCount; ++outIdx)
	{
		// Fallback full scan only when cache has no active adjacent triangles
		if (bestTriScore < 0.0f)
		{
			while (scanCursor < triCount && triAdded[scanCursor])
			{
				++scanCursor;
			}
			bestTri = scanCursor;
		}

		const uint16_t tv[3] = {
			inIndices[bestTri * 3u + 0u],
			inIndices[bestTri * 3u + 1u],
			inIndices[bestTri * 3u + 2u]
		};
		dst[outIdx * 3u + 0u] = tv[0];
		dst[outIdx * 3u + 1u] = tv[1];
		dst[outIdx * 3u + 2u] = tv[2];
		triAdded[bestTri] = 1u;

		// Decrement active valence and move triangle vertices to front of LRU cache
		for (int k = 0; k < 3; ++k)
		{
			const unsigned int v = tv[k];
			if (vertActiveTris[v] > 0u)
			{
				--vertActiveTris[v];
			}

			int existingPos = vertCachePos[v];
			unsigned int shiftEnd = 0u;
			if (existingPos >= 0)
			{
				shiftEnd = static_cast<unsigned int>(existingPos);
			}
			else
			{
				shiftEnd = cacheUsed;
				if (cacheUsed < kForsythCacheSize + 3u)
				{
					++cacheUsed;
				}
			}

			for (unsigned int c = shiftEnd; c > 0u; --c)
			{
				cache[c] = cache[c - 1u];
				if (cache[c] >= 0)
				{
					vertCachePos[static_cast<unsigned int>(cache[c])] = static_cast<int>(c);
				}
			}
			cache[0] = static_cast<int>(v);
			vertCachePos[v] = 0;
		}

		// Update scores for all vertices in cache + evicted tail, and pick next best triangle
		bestTriScore = -1.0f;
		const unsigned int updateLimit = cacheUsed;
		for (unsigned int c = 0u; c < updateLimit; ++c)
		{
			const int vi = cache[c];
			if (vi < 0)
			{
				continue;
			}
			const unsigned int v = static_cast<unsigned int>(vi);
			int newCachePos = (c < kForsythCacheSize) ? static_cast<int>(c) : -1;
			if (newCachePos < 0)
			{
				vertCachePos[v] = -1;
				cache[c] = -1;
			}

			const float oldScore = vertScore[v];
			const float newScore = ComputeVertexScore(newCachePos, vertActiveTris[v]);
			const float delta = newScore - oldScore;
			vertScore[v] = newScore;

			const unsigned int startAdj = vertOffsets[v];
			const unsigned int endAdj = vertOffsets[v + 1u];
			for (unsigned int a = startAdj; a < endAdj; ++a)
			{
				const unsigned int t = adjTris[a];
				if (triAdded[t])
				{
					continue;
				}
				triScore[t] += delta;
				if (triScore[t] > bestTriScore)
				{
					bestTriScore = triScore[t];
					bestTri = t;
				}
			}
		}

		if (cacheUsed > kForsythCacheSize)
		{
			cacheUsed = kForsythCacheSize;
		}
	}

	if (inIndices == outIndices)
	{
		std::memcpy(outIndices, tempOut.data(), static_cast<size_t>(triCount) * 3u * sizeof(uint16_t));
	}
}

double ComputeACMR(
	const uint16_t* indices,
	unsigned int triCount,
	unsigned int cacheSize)
{
	if (!indices || triCount == 0u || cacheSize == 0u)
	{
		return 0.0;
	}

	std::vector<int> fifo(cacheSize, -1);
	unsigned int head = 0u;
	unsigned int misses = 0u;

	for (unsigned int i = 0u; i < triCount * 3u; ++i)
	{
		const int v = static_cast<int>(indices[i]);
		bool hit = false;
		for (unsigned int c = 0u; c < cacheSize; ++c)
		{
			if (fifo[c] == v)
			{
				hit = true;
				break;
			}
		}
		if (!hit)
		{
			++misses;
			fifo[head] = v;
			head = (head + 1u) % cacheSize;
		}
	}
	return static_cast<double>(misses) / static_cast<double>(triCount);
}

void ParallelFor(
	size_t count,
	const std::function<void(size_t begin, size_t end)>& fn,
	size_t minItemsPerTask)
{
	ShadowWorkerPool::Instance().ParallelFor(count, fn, minItemsPerTask);
}

unsigned int GetWorkerThreadCount()
{
	return ShadowWorkerPool::Instance().ThreadCount();
}
} // namespace W3DLosslessOpt
