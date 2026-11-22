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

#include "WW3D2/w3d_lossless_opt.h"
#include "WWMath/vector3.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
struct ClosedSphereMesh
{
	std::vector<Vector3> verts;
	std::vector<uint16_t> indices; // 3 * triCount
	std::vector<Vector3> normals;
};

// Generates a watertight closed 2-manifold UV-sphere with duplicated seam vertices
// (realistic W3D mesh with split UV/normal vertices that must weld into a closed 2-manifold).
ClosedSphereMesh BuildClosedSphere(int stacks, int slices)
{
	ClosedSphereMesh mesh;
	constexpr float kPi = 3.14159265358979323846f;

	// Ring vertices including duplicated seam column (slices + 1) to test exact welding
	const int cols = slices + 1;
	mesh.verts.reserve(static_cast<size_t>((stacks - 1) * cols + 2));

	// North pole (index 0) and south pole (index 1)
	mesh.verts.push_back(Vector3(0.0f, 0.0f, 1.0f));
	mesh.verts.push_back(Vector3(0.0f, 0.0f, -1.0f));

	for (int st = 1; st < stacks; ++st)
	{
		const float phi = kPi * static_cast<float>(st) / static_cast<float>(stacks);
		const float sinPhi = std::sin(phi);
		const float cosPhi = std::cos(phi);
		for (int sl = 0; sl <= slices; ++sl)
		{
			// Wrap seam vertex back to sl == 0 exact float bits so welding closes the seam losslessly
			const int canonicalSl = (sl == slices) ? 0 : sl;
			const float theta = 2.0f * kPi * static_cast<float>(canonicalSl) / static_cast<float>(slices);
			mesh.verts.push_back(Vector3(sinPhi * std::cos(theta), sinPhi * std::sin(theta), cosPhi));
		}
	}

	auto ringVertex = [cols](int st, int sl) -> uint16_t
	{
		return static_cast<uint16_t>(2 + (st - 1) * cols + sl);
	};

	auto pushTri = [&mesh](uint16_t a, uint16_t b, uint16_t c)
	{
		mesh.indices.push_back(a);
		mesh.indices.push_back(b);
		mesh.indices.push_back(c);
	};

	// Top cap
	for (int sl = 0; sl < slices; ++sl)
	{
		pushTri(0u, ringVertex(1, sl), ringVertex(1, sl + 1));
	}

	// Middle quads
	for (int st = 1; st < stacks - 1; ++st)
	{
		for (int sl = 0; sl < slices; ++sl)
		{
			const uint16_t v00 = ringVertex(st, sl);
			const uint16_t v01 = ringVertex(st, sl + 1);
			const uint16_t v10 = ringVertex(st + 1, sl);
			const uint16_t v11 = ringVertex(st + 1, sl + 1);
			pushTri(v00, v10, v11);
			pushTri(v00, v11, v01);
		}
	}

	// Bottom cap
	for (int sl = 0; sl < slices; ++sl)
	{
		pushTri(1u, ringVertex(stacks - 1, sl + 1), ringVertex(stacks - 1, sl));
	}

	// Compute outward triangle normals matching W3DShadowGeometryMesh::buildPolygonNormal
	const size_t triCount = mesh.indices.size() / 3u;
	mesh.normals.resize(triCount);
	for (size_t t = 0; t < triCount; ++t)
	{
		const Vector3& v0 = mesh.verts[mesh.indices[t * 3u + 0u]];
		const Vector3& v1 = mesh.verts[mesh.indices[t * 3u + 1u]];
		const Vector3& v2 = mesh.verts[mesh.indices[t * 3u + 2u]];
		Vector3 edge1 = v1 - v0;
		Vector3 edge2 = v1 - v2;
		Vector3::Normalized_Cross_Product(edge2, edge1, &mesh.normals[t]);
	}

	return mesh;
}

// Legacy O(V^2) reference implementation from W3DVolumetricShadow.cpp
int LegacyWeldVertices(const Vector3* verts, int numVerts, uint16_t* vertParent)
{
	std::fill(vertParent, vertParent + numVerts, static_cast<uint16_t>(0xffffu));
	int newVertexCount = numVerts;
	for (int j = 0; j < numVerts; ++j)
	{
		if (vertParent[j] != 0xffffu)
			continue;
		const Vector3* v_curr = &verts[j];
		for (int k = j + 1; k < numVerts; ++k)
		{
			Vector3 len(*v_curr - verts[k]);
			if (len.Length2() == 0.0f)
			{
				vertParent[k] = static_cast<uint16_t>(j);
				--newVertexCount;
			}
		}
		vertParent[j] = static_cast<uint16_t>(j);
	}
	return newVertexCount;
}

// Legacy O(P^2) reference implementation from W3DVolumetricShadow.cpp
void LegacyBuildPolygonNeighbors(
	int numPolys,
	const uint16_t* rawTriIndices,
	const uint16_t* parentVerts,
	const Vector3* polyNormals,
	W3DLosslessOpt::ShadowPolyNeighbor* outNeighbors)
{
	for (int i = 0; i < numPolys; ++i)
	{
		outNeighbors[i].myIndex = i;
		outNeighbors[i].status = 0;
		for (int j = 0; j < 3; ++j)
		{
			outNeighbors[i].neighbor[j].neighborIndex = -1;
			outNeighbors[i].neighbor[j].neighborEdgeIndex[0] = 0;
			outNeighbors[i].neighbor[j].neighborEdgeIndex[1] = 0;
		}
	}

	for (int i = 0; i < numPolys; ++i)
	{
		const int poly[3] = {
			parentVerts[rawTriIndices[i * 3 + 0]],
			parentVerts[rawTriIndices[i * 3 + 1]],
			parentVerts[rawTriIndices[i * 3 + 2]]
		};
		const Vector3& vNorm = polyNormals[i];

		for (int j = 0; j < numPolys; ++j)
		{
			if (i == j)
				continue;

			const int otherPoly[3] = {
				parentVerts[rawTriIndices[j * 3 + 0]],
				parentVerts[rawTriIndices[j * 3 + 1]],
				parentVerts[rawTriIndices[j * 3 + 2]]
			};

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
									continue;
								index2 = poly[a];
							}
							else
							{
								continue;
							}
						}
						else
						{
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
					if (outNeighbors[i].neighbor[a].neighborIndex == -1)
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
} // namespace

TEST(LosslessW3DHighPolyTest, WeldVerticesExact_BitIdenticalAndBenchmark)
{
	// 10,000 vertices with ~35% duplicates plus explicit -0.0f vs +0.0f edge cases
	constexpr int kNumVerts = 10000;
	std::vector<Vector3> verts;
	verts.reserve(kNumVerts);

	verts.push_back(Vector3(0.0f, -0.0f, 0.0f));
	verts.push_back(Vector3(-0.0f, 0.0f, -0.0f)); // Must weld to index 0!

	std::mt19937 rng(20260401u);
	std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
	for (int i = 2; i < kNumVerts; ++i)
	{
		if ((i % 3) == 0)
		{
			verts.push_back(verts[static_cast<size_t>(rng() % static_cast<unsigned>(i))]);
		}
		else
		{
			verts.push_back(Vector3(dist(rng), dist(rng), dist(rng)));
		}
	}

	std::vector<uint16_t> legacyParent(kNumVerts);
	std::vector<uint16_t> fastParent(kNumVerts);

	const auto t0 = std::chrono::high_resolution_clock::now();
	const int legacyUnique = LegacyWeldVertices(verts.data(), kNumVerts, legacyParent.data());
	const auto t1 = std::chrono::high_resolution_clock::now();
	const int fastUnique = W3DLosslessOpt::WeldVerticesExact(verts.data(), kNumVerts, fastParent.data());
	const auto t2 = std::chrono::high_resolution_clock::now();

	EXPECT_EQ(fastUnique, legacyUnique);
	EXPECT_EQ(fastParent[1], 0u); // -0.0f canonicalized to +0.0f
	EXPECT_EQ(fastParent, legacyParent);

	const double legacyMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
	const double fastMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
	std::printf("[BENCH] WeldVerticesExact (10,000 verts): legacy O(V^2)=%.2f ms, fast O(V)=%.3f ms (%.1fx faster)\n",
		legacyMs, fastMs, legacyMs / (std::max)(fastMs, 0.001));

	// Also verify > 32,767 vertices (previously overflowed signed 16-bit / MAX_SHADOW_VOLUME_VERTS=16384)
	constexpr int kHugeVerts = 40000;
	std::vector<Vector3> hugeVerts(kHugeVerts);
	for (int i = 0; i < kHugeVerts; ++i)
	{
		hugeVerts[i] = Vector3(static_cast<float>(i % 35000), static_cast<float>(i / 35000), 1.0f);
	}
	hugeVerts[39999] = hugeVerts[34999];
	std::vector<uint16_t> hugeParent(kHugeVerts);
	const int hugeUnique = W3DLosslessOpt::WeldVerticesExact(hugeVerts.data(), kHugeVerts, hugeParent.data());
	EXPECT_EQ(hugeUnique, kHugeVerts - 1);
	EXPECT_EQ(hugeParent[39999], 34999u);
}

TEST(LosslessW3DHighPolyTest, BuildPolygonNeighborsExact_BitIdenticalClosedManifoldAndBenchmark)
{
	// 1. Exact bit-for-bit parity check on a 2,000-triangle watertight closed 2-manifold sphere
	const ClosedSphereMesh smallSphere = BuildClosedSphere(26, 40); // (25*2 - 2)*40? = 2000 tris
	const int smallPolys = static_cast<int>(smallSphere.indices.size() / 3u);
	ASSERT_GE(smallPolys, 1900);

	std::vector<uint16_t> smallParents(smallSphere.verts.size());
	W3DLosslessOpt::WeldVerticesExact(smallSphere.verts.data(), static_cast<int>(smallSphere.verts.size()), smallParents.data());

	std::vector<W3DLosslessOpt::ShadowPolyNeighbor> legacyNbrs(static_cast<size_t>(smallPolys));
	std::vector<W3DLosslessOpt::ShadowPolyNeighbor> fastNbrs(static_cast<size_t>(smallPolys));

	const auto p0 = std::chrono::high_resolution_clock::now();
	LegacyBuildPolygonNeighbors(smallPolys, smallSphere.indices.data(), smallParents.data(), smallSphere.normals.data(), legacyNbrs.data());
	const auto p1 = std::chrono::high_resolution_clock::now();
	W3DLosslessOpt::BuildPolygonNeighborsExact(smallPolys, smallSphere.indices.data(), smallParents.data(), smallSphere.normals.data(), fastNbrs.data());
	const auto p2 = std::chrono::high_resolution_clock::now();

	// Byte-for-byte identical neighbor graph (including exact slot order & edge indices)
	for (int i = 0; i < smallPolys; ++i)
	{
		EXPECT_EQ(fastNbrs[i].myIndex, legacyNbrs[i].myIndex);
		for (int a = 0; a < 3; ++a)
		{
			EXPECT_EQ(fastNbrs[i].neighbor[a].neighborIndex, legacyNbrs[i].neighbor[a].neighborIndex);
			EXPECT_EQ(fastNbrs[i].neighbor[a].neighborEdgeIndex[0], legacyNbrs[i].neighbor[a].neighborEdgeIndex[0]);
			EXPECT_EQ(fastNbrs[i].neighbor[a].neighborEdgeIndex[1], legacyNbrs[i].neighbor[a].neighborEdgeIndex[1]);
			// Watertight closed 2-manifold guarantee: zero missing neighbors => zero aircraft shadow streaks!
			EXPECT_NE(fastNbrs[i].neighbor[a].neighborIndex, -1);
		}
	}

	const double legacySmallMs = std::chrono::duration<double, std::milli>(p1 - p0).count();
	const double fastSmallMs = std::chrono::duration<double, std::milli>(p2 - p1).count();

	// 2. Benchmark on a 10,000-triangle watertight high-poly sphere
	const ClosedSphereMesh highPolySphere = BuildClosedSphere(51, 100); // 10,000 triangles exact!
	const int highPolys = static_cast<int>(highPolySphere.indices.size() / 3u);
	ASSERT_EQ(highPolys, 10000);

	std::vector<uint16_t> highParents(highPolySphere.verts.size());
	W3DLosslessOpt::WeldVerticesExact(highPolySphere.verts.data(), static_cast<int>(highPolySphere.verts.size()), highParents.data());

	std::vector<W3DLosslessOpt::ShadowPolyNeighbor> highNbrs(static_cast<size_t>(highPolys));
	const auto h0 = std::chrono::high_resolution_clock::now();
	W3DLosslessOpt::BuildPolygonNeighborsExact(highPolys, highPolySphere.indices.data(), highParents.data(), highPolySphere.normals.data(), highNbrs.data());
	const auto h1 = std::chrono::high_resolution_clock::now();
	const double fast10kMs = std::chrono::duration<double, std::milli>(h1 - h0).count();

	// Verify ALL 10,000 triangles remain 100% watertight 3-neighbor 2-manifold!
	int missingEdges = 0;
	for (int i = 0; i < highPolys; ++i)
	{
		for (int a = 0; a < 3; ++a)
		{
			if (highNbrs[i].neighbor[a].neighborIndex == -1)
			{
				++missingEdges;
			}
		}
	}
	EXPECT_EQ(missingEdges, 0);

	std::printf("[BENCH] BuildPolygonNeighborsExact: 2k tris legacy=%.2f ms vs fast=%.3f ms (%.1fx); 10k tris fast=%.3f ms\n",
		legacySmallMs, fastSmallMs, legacySmallMs / (std::max)(fastSmallMs, 0.001), fast10kMs);
}

TEST(LosslessW3DHighPolyTest, ChainSilhouetteEdgesExact_PreservesClosedRingsAndBenchmark)
{
	// Construct 3 closed rings + 1 open chain = 3,000 directed edges, including vertex indices > 32,767
	std::vector<std::pair<int, int>> inputEdges;
	inputEdges.reserve(3000);

	auto addRing = [&inputEdges](int baseVert, int len)
	{
		for (int i = 0; i < len; ++i)
		{
			inputEdges.emplace_back(baseVert + i, baseVert + ((i + 1) % len));
		}
	};
	addRing(100, 1000);
	addRing(20000, 1000);
	addRing(40000, 800); // > 32,767 verifies 32-bit int silhouette indices!
	for (int i = 0; i < 200; ++i)
	{
		inputEdges.emplace_back(50000 + i, 50000 + i + 1);
	}
	ASSERT_EQ(inputEdges.size(), 3000u);

	std::mt19937 rng(424242u);
	std::shuffle(inputEdges.begin(), inputEdges.end(), rng);

	std::vector<int> legacyIndices(inputEdges.size() * 2u);
	std::vector<int> fastIndices(inputEdges.size() * 2u);
	for (size_t e = 0; e < inputEdges.size(); ++e)
	{
		legacyIndices[e * 2u] = fastIndices[e * 2u] = inputEdges[e].first;
		legacyIndices[e * 2u + 1u] = fastIndices[e * 2u + 1u] = inputEdges[e].second;
	}

	const int indicesPerMesh = static_cast<int>(legacyIndices.size());

	// Legacy O(E^2) nested scan from constructVolume / constructVolumeVB
	const auto l0 = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < indicesPerMesh; i += 2)
	{
		const int currentEdgeEnd = legacyIndices[i + 1];
		for (int k = i + 2; k < indicesPerMesh; k += 2)
		{
			if (legacyIndices[k] == currentEdgeEnd)
			{
				std::swap(legacyIndices[i + 2], legacyIndices[k]);
				std::swap(legacyIndices[i + 3], legacyIndices[k + 1]);
				break;
			}
		}
	}
	const auto l1 = std::chrono::high_resolution_clock::now();

	// New O(E) linear-time edge chaining
	const auto f0 = std::chrono::high_resolution_clock::now();
	W3DLosslessOpt::ChainSilhouetteEdgesExact(fastIndices.data(), indicesPerMesh);
	const auto f1 = std::chrono::high_resolution_clock::now();

	// Verify 100% exact directed edge multiset parity
	std::vector<std::pair<int, int>> expectedSorted = inputEdges;
	std::vector<std::pair<int, int>> actualSorted;
	actualSorted.reserve(inputEdges.size());
	for (int i = 0; i < indicesPerMesh; i += 2)
	{
		actualSorted.emplace_back(fastIndices[i], fastIndices[i + 1]);
	}
	std::sort(expectedSorted.begin(), expectedSorted.end());
	std::sort(actualSorted.begin(), actualSorted.end());
	EXPECT_EQ(actualSorted, expectedSorted);

	// Count closed loops & strip breaks after O(E) chaining:
	// 3 closed rings must each close in 1 unbroken strip (closedLoops == 3).
	int closedLoops = 0;
	int stripStart = fastIndices[0];
	for (int i = 0; i < indicesPerMesh; i += 2)
	{
		const int edgeEnd = fastIndices[i + 1];
		const bool connected = ((i + 2) < indicesPerMesh && fastIndices[i + 2] == edgeEnd);
		if (!connected)
		{
			if (edgeEnd == stripStart)
			{
				++closedLoops;
			}
			if ((i + 2) < indicesPerMesh)
			{
				stripStart = fastIndices[i + 2];
			}
		}
	}
	EXPECT_EQ(closedLoops, 3);

	const double legacyMs = std::chrono::duration<double, std::milli>(l1 - l0).count();
	const double fastMs = std::chrono::duration<double, std::milli>(f1 - f0).count();
	std::printf("[BENCH] ChainSilhouetteEdgesExact (3,000 edges): legacy O(E^2)=%.3f ms, fast O(E)=%.3f ms (%.1fx faster)\n",
		legacyMs, fastMs, legacyMs / (std::max)(fastMs, 0.001));
}

TEST(LosslessW3DHighPolyTest, OptimizeVertexCacheForsyth_100PercentLosslessAndACMRBenchmark)
{
	// 10,000-triangle grid mesh (71 x 71 quads ~= 10,082 triangles) with worst-case shuffled draw order
	constexpr unsigned int kGridW = 72u;
	constexpr unsigned int kGridH = 72u;
	constexpr unsigned int kVertCount = kGridW * kGridH;

	std::vector<std::tuple<uint16_t, uint16_t, uint16_t>> tris;
	tris.reserve((kGridW - 1u) * (kGridH - 1u) * 2u);
	for (unsigned int y = 0; y < kGridH - 1u; ++y)
	{
		for (unsigned int x = 0; x < kGridW - 1u; ++x)
		{
			const uint16_t v00 = static_cast<uint16_t>(y * kGridW + x);
			const uint16_t v10 = static_cast<uint16_t>(y * kGridW + x + 1u);
			const uint16_t v01 = static_cast<uint16_t>((y + 1u) * kGridW + x);
			const uint16_t v11 = static_cast<uint16_t>((y + 1u) * kGridW + x + 1u);
			tris.emplace_back(v00, v10, v11);
			tris.emplace_back(v00, v11, v01);
		}
	}
	const unsigned int triCount = static_cast<unsigned int>(tris.size());
	ASSERT_GE(triCount, 10000u);

	std::mt19937 rng(1337u);
	std::shuffle(tris.begin(), tris.end(), rng);

	std::vector<uint16_t> inIndices(triCount * 3u);
	for (unsigned int t = 0; t < triCount; ++t)
	{
		inIndices[t * 3u + 0u] = std::get<0>(tris[t]);
		inIndices[t * 3u + 1u] = std::get<1>(tris[t]);
		inIndices[t * 3u + 2u] = std::get<2>(tris[t]);
	}

	std::vector<uint16_t> outIndices(triCount * 3u, 0u);
	const double acmrBefore = W3DLosslessOpt::ComputeACMR(inIndices.data(), triCount, 32u);

	const auto t0 = std::chrono::high_resolution_clock::now();
	W3DLosslessOpt::OptimizeVertexCacheForsyth(inIndices.data(), outIndices.data(), triCount, kVertCount);
	const auto t1 = std::chrono::high_resolution_clock::now();

	const double acmrAfter = W3DLosslessOpt::ComputeACMR(outIndices.data(), triCount, 32u);
	const double optMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

	// 100% lossless guarantee: exact same set of triangles with exact same winding order (zero geometry alteration!)
	std::vector<std::tuple<uint16_t, uint16_t, uint16_t>> expectedTris = tris;
	std::vector<std::tuple<uint16_t, uint16_t, uint16_t>> actualTris;
	actualTris.reserve(triCount);
	for (unsigned int t = 0; t < triCount; ++t)
	{
		actualTris.emplace_back(outIndices[t * 3u + 0u], outIndices[t * 3u + 1u], outIndices[t * 3u + 2u]);
	}
	std::sort(expectedTris.begin(), expectedTris.end());
	std::sort(actualTris.begin(), actualTris.end());
	EXPECT_EQ(actualTris, expectedTris);

	// Expect dramatic GPU post-transform vertex cache improvement (typical ~2.85 -> ~0.65)
	EXPECT_LT(acmrAfter, 0.90);
	EXPECT_LT(acmrAfter, acmrBefore * 0.40);

	std::printf("[BENCH] OptimizeVertexCacheForsyth (%u tris): ACMR %.3f -> %.3f (%.2fx fewer vertex transforms), time=%.2f ms\n",
		triCount, acmrBefore, acmrAfter, acmrBefore / (std::max)(acmrAfter, 0.001), optMs);
}

TEST(LosslessW3DHighPolyTest, ShadowThreadPool_DeterministicParallelExecutionAndSpeedup)
{
	// Shared immutable 5,000-triangle watertight closed mesh (simulating 64 moving units of the same high-poly model)
	const ClosedSphereMesh sharedMesh = BuildClosedSphere(36, 72); // ~5,040 triangles
	const int numPolys = static_cast<int>(sharedMesh.indices.size() / 3u);
	ASSERT_GE(numPolys, 5000);

	std::vector<uint16_t> parentVerts(sharedMesh.verts.size());
	W3DLosslessOpt::WeldVerticesExact(sharedMesh.verts.data(), static_cast<int>(sharedMesh.verts.size()), parentVerts.data());

	std::vector<W3DLosslessOpt::ShadowPolyNeighbor> neighbors(static_cast<size_t>(numPolys));
	W3DLosslessOpt::BuildPolygonNeighborsExact(numPolys, sharedMesh.indices.data(), parentVerts.data(), sharedMesh.normals.data(), neighbors.data());

	constexpr size_t kNumCasters = 64u;
	std::vector<Vector3> lightPositions(kNumCasters);
	for (size_t c = 0; c < kNumCasters; ++c)
	{
		const float angle = static_cast<float>(c) * 0.19f;
		lightPositions[c] = Vector3(std::cos(angle) * 50.0f, std::sin(angle) * 50.0f, 35.0f + static_cast<float>(c % 7) * 5.0f);
	}

	// Exact CPU silhouette extraction + O(E) edge chaining kernel (read-only shared neighbors + thread_local status)
	auto buildCasterSilhouette = [&](size_t casterIdx, std::vector<int>& outEdges)
	{
		outEdges.clear();
		thread_local std::vector<uint8_t> polyStatus;
		polyStatus.assign(static_cast<size_t>(numPolys), 0u);

		const Vector3& lightPos = lightPositions[casterIdx];
		for (int i = 0; i < numPolys; ++i)
		{
			const int v0 = parentVerts[sharedMesh.indices[i * 3]];
			const Vector3 lightVec = sharedMesh.verts[v0] - lightPos;
			if (Vector3::Dot_Product(lightVec, sharedMesh.normals[i]) < 0.0f)
			{
				polyStatus[i] = 1u;
			}
		}

		for (int i = 0; i < numPolys; ++i)
		{
			const bool usVis = (polyStatus[i] & 1u) != 0u;
			const auto& pn = neighbors[i];
			for (int j = 0; j < 3; ++j)
			{
				const int other = pn.neighbor[j].neighborIndex;
				if (other == -1 || (polyStatus[other] & 2u) != 0u)
					continue;
				const bool otherVis = (polyStatus[other] & 1u) != 0u;
				if (usVis != otherVis)
				{
					const int visPoly = usVis ? i : other;
					const int hidPoly = usVis ? other : i;
					const auto& visPn = neighbors[visPoly];
					int nbrSlot = 0;
					for (int s = 0; s < 3; ++s)
					{
						if (visPn.neighbor[s].neighborIndex == hidPoly)
						{
							nbrSlot = s;
							break;
						}
					}
					const int vl[3] = {
						parentVerts[sharedMesh.indices[visPoly * 3 + 0]],
						parentVerts[sharedMesh.indices[visPoly * 3 + 1]],
						parentVerts[sharedMesh.indices[visPoly * 3 + 2]]
					};
					const int e0 = visPn.neighbor[nbrSlot].neighborEdgeIndex[0];
					const int e1 = visPn.neighbor[nbrSlot].neighborEdgeIndex[1];
					if (vl[0] != e0 && vl[0] != e1)
					{
						outEdges.push_back(vl[1]);
						outEdges.push_back(vl[2]);
					}
					else if (vl[1] != e0 && vl[1] != e1)
					{
						outEdges.push_back(vl[2]);
						outEdges.push_back(vl[0]);
					}
					else
					{
						outEdges.push_back(vl[0]);
						outEdges.push_back(vl[1]);
					}
				}
			}
			polyStatus[i] |= 2u;
		}

		W3DLosslessOpt::ChainSilhouetteEdgesExact(outEdges.data(), static_cast<int>(outEdges.size()));
	};

	std::vector<std::vector<int>> serialOutputs(kNumCasters);
	std::vector<std::vector<int>> parallelOutputs(kNumCasters);

	const auto s0 = std::chrono::high_resolution_clock::now();
	for (size_t c = 0; c < kNumCasters; ++c)
	{
		buildCasterSilhouette(c, serialOutputs[c]);
	}
	const auto s1 = std::chrono::high_resolution_clock::now();

	// Warm up thread pool once, then measure steady-state frame execution
	W3DLosslessOpt::ParallelFor(kNumCasters, [&](size_t begin, size_t end)
	{
		for (size_t c = begin; c < end; ++c)
		{
			buildCasterSilhouette(c, parallelOutputs[c]);
		}
	});

	const auto p0 = std::chrono::high_resolution_clock::now();
	W3DLosslessOpt::ParallelFor(kNumCasters, [&](size_t begin, size_t end)
	{
		for (size_t c = begin; c < end; ++c)
		{
			buildCasterSilhouette(c, parallelOutputs[c]);
		}
	});
	const auto p1 = std::chrono::high_resolution_clock::now();

	// Bit-exact deterministic match across all 64 dynamic casters!
	for (size_t c = 0; c < kNumCasters; ++c)
	{
		EXPECT_GT(serialOutputs[c].size(), 0u);
		EXPECT_EQ(parallelOutputs[c], serialOutputs[c]);
	}

	const double serialMs = std::chrono::duration<double, std::milli>(s1 - s0).count();
	const double parallelMs = std::chrono::duration<double, std::milli>(p1 - p0).count();
	std::printf("[BENCH] Parallel Dynamic Shadows (%zu x %d-tri casters, %u threads): serial=%.2f ms, parallel=%.2f ms (%.2fx speedup)\n",
		kNumCasters, numPolys, W3DLosslessOpt::GetWorkerThreadCount(), serialMs, parallelMs, serialMs / (std::max)(parallelMs, 0.001));
}
