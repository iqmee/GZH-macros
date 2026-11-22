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

#include <cstddef>
#include <cstdint>
#include <functional>

class Vector3;

/**
 * 100% lossless W3D mesh & volumetric shadow performance utilities.
 *
 * Guarantees zero visual distortion and zero geometry/topology alteration:
 * - No Auto-LOD / no vertex merging or decimation of visible meshes.
 * - No Shadow Proxy decimation (preserves exact closed 2-manifold shadow geometry).
 * - All algorithms produce bit-exact or exact-permutation outputs.
 */
namespace W3DLosslessOpt
{
	/**
	 * Exact O(V) bit-identical replacement for legacy O(V^2) duplicate-vertex welding
	 * in W3DShadowGeometry::initFromHLOD / initFromMesh.
	 *
	 * Matches `(verts[j] - verts[k]).Length2() == 0.0f` bit-for-bit (including -0.0f == +0.0f)
	 * and always selects the lowest index `j` as the parent vertex.
	 *
	 * @return Number of unique (welded) vertices.
	 */
	int WeldVerticesExact(
		const Vector3* verts,
		int numVerts,
		uint16_t* outParentVerts);

	struct ShadowPolyNeighborEdge
	{
		int neighborIndex;        // -1 (NO_NEIGHBOR) if no adjacent polygon on this edge
		int neighborEdgeIndex[2]; // Two welded vertex indices representing the shared edge
	};

	struct ShadowPolyNeighbor
	{
		int myIndex;
		uint8_t status;
		ShadowPolyNeighborEdge neighbor[3];
	};

	/**
	 * Exact O(P) directed half-edge hash replacement for legacy O(P^2)
	 * W3DShadowGeometryMesh::buildPolygonNeighbors().
	 *
	 * Preserves the exact closed 2-manifold neighbor graph (identical neighbor indices,
	 * shared edges, opposite-winding checks, opposite-normal checks, and slot ordering)
	 * with zero approximation and zero shadow-proxy artifacts.
	 */
	void BuildPolygonNeighborsExact(
		int numPolys,
		const uint16_t* rawTriIndices,
		const uint16_t* parentVerts,
		const Vector3* polyNormals,
		ShadowPolyNeighbor* outNeighbors);

	/**
	 * Exact O(E) linear-time replacement for legacy O(E^2) silhouette edge chaining
	 * in W3DVolumetricShadow::constructVolume / constructVolumeVB.
	 *
	 * Reorders the directed edge pairs `(silhouetteIndices[2*e], silhouetteIndices[2*e+1])`
	 * into contiguous connected chains so consecutive edges share endpoints (`edge[i].end == edge[i+1].start`).
	 * Preserves the exact multiset and direction of every input edge (zero dropped or altered edges).
	 */
	void ChainSilhouetteEdgesExact(
		int* silhouetteIndices,
		int indicesPerMesh);

	/**
	 * 100% lossless Tom Forsyth Linear-Speed GPU Post-Transform Vertex Cache Optimization.
	 *
	 * Reorders triangle draw order inside a triangle list for a 32-entry GPU vertex FIFO cache.
	 * Does NOT drop, merge, or move any vertex or triangle, and preserves exact triangle winding.
	 */
	void OptimizeVertexCacheForsyth(
		const uint16_t* inIndices,
		uint16_t* outIndices,
		unsigned int triCount,
		unsigned int vertexCount);

	/**
	 * Computes Average Cache Miss Ratio (ACMR) for a triangle list under a FIFO vertex cache.
	 * Used by automated tests and benchmarks.
	 */
	double ComputeACMR(
		const uint16_t* indices,
		unsigned int triCount,
		unsigned int cacheSize = 32u);

	/**
	 * Persistent worker pool for parallel CPU-only volumetric shadow silhouette/extrusion work.
	 * Worker threads NEVER touch Direct3D8 or W3DBufferManager; they write exclusively into
	 * disjoint per-shadow RAM geometry buffers.
	 */
	void ParallelFor(
		size_t count,
		const std::function<void(size_t begin, size_t end)>& fn,
		size_t minItemsPerTask = 1);

	unsigned int GetWorkerThreadCount();
}
