//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <math.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

/// Check whether two bounding boxes overlap
///
/// @param[in]	boundAMin	Min axis extents of bounding box A
/// @param[in]	boundAMax	Max axis extents of bounding box A
/// @param[in]	boundBMin	Min axis extents of bounding box B
/// @param[in]	boundBMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const float* boundAMin, const float* boundAMax, const float* boundBMin, const float* boundBMax)
{
	return
		boundAMin[0] <= boundBMax[0] && boundAMax[0] >= boundBMin[0] &&
		boundAMin[1] <= boundBMax[1] && boundAMax[1] >= boundBMin[1] &&
		boundAMin[2] <= boundBMax[2] && boundAMax[2] >= boundBMin[2];
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	heightfield		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocateSpan(rcHeightfield& heightfield)
{
	// If necessary, allocate new page and update the freelist.
	if (heightfield.freelist == nullptr || heightfield.freelist->next == nullptr)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* spanPool = (rcSpanPool*)rcAllocate(sizeof(rcSpanPool), RC_ALLOC_PERMANENT);
		if (spanPool == nullptr)
			return nullptr;

		// Add the pool into the list of pools.
		spanPool->next = heightfield.pools;
		heightfield.pools = spanPool;

		// Add new spans to the free list.
		rcSpan* freeList = heightfield.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		} while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	heightfield		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& heightfield, rcSpan* span)
{
	if (span == nullptr)
		return;

	// Add the span to the front of the free list.
	span->next = heightfield.freelist;
	heightfield.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	heightfield			Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaId				The new span's areaId type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge areaId type IDs
static bool addSpan(rcHeightfield& heightfield, const int x, const int z, const unsigned short min, const unsigned short max, const unsigned char areaId, const int flagMergeThreshold)
{
	// Create the new span.
	rcSpan* newSpan = allocateSpan(heightfield);
	if (newSpan == nullptr)
	{
		return false;
	}
	newSpan->smin = min;
	newSpan->smax = max;
	newSpan->area = areaId;
	newSpan->next = nullptr;

	const int columnIndex = x + z * heightfield.width;
	rcSpan* previousSpan = nullptr;
	rcSpan* currentSpan = heightfield.spans[columnIndex];

	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != nullptr)
	{
		if (currentSpan->smin > newSpan->smax)
		{
			// Current span is completely after the new span, break.
			break;
		}

		if (currentSpan->smax < newSpan->smin)
		{
			// Current span is completely before the new span.  Keep going.
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			if (currentSpan->smin < newSpan->smin)
			{
				newSpan->smin = currentSpan->smin;
			}
			if (currentSpan->smax > newSpan->smax)
			{
				newSpan->smax = currentSpan->smax;
			}

			// Merge flags.
			if (rcAbs((int)newSpan->smax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// Higher areaId ID numbers indicate higher resolution priority.
				newSpan->area = rcMax(newSpan->area, currentSpan->area);
			}

			// Remove the current span since it's now merged with newSpan.
			// Keep going because there might be other overlapping spans that also need to be merged.
			rcSpan* next = currentSpan->next;
			freeSpan(heightfield, currentSpan);
			if (previousSpan)
			{
				previousSpan->next = next;
			}
			else
			{
				heightfield.spans[columnIndex] = next;
			}
			currentSpan = next;
		}
	}

	// Insert new span after prev
	if (previousSpan != nullptr)
	{
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// This span should go before the others in the list
		newSpan->next = heightfield.spans[columnIndex];
		heightfield.spans[columnIndex] = newSpan;
	}

	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
	const int x, const int z,
	const unsigned short spanMin, const unsigned short spanMax,
	const unsigned char areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

enum rcAxis
{
	RC_AXIS_X = 0,
	RC_AXIS_Y = 1,
	RC_AXIS_Z = 2
};

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
/// 
/// @param[in]	inputVertices			The input polygon vertices
/// @param[in]	inputVerticesCount		The number of input polygon vertices
/// @param[out]	outputVertices1			Resulting polygon 1's vertices
/// @param[out]	outputVertices1Count	The number of resulting polygon 1 vertices
/// @param[out]	outputVertices2			Resulting polygon 2's vertices
/// @param[out]	outputVertices2Count	The number of resulting polygon 2 vertices
/// @param[in]	axisOffset				The offset along the specified axis
/// @param[in]	axis					The separating axis
static void dividePolygon(const float* inputVertices, int inputVerticesCount,
	float* outputVertices1, int* outputVertices1Count,
	float* outputVertices2, int* outputVertices2Count,
	float axisOffset, rcAxis axis)
{
	rcAssert(inputVerticesCount <= 12);

	// How far positive or negative away from the separating axis is each vertex.
	float inVertAxisDelta[12];
	for (int inVert = 0; inVert < inputVerticesCount; ++inVert)
	{
		inVertAxisDelta[inVert] = axisOffset - inputVertices[inVert * 3 + axis];
	}

	int poly1Vert = 0;
	int poly2Vert = 0;
	for (int inVertA = 0, inVertB = inputVerticesCount - 1; inVertA < inputVerticesCount; inVertB = inVertA, ++inVertA)
	{
		// If the two vertices are on the same side of the separating axis
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

		if (!sameSide)
		{
			float s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
			outputVertices1[poly1Vert * 3 + 0] = inputVertices[inVertB * 3 + 0] + (inputVertices[inVertA * 3 + 0] - inputVertices[inVertB * 3 + 0]) * s;
			outputVertices1[poly1Vert * 3 + 1] = inputVertices[inVertB * 3 + 1] + (inputVertices[inVertA * 3 + 1] - inputVertices[inVertB * 3 + 1]) * s;
			outputVertices1[poly1Vert * 3 + 2] = inputVertices[inVertB * 3 + 2] + (inputVertices[inVertA * 3 + 2] - inputVertices[inVertB * 3 + 2]) * s;
			rcCopyVector(&outputVertices2[poly2Vert * 3], &outputVertices1[poly1Vert * 3]);
			poly1Vert++;
			poly2Vert++;

			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (inVertAxisDelta[inVertA] > 0)
			{
				rcCopyVector(&outputVertices1[poly1Vert * 3], &inputVertices[inVertA * 3]);
				poly1Vert++;
			}
			else if (inVertAxisDelta[inVertA] < 0)
			{
				rcCopyVector(&outputVertices2[poly2Vert * 3], &inputVertices[inVertA * 3]);
				poly2Vert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (inVertAxisDelta[inVertA] >= 0)
			{
				rcCopyVector(&outputVertices1[poly1Vert * 3], &inputVertices[inVertA * 3]);
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0)
				{
					continue;
				}
			}
			rcCopyVector(&outputVertices2[poly2Vert * 3], &inputVertices[inVertA * 3]);
			poly2Vert++;
		}
	}

	*outputVertices1Count = poly1Vert;
	*outputVertices2Count = poly2Vert;
}

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// @param[in] 	vertex0				Triangle vertex 0
/// @param[in] 	vertex1				Triangle vertex 1
/// @param[in] 	vertex2				Triangle vertex 2
/// @param[in] 	areaId				The areaId ID to assign to the rasterized spans
/// @param[in] 	heightfield			Heightfield to rasterize into
/// @param[in] 	heightfieldBBMin	The min extents of the heightfield bounding box
/// @param[in] 	heightfieldBBMax	The max extents of the heightfield bounding box
/// @param[in] 	cellSize			The x and z axis size of a voxel in the heightfield
/// @param[in] 	inverseCellSize		1 / cellSize
/// @param[in] 	inverseCellHeight	1 / cellHeight
/// @param[in] 	flagMergeThreshold	The threshold in which areaId flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTriangle(const float* vertex0, const float* vertex1, const float* vertex2,
	const unsigned char areaID, rcHeightfield& heightfield,
	const float* heightfieldBBMin, const float* heightfieldBBMax,
	const float cellSize, const float inverseCellSize, const float inverseCellHeight,
	const int flagMergeThreshold)
{
	// Calculate the bounding box of the triangle.
	float triBBMin[3];
	rcCopyVector(triBBMin, vertex0);
	rcVmin(triBBMin, vertex1);
	rcVmin(triBBMin, vertex2);

	float triBBMax[3];
	rcCopyVector(triBBMax, vertex0);
	rcVmax(triBBMax, vertex1);
	rcVmax(triBBMax, vertex2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if (!overlapBounds(triBBMin, triBBMax, heightfieldBBMin, heightfieldBBMax))
	{
		return true;
	}

	const int w = heightfield.width;
	const int h = heightfield.height;
	const float by = heightfieldBBMax[1] - heightfieldBBMin[1];

	// Calculate the footprint of the triangle on the grid's z-axis
	int z0 = (int)((triBBMin[2] - heightfieldBBMin[2]) * inverseCellSize);
	int z1 = (int)((triBBMax[2] - heightfieldBBMin[2]) * inverseCellSize);

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	z0 = rcClamp(z0, -1, h - 1);
	z1 = rcClamp(z1, 0, h - 1);

	// Clip the triangle into all grid cells it touches.
	float buf[7 * 3 * 4];
	float* in = buf;
	float* inRow = buf + 7 * 3;
	float* p1 = inRow + 7 * 3;
	float* p2 = p1 + 7 * 3;

	rcCopyVector(&in[0], vertex0);
	rcCopyVector(&in[1 * 3], vertex1);
	rcCopyVector(&in[2 * 3], vertex2);
	int nvRow;
	int nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cellZ = heightfieldBBMin[2] + (float)z * cellSize;
		dividePolygon(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ + cellSize, RC_AXIS_Z);
		rcSwap(in, p1);

		if (nvRow < 3)
		{
			continue;
		}
		if (z < 0)
		{
			continue;
		}

		// find X-axis bounds of the row
		float minX = inRow[0];
		float maxX = inRow[0];
		for (int vert = 1; vert < nvRow; ++vert)
		{
			if (minX > inRow[vert * 3])
			{
				minX = inRow[vert * 3];
			}
			if (maxX < inRow[vert * 3])
			{
				maxX = inRow[vert * 3];
			}
		}
		int x0 = (int)((minX - heightfieldBBMin[0]) * inverseCellSize);
		int x1 = (int)((maxX - heightfieldBBMin[0]) * inverseCellSize);
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}
		x0 = rcClamp(x0, -1, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		int nv;
		int nv2 = nvRow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = heightfieldBBMin[0] + (float)x * cellSize;
			dividePolygon(inRow, nv2, p1, &nv, p2, &nv2, cx + cellSize, RC_AXIS_X);
			rcSwap(inRow, p2);

			if (nv < 3)
			{
				continue;
			}
			if (x < 0)
			{
				continue;
			}

			// Calculate min and max of the span.
			float spanMin = p1[1];
			float spanMax = p1[1];
			for (int vert = 1; vert < nv; ++vert)
			{
				spanMin = rcMin(spanMin, p1[vert * 3 + 1]);
				spanMax = rcMax(spanMax, p1[vert * 3 + 1]);
			}
			spanMin -= heightfieldBBMin[1];
			spanMax -= heightfieldBBMin[1];

			// Skip the span if it's completely outside the heightfield bounding box
			if (spanMax < 0.0f)
			{
				continue;
			}
			if (spanMin > by)
			{
				continue;
			}

			// Clamp the span to the heightfield bounding box.
			if (spanMin < 0.0f)
			{
				spanMin = 0;
			}
			if (spanMax > by)
			{
				spanMax = by;
			}

			// Snap the span to the heightfield height grid.
			unsigned short spanMinCellIndex = (unsigned short)rcClamp((int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short spanMaxCellIndex = (unsigned short)rcClamp((int)ceilf(spanMax * inverseCellHeight), (int)spanMinCellIndex + 1, RC_SPAN_MAX_HEIGHT);

			if (!addSpan(heightfield, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold))
			{
				return false;
			}
		}
	}

	return true;
}

bool rcRasterizeTriangle(rcContext* context,
	const float* vertex0, const float* vertex1, const float* vertex2,
	const unsigned char areaId, rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != nullptr);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	const float inverseCellSize = 1.0f / heightfield.cellSize;
	const float inverseCellHeight = 1.0f / heightfield.cellHeight;
	if (!rasterizeTriangle(vertex0, vertex1, vertex2, areaId, heightfield, heightfield.boundMin, heightfield.boundMax, heightfield.cellSize, inverseCellSize, inverseCellHeight, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
	const float* vertices, const int /*nv*/,
	const int* triangles, const unsigned char* triangleAreaIds, const int trianglesCount,
	rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != nullptr);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cellSize;
	const float inverseCellHeight = 1.0f / heightfield.cellHeight;
	for (int triIndex = 0; triIndex < trianglesCount; ++triIndex)
	{
		const float* v0 = &vertices[triangles[triIndex * 3 + 0] * 3];
		const float* v1 = &vertices[triangles[triIndex * 3 + 1] * 3];
		const float* v2 = &vertices[triangles[triIndex * 3 + 2] * 3];
		if (!rasterizeTriangle(v0, v1, v2, triangleAreaIds[triIndex], heightfield, heightfield.boundMin, heightfield.boundMax, heightfield.cellSize, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
	const float* vertices, const int /*nv*/,
	const unsigned short* triangles, const unsigned char* triAreaIds, const int trianglesCount,
	rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != nullptr);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cellSize;
	const float inverseCellHeight = 1.0f / heightfield.cellHeight;
	for (int triIndex = 0; triIndex < trianglesCount; ++triIndex)
	{
		const float* v0 = &vertices[triangles[triIndex * 3 + 0] * 3];
		const float* v1 = &vertices[triangles[triIndex * 3 + 1] * 3];
		const float* v2 = &vertices[triangles[triIndex * 3 + 2] * 3];
		if (!rasterizeTriangle(v0, v1, v2, triAreaIds[triIndex], heightfield, heightfield.boundMin, heightfield.boundMax, heightfield.cellSize, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
	const float* vertices, const unsigned char* triangleAreaIds, const int trianglesCount,
	rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != nullptr);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cellSize;
	const float inverseCellHeight = 1.0f / heightfield.cellHeight;
	for (int triIndex = 0; triIndex < trianglesCount; ++triIndex)
	{
		const float* v0 = &vertices[(triIndex * 3 + 0) * 3];
		const float* v1 = &vertices[(triIndex * 3 + 1) * 3];
		const float* v2 = &vertices[(triIndex * 3 + 2) * 3];
		if (!rasterizeTriangle(v0, v1, v2, triangleAreaIds[triIndex], heightfield, heightfield.boundMin, heightfield.boundMax, heightfield.cellSize, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}
