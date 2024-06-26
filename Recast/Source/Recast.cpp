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

#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

namespace
{
	/// Allocates and constructs an object of the given type, returning a pointer.
	/// @param[in]		allocLifetime	Allocation lifetime hint
	template<typename T>
	T* rcNew(const rcAllocateHint allocLifetime)
	{
		T* ptr = (T*)rcAllocate(sizeof(T), allocLifetime);
		::new(rcNewTag(), (void*)ptr) T();
		return ptr;
	}

	/// Destroys and frees an object allocated with rcNew.
	/// @param[in]     ptr    The object pointer to delete.
	template<typename T>
	void rcDelete(T* ptr)
	{
		if (ptr)
		{
			ptr->~T();
			rcFree((void*)ptr);
		}
	}
} // anonymous namespace

float rcSqrt(float x)
{
	return sqrtf(x);
}

void rcContext::log(const rcLogCategory category, const char* format, ...)
{
	if (!m_logEnabled)
	{
		return;
	}
	static const int MSG_SIZE = 512;
	char msg[MSG_SIZE];
	va_list argList;
	va_start(argList, format);
	int len = vsnprintf(msg, MSG_SIZE, format, argList);
	if (len >= MSG_SIZE)
	{
		len = MSG_SIZE - 1;
		msg[MSG_SIZE - 1] = '\0';

		const char* errorMessage = "Log message was truncated";
		doLog(RC_LOG_ERROR, errorMessage, (int)strlen(errorMessage));
	}
	va_end(argList);
	doLog(category, msg, len);
}

void rcContext::doResetLog()
{
	// Defined out of line to fix the weak v-tables warning
}

rcHeightfield* rcAllocateHeightfield()
{
	return rcNew<rcHeightfield>(RC_ALLOC_PERMANENT);
}

void rcFreeHeightField(rcHeightfield* heightfield)
{
	rcDelete(heightfield);
}

rcHeightfield::rcHeightfield()
	: width()
	, height()
	, boundMin()
	, boundMax()
	, cellSize()
	, cellHeight()
	, spans()
	, pools()
	, freelist()
{
}

rcHeightfield::~rcHeightfield()
{
	// Delete span array.
	rcFree(spans);
	// Delete span pools.
	while (pools)
	{
		rcSpanPool* next = pools->next;
		rcFree(pools);
		pools = next;
	}
}

rcCompactHeightfield* rcAllocateCompactHeightfield()
{
	return rcNew<rcCompactHeightfield>(RC_ALLOC_PERMANENT);
}

void rcFreeCompactHeightfield(rcCompactHeightfield* compactHeightfield)
{
	rcDelete(compactHeightfield);
}

rcCompactHeightfield::rcCompactHeightfield()
	: width()
	, height()
	, spanCount()
	, walkableHeight()
	, walkableClimb()
	, borderSize()
	, maxDistanceToBorder()
	, maxRegions()
	, boundMin()
	, boundMax()
	, cellSize()
	, cellHeight()
	, cells()
	, spans()
	, distancesToBorder()
	, areaIds()
{
}

rcCompactHeightfield::~rcCompactHeightfield()
{
	rcFree(cells);
	rcFree(spans);
	rcFree(distancesToBorder);
	rcFree(areaIds);
}

rcHeightfieldLayerSet* rcAllocateHeightfieldLayerSet()
{
	return rcNew<rcHeightfieldLayerSet>(RC_ALLOC_PERMANENT);
}

void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* layerSet)
{
	rcDelete(layerSet);
}

rcHeightfieldLayerSet::rcHeightfieldLayerSet()
	: layers()
	, nlayers()
{
}

rcHeightfieldLayerSet::~rcHeightfieldLayerSet()
{
	for (int i = 0; i < nlayers; ++i)
	{
		rcFree(layers[i].heights);
		rcFree(layers[i].areaIds);
		rcFree(layers[i].connections);
	}
	rcFree(layers);
}


rcContourSet* rcAllocateContourSet()
{
	return rcNew<rcContourSet>(RC_ALLOC_PERMANENT);
}

void rcFreeContourSet(rcContourSet* contourSet)
{
	rcDelete(contourSet);
}

rcContourSet::rcContourSet()
	: contours()
	, contoursCount()
	, boundMin()
	, boundMax()
	, cellSize()
	, cellHeight()
	, width()
	, height()
	, borderSize()
	, maxError()
{
}

rcContourSet::~rcContourSet()
{
	for (int i = 0; i < contoursCount; ++i)
	{
		rcFree(contours[i].vertices);
		rcFree(contours[i].rawVertices);
	}
	rcFree(contours);
}

rcPolyMesh* rcAllocatePolyMesh()
{
	return rcNew<rcPolyMesh>(RC_ALLOC_PERMANENT);
}

void rcFreePolyMesh(rcPolyMesh* polyMesh)
{
	rcDelete(polyMesh);
}

rcPolyMesh::rcPolyMesh()
	: vertices()
	, polygons()
	, regionIds()
	, flags()
	, areaIds()
	, verticesCount()
	, polygonsCount()
	, maxAllocatedPolygons()
	, maxVerticesPerPolygon()
	, boundMin()
	, boundMax()
	, cellSize()
	, cellHeight()
	, borderSize()
	, maxEdgeError()
{
}

rcPolyMesh::~rcPolyMesh()
{
	rcFree(vertices);
	rcFree(polygons);
	rcFree(regionIds);
	rcFree(flags);
	rcFree(areaIds);
}

rcPolyMeshDetail* rcAllocatePolyMeshDetail()
{
	return rcNew<rcPolyMeshDetail>(RC_ALLOC_PERMANENT);
}

void rcFreePolyMeshDetail(rcPolyMeshDetail* detailMesh)
{
	if (detailMesh == NULL)
	{
		return;
	}
	rcFree(detailMesh->meshes);
	rcFree(detailMesh->vertices);
	rcFree(detailMesh->triangles);
	rcFree(detailMesh);
}

rcPolyMeshDetail::rcPolyMeshDetail()
	: meshes()
	, vertices()
	, triangles()
	, meshesCount()
	, verticesCount()
	, trianglesCount()
{
}

void rcCalculateBounds(const float* verts, int numVerts, float* minBounds, float* maxBounds)
{
	// Calculate bounding box.
	rcCopyVector(minBounds, verts);
	rcCopyVector(maxBounds, verts);
	for (int i = 1; i < numVerts; ++i)
	{
		const float* v = &verts[i * 3];
		rcVmin(minBounds, v);
		rcVmax(maxBounds, v);
	}
}

void rcCalculateGridSize(const float* minBounds, const float* maxBounds, const float cellSize, int* sizeX, int* sizeZ)
{
	*sizeX = (int)((maxBounds[0] - minBounds[0]) / cellSize + 0.5f);
	*sizeZ = (int)((maxBounds[2] - minBounds[2]) / cellSize + 0.5f);
}

bool rcCreateHeightfield(rcContext* context, rcHeightfield& heightfield, int sizeX, int sizeZ,
	const float* minBounds, const float* maxBounds,
	float cellSize, float cellHeight)
{
	rcIgnoreUnused(context);

	heightfield.width = sizeX;
	heightfield.height = sizeZ;
	rcCopyVector(heightfield.boundMin, minBounds);
	rcCopyVector(heightfield.boundMax, maxBounds);
	heightfield.cellSize = cellSize;
	heightfield.cellHeight = cellHeight;
	heightfield.spans = (rcSpan**)rcAllocate(sizeof(rcSpan*) * heightfield.width * heightfield.height, RC_ALLOC_PERMANENT);
	if (!heightfield.spans)
	{
		return false;
	}
	memset(heightfield.spans, 0, sizeof(rcSpan*) * heightfield.width * heightfield.height);
	return true;
}

static void CalculateTriangleNormal(const float* vertex0, const float* vertex1, const float* vertex2, float* faceNormal)
{
	float e0[3], e1[3];
	rcSubtractVector(e0, vertex1, vertex0);
	rcSubtractVector(e1, vertex2, vertex0);
	rcCrossProduct(faceNormal, e0, e1);
	rcNormalize(faceNormal);
}

void rcMarkWalkableTriangles(rcContext* context, const float walkableSlopeAngle,
	const float* vertices, const int verticesCount,
	const int* triangles, const int trianglesCount,
	unsigned char* triangleAreaIds)
{
	rcIgnoreUnused(context);
	rcIgnoreUnused(verticesCount);

	constexpr float DEGREES_TO_RADIANS = RC_PI / 180.0f;
	const float walkableThreshold = cosf(walkableSlopeAngle * DEGREES_TO_RADIANS);

	float triangleNormal[3];

	for (int i = 0; i < trianglesCount; ++i)
	{
		const int* tri = &triangles[i * 3];
		CalculateTriangleNormal(&vertices[tri[0] * 3], &vertices[tri[1] * 3], &vertices[tri[2] * 3], triangleNormal);

		// Check if the face is walkable.
		if (triangleNormal[1] > walkableThreshold)
			triangleAreaIds[i] = RC_WALKABLE_AREA_ID;
	}
}

void rcClearUnwalkableTriangles(rcContext* context, const float walkableSlopeAngle,
	const float* vertices, int verticesCount,
	const int* triangles, int trianglesCount,
	unsigned char* triangleAreaIds)
{
	rcIgnoreUnused(context);
	rcIgnoreUnused(verticesCount);

	// The minimum Y value for a face triangleNormal of a triangle with a walkable slope.
	const float walkableLimitY = cosf(walkableSlopeAngle / 180.0f * RC_PI);

	float triangleNormal[3];
	for (int i = 0; i < trianglesCount; ++i)
	{
		const int* tri = &triangles[i * 3];
		CalculateTriangleNormal(&vertices[tri[0] * 3], &vertices[tri[1] * 3], &vertices[tri[2] * 3], triangleNormal);
		// Check if the face is walkable.
		if (triangleNormal[1] <= walkableLimitY)
			triangleAreaIds[i] = RC_NULL_AREA;
	}
}

int rcGetHeightFieldSpanCount(rcContext* context, const rcHeightfield& heightfield)
{
	rcIgnoreUnused(context);

	const int columnsCount = heightfield.width * heightfield.height;
	int spanCount = 0;
	for (int columnIndex = 0; columnIndex < columnsCount; ++columnIndex)
	{
		for (rcSpan* span = heightfield.spans[columnIndex]; span != nullptr; span = span->next)
		{
			if (span->area != RC_NULL_AREA)
				spanCount++;
		}
	}
	return spanCount;
}

bool rcBuildCompactHeightfield(rcContext* context, const int walkableHeight, const int walkableClimb,
	const rcHeightfield& heightfield, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int spanCount = rcGetHeightFieldSpanCount(context, heightfield);

	// Fill in header.
	compactHeightfield.width = xSize;
	compactHeightfield.height = zSize;
	compactHeightfield.spanCount = spanCount;
	compactHeightfield.walkableHeight = walkableHeight;
	compactHeightfield.walkableClimb = walkableClimb;
	compactHeightfield.maxRegions = 0;
	rcCopyVector(compactHeightfield.boundMin, heightfield.boundMin);
	rcCopyVector(compactHeightfield.boundMax, heightfield.boundMax);
	compactHeightfield.boundMax[1] += walkableHeight * heightfield.cellHeight;
	compactHeightfield.cellSize = heightfield.cellSize;
	compactHeightfield.cellHeight = heightfield.cellHeight;
	compactHeightfield.cells = (rcCompactCell*)rcAllocate(sizeof(rcCompactCell) * xSize * zSize, RC_ALLOC_PERMANENT);
	if (!compactHeightfield.cells)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", xSize * zSize);
		return false;
	}
	memset(compactHeightfield.cells, 0, sizeof(rcCompactCell) * xSize * zSize);
	compactHeightfield.spans = (rcCompactSpan*)rcAllocate(sizeof(rcCompactSpan) * spanCount, RC_ALLOC_PERMANENT);
	if (!compactHeightfield.spans)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.spans, 0, sizeof(rcCompactSpan) * spanCount);
	compactHeightfield.areaIds = (unsigned char*)rcAllocate(sizeof(unsigned char) * spanCount, RC_ALLOC_PERMANENT);
	if (!compactHeightfield.areaIds)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.areaIds, RC_NULL_AREA, sizeof(unsigned char) * spanCount);

	constexpr int MAX_HEIGHT = 0xffff;

	// Fill in cells and spans.
	int currentCellIndex = 0;
	const int columnsCount = xSize * zSize;
	for (int columnIndex = 0; columnIndex < columnsCount; ++columnIndex)
	{
		const rcSpan* span = heightfield.spans[columnIndex];

		// If there are no spans at this cell, just leave the data to index=0, count=0.
		if (span == nullptr)
			continue;

		rcCompactCell& cell = compactHeightfield.cells[columnIndex];
		cell.index = currentCellIndex;
		cell.count = 0;

		for (; span != nullptr; span = span->next)
		{
			if (span->area != RC_NULL_AREA)
			{
				const int bottom = (int)span->smax;
				const int top = span->next ? (int)span->next->smin : MAX_HEIGHT;
				compactHeightfield.spans[currentCellIndex].y = (unsigned short)rcClamp(bottom, 0, 0xffff);
				compactHeightfield.spans[currentCellIndex].height = (unsigned char)rcClamp(top - bottom, 0, 0xff);
				compactHeightfield.areaIds[currentCellIndex] = span->area;
				currentCellIndex++;
				cell.count++;
			}
		}
	}

	// Find neighbour connections.
	constexpr int MAX_LAYERS = RC_NOT_CONNECTED - 1;
	int maxLayerIndex = 0;
	const int zStride = xSize; // for readability
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				rcCompactSpan& span = compactHeightfield.spans[i];

				for (int direction = 0; direction < 4; ++direction)
				{
					rcSetConnection(span, direction, RC_NOT_CONNECTED);
					const int neighborX = x + rcGetDirectionOffsetX(direction);
					const int neighborZ = z + rcGetDirectionOffsetY(direction);

					// First check that the neighbour cell is in bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
						continue;

					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					const rcCompactCell& neighborCell = compactHeightfield.cells[neighborX + neighborZ * zStride];
					for (int k = (int)neighborCell.index, nk = (int)(neighborCell.index + neighborCell.count); k < nk; ++k)
					{
						const rcCompactSpan& neighborSpan = compactHeightfield.spans[k];
						const int bottom = rcMax(span.y, neighborSpan.y);
						const int top = rcMin(span.y + span.height, neighborSpan.y + neighborSpan.height);

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if ((top - bottom) >= walkableHeight && rcAbs((int)neighborSpan.y - (int)span.y) <= walkableClimb)
						{
							// Mark direction as walkable.
							const int layerIndex = k - (int)neighborCell.index;
							if (layerIndex < 0 || layerIndex > MAX_LAYERS)
							{
								maxLayerIndex = rcMax(maxLayerIndex, layerIndex);
								continue;
							}
							rcSetConnection(span, direction, layerIndex);
							break;
						}
					}
				}
			}
		}
	}

	if (maxLayerIndex > MAX_LAYERS)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
			maxLayerIndex, MAX_LAYERS);
	}

	return true;
}
