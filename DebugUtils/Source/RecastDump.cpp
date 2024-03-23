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
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastDump.h"

duFileIO::~duFileIO()
{
	// Defined out of line to fix the weak v-tables warning
}
	
static void ioprintf(duFileIO* io, const char* format, ...)
{
	char line[256];
	va_list ap;
	va_start(ap, format);
	const int n = vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	if (n > 0)
		io->write(line, sizeof(char)*n);
}

bool duDumpPolyMeshToObj(rcPolyMesh& pmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshToObj: input IO not writing.\n"); 
		return false;
	}
	
	const int nvp = pmesh.maxVerticesPerPolygon;
	const float cs = pmesh.cellSize;
	const float ch = pmesh.cellHeight;
	const float* orig = pmesh.boundMin;
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");

	ioprintf(io, "\n");
	
	for (int i = 0; i < pmesh.verticesCount; ++i)
	{
		const unsigned short* v = &pmesh.vertices[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		ioprintf(io, "v %f %f %f\n", x,y,z);
	}

	ioprintf(io, "\n");

	for (int i = 0; i < pmesh.polygonsCount; ++i)
	{
		const unsigned short* p = &pmesh.polygons[i*nvp*2];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			ioprintf(io, "f %d %d %d\n", p[0]+1, p[j-1]+1, p[j]+1); 
		}
	}
	
	return true;
}

bool duDumpPolyMeshDetailToObj(rcPolyMeshDetail& dmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshDetailToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshDetailToObj: input IO not writing.\n"); 
		return false;
	}
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");
	
	ioprintf(io, "\n");

	for (int i = 0; i < dmesh.verticesCount; ++i)
	{
		const float* v = &dmesh.vertices[i*3];
		ioprintf(io, "v %f %f %f\n", v[0],v[1],v[2]);
	}
	
	ioprintf(io, "\n");
	
	for (int i = 0; i < dmesh.meshesCount; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const unsigned int ntris = m[3];
		const unsigned char* tris = &dmesh.triangles[btris*4];
		for (unsigned int j = 0; j < ntris; ++j)
		{
			ioprintf(io, "f %d %d %d\n",
					(int)(bverts+tris[j*4+0])+1,
					(int)(bverts+tris[j*4+1])+1,
					(int)(bverts+tris[j*4+2])+1);
		}
	}
	
	return true;
}

static const int CSET_MAGIC = ('c' << 24) | ('s' << 16) | ('e' << 8) | 't';
static const int CSET_VERSION = 2;

bool duDumpContourSet(struct rcContourSet& cset, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpContourSet: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpContourSet: input IO not writing.\n"); 
		return false;
	}
	
	io->write(&CSET_MAGIC, sizeof(CSET_MAGIC));
	io->write(&CSET_VERSION, sizeof(CSET_VERSION));

	io->write(&cset.contoursCount, sizeof(cset.contoursCount));
	
	io->write(cset.boundMin, sizeof(cset.boundMin));
	io->write(cset.boundMax, sizeof(cset.boundMax));
	
	io->write(&cset.cellSize, sizeof(cset.cellSize));
	io->write(&cset.cellHeight, sizeof(cset.cellHeight));

	io->write(&cset.width, sizeof(cset.width));
	io->write(&cset.height, sizeof(cset.height));
	io->write(&cset.borderSize, sizeof(cset.borderSize));

	for (int i = 0; i < cset.contoursCount; ++i)
	{
		const rcContour& cont = cset.contours[i];
		io->write(&cont.verticesCount, sizeof(cont.verticesCount));
		io->write(&cont.rawVerticesCount, sizeof(cont.rawVerticesCount));
		io->write(&cont.regionId, sizeof(cont.regionId));
		io->write(&cont.areaId, sizeof(cont.areaId));
		io->write(cont.vertices, sizeof(int)*4*cont.verticesCount);
		io->write(cont.rawVertices, sizeof(int)*4*cont.rawVerticesCount);
	}

	return true;
}

bool duReadContourSet(struct rcContourSet& cset, duFileIO* io)
{
	if (!io)
	{
		printf("duReadContourSet: input IO is null.\n"); 
		return false;
	}
	if (!io->isReading())
	{
		printf("duReadContourSet: input IO not reading.\n"); 
		return false;
	}
	
	int magic = 0;
	int version = 0;
	
	io->read(&magic, sizeof(magic));
	io->read(&version, sizeof(version));
	
	if (magic != CSET_MAGIC)
	{
		printf("duReadContourSet: Bad voodoo.\n");
		return false;
	}
	if (version != CSET_VERSION)
	{
		printf("duReadContourSet: Bad version.\n");
		return false;
	}
	
	io->read(&cset.contoursCount, sizeof(cset.contoursCount));

	cset.contours = (rcContour*)rcAlloc(sizeof(rcContour)*cset.contoursCount, RC_ALLOC_PERM);
	if (!cset.contours)
	{
		printf("duReadContourSet: Could not alloc contours (%d)\n", cset.contoursCount);
		return false;
	}
	memset(cset.contours, 0, sizeof(rcContour)*cset.contoursCount);
	
	io->read(cset.boundMin, sizeof(cset.boundMin));
	io->read(cset.boundMax, sizeof(cset.boundMax));
	
	io->read(&cset.cellSize, sizeof(cset.cellSize));
	io->read(&cset.cellHeight, sizeof(cset.cellHeight));
	
	io->read(&cset.width, sizeof(cset.width));
	io->read(&cset.height, sizeof(cset.height));
	io->read(&cset.borderSize, sizeof(cset.borderSize));
	
	for (int i = 0; i < cset.contoursCount; ++i)
	{
		rcContour& cont = cset.contours[i];
		io->read(&cont.verticesCount, sizeof(cont.verticesCount));
		io->read(&cont.rawVerticesCount, sizeof(cont.rawVerticesCount));
		io->read(&cont.regionId, sizeof(cont.regionId));
		io->read(&cont.areaId, sizeof(cont.areaId));

		cont.vertices = (int*)rcAlloc(sizeof(int)*4*cont.verticesCount, RC_ALLOC_PERM);
		if (!cont.vertices)
		{
			printf("duReadContourSet: Could not alloc contour verts (%d)\n", cont.verticesCount);
			return false;
		}
		cont.rawVertices = (int*)rcAlloc(sizeof(int)*4*cont.rawVerticesCount, RC_ALLOC_PERM);
		if (!cont.rawVertices)
		{
			printf("duReadContourSet: Could not alloc contour rverts (%d)\n", cont.rawVerticesCount);
			return false;
		}
		
		io->read(cont.vertices, sizeof(int)*4*cont.verticesCount);
		io->read(cont.rawVertices, sizeof(int)*4*cont.rawVerticesCount);
	}
	
	return true;
}
	

static const int CHF_MAGIC = ('r' << 24) | ('c' << 16) | ('h' << 8) | 'f';
static const int CHF_VERSION = 3;

bool duDumpCompactHeightfield(struct rcCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpCompactHeightfield: input IO not writing.\n"); 
		return false;
	}
	
	io->write(&CHF_MAGIC, sizeof(CHF_MAGIC));
	io->write(&CHF_VERSION, sizeof(CHF_VERSION));
	
	io->write(&chf.width, sizeof(chf.width));
	io->write(&chf.height, sizeof(chf.height));
	io->write(&chf.spanCount, sizeof(chf.spanCount));

	io->write(&chf.walkableHeight, sizeof(chf.walkableHeight));
	io->write(&chf.walkableClimb, sizeof(chf.walkableClimb));
	io->write(&chf.borderSize, sizeof(chf.borderSize));

	io->write(&chf.maxDistanceToBorder, sizeof(chf.maxDistanceToBorder));
	io->write(&chf.maxRegions, sizeof(chf.maxRegions));

	io->write(chf.boundMin, sizeof(chf.boundMin));
	io->write(chf.boundMax, sizeof(chf.boundMax));

	io->write(&chf.cellSize, sizeof(chf.cellSize));
	io->write(&chf.cellHeight, sizeof(chf.cellHeight));

	int tmp = 0;
	if (chf.cells) tmp |= 1;
	if (chf.spans) tmp |= 2;
	if (chf.distancesToBorder) tmp |= 4;
	if (chf.areaIds) tmp |= 8;

	io->write(&tmp, sizeof(tmp));

	if (chf.cells)
		io->write(chf.cells, sizeof(rcCompactCell)*chf.width*chf.height);
	if (chf.spans)
		io->write(chf.spans, sizeof(rcCompactSpan)*chf.spanCount);
	if (chf.distancesToBorder)
		io->write(chf.distancesToBorder, sizeof(unsigned short)*chf.spanCount);
	if (chf.areaIds)
		io->write(chf.areaIds, sizeof(unsigned char)*chf.spanCount);

	return true;
}

bool duReadCompactHeightfield(struct rcCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duReadCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isReading())
	{
		printf("duReadCompactHeightfield: input IO not reading.\n"); 
		return false;
	}

	int magic = 0;
	int version = 0;
	
	io->read(&magic, sizeof(magic));
	io->read(&version, sizeof(version));
	
	if (magic != CHF_MAGIC)
	{
		printf("duReadCompactHeightfield: Bad voodoo.\n");
		return false;
	}
	if (version != CHF_VERSION)
	{
		printf("duReadCompactHeightfield: Bad version.\n");
		return false;
	}
	
	io->read(&chf.width, sizeof(chf.width));
	io->read(&chf.height, sizeof(chf.height));
	io->read(&chf.spanCount, sizeof(chf.spanCount));
	
	io->read(&chf.walkableHeight, sizeof(chf.walkableHeight));
	io->read(&chf.walkableClimb, sizeof(chf.walkableClimb));
	io->read(&chf.borderSize, sizeof(chf.borderSize));

	io->read(&chf.maxDistanceToBorder, sizeof(chf.maxDistanceToBorder));
	io->read(&chf.maxRegions, sizeof(chf.maxRegions));
	
	io->read(chf.boundMin, sizeof(chf.boundMin));
	io->read(chf.boundMax, sizeof(chf.boundMax));
	
	io->read(&chf.cellSize, sizeof(chf.cellSize));
	io->read(&chf.cellHeight, sizeof(chf.cellHeight));
	
	int tmp = 0;
	io->read(&tmp, sizeof(tmp));
	
	if (tmp & 1)
	{
		chf.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell)*chf.width*chf.height, RC_ALLOC_PERM);
		if (!chf.cells)
		{
			printf("duReadCompactHeightfield: Could not alloc cells (%d)\n", chf.width*chf.height);
			return false;
		}
		io->read(chf.cells, sizeof(rcCompactCell)*chf.width*chf.height);
	}
	if (tmp & 2)
	{
		chf.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan)*chf.spanCount, RC_ALLOC_PERM);
		if (!chf.spans)
		{
			printf("duReadCompactHeightfield: Could not alloc spans (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.spans, sizeof(rcCompactSpan)*chf.spanCount);
	}
	if (tmp & 4)
	{
		chf.distancesToBorder = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_PERM);
		if (!chf.distancesToBorder)
		{
			printf("duReadCompactHeightfield: Could not alloc dist (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.distancesToBorder, sizeof(unsigned short)*chf.spanCount);
	}
	if (tmp & 8)
	{
		chf.areaIds = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_PERM);
		if (!chf.areaIds)
		{
			printf("duReadCompactHeightfield: Could not alloc areas (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.areaIds, sizeof(unsigned char)*chf.spanCount);
	}
	
	return true;
}


static void logLine(rcContext& ctx, rcTimerLabel label, const char* name, const float pc)
{
	const int t = ctx.getAccumulatedTime(label);
	if (t < 0) return;
	ctx.log(RC_LOG_PROGRESS, "%s:\t%.2fms\t(%.1f%%)", name, t/1000.0f, t*pc);
}

void duLogBuildTimes(rcContext& ctx, const int totalTimeUsec)
{
	const float pc = 100.0f / totalTimeUsec;
 
	ctx.log(RC_LOG_PROGRESS, "Build Times");
	logLine(ctx, RC_TIMER_RASTERIZE_TRIANGLES,		"- Rasterize", pc);
	logLine(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD,	"- Build Compact", pc);
	logLine(ctx, RC_TIMER_FILTER_BORDER,				"- Filter Border", pc);
	logLine(ctx, RC_TIMER_FILTER_WALKABLE,			"- Filter Walkable", pc);
	logLine(ctx, RC_TIMER_ERODE_AREA,				"- Erode Area", pc);
	logLine(ctx, RC_TIMER_MEDIAN_AREA,				"- Median Area", pc);
	logLine(ctx, RC_TIMER_MARK_BOX_AREA,				"- Mark Box Area", pc);
	logLine(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA,		"- Mark Convex Area", pc);
	logLine(ctx, RC_TIMER_MARK_CYLINDER_AREA,		"- Mark Cylinder Area", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD,		"- Build Distance Field", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST,	"    - Distance", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR,	"    - Blur", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS,				"- Build Regions", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_WATERSHED,	"    - Watershed", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_EXPAND,		"      - Expand", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FLOOD,		"      - Find Basins", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FILTER,		"    - Filter", pc);
	logLine(ctx, RC_TIMER_BUILD_LAYERS,				"- Build Layers", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS,			"- Build Contours", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_TRACE,		"    - Trace", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_SIMPLIFY,	"    - Simplify", pc);
	logLine(ctx, RC_TIMER_BUILD_POLYMESH,			"- Build Polymesh", pc);
	logLine(ctx, RC_TIMER_BUILD_POLYMESHDETAIL,		"- Build Polymesh Detail", pc);
	logLine(ctx, RC_TIMER_MERGE_POLYMESH,			"- Merge Polymeshes", pc);
	logLine(ctx, RC_TIMER_MERGE_POLYMESHDETAIL,		"- Merge Polymesh Details", pc);
	ctx.log(RC_LOG_PROGRESS, "=== TOTAL:\t%.2fms", totalTimeUsec/1000.0f);
}

