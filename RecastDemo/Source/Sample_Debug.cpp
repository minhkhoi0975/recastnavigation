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
#include "Sample_Debug.h"
#include "InputGeom.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "RecastDump.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

/*
static int loadBin(const char* path, unsigned char** data)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;
	fseek(fp, 0, SEEK_END);
	int size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	*data = new unsigned char[size];
	fread(*data, size, 1, fp);
	fclose(fp);
	return size;
} 
*/

Sample_Debug::Sample_Debug() :
	m_chf(0),
	m_cset(0),
	m_pmesh(0)
{
	resetCommonSettings();

	// Test
/*	m_chf = rcAllocateCompactHeightfield();
	FileIO io;
	if (!io.openForRead("test.chf"))
	{
		delete m_chf;
		m_chf = 0;
	}
	else
	{
		if (!duReadCompactHeightfield(*m_chf, &io))
		{
			delete m_chf;
			m_chf = 0;
		}
	}*/
	
/*	if (m_chf)
	{
		unsigned short ymin = 0xffff;
		unsigned short ymax = 0;
		for (int i = 0; i < m_chf->spanCount; ++i)
		{
			const rcCompactSpan& s = m_chf->spans[i];
			if (s.y < ymin) ymin = s.y;
			if (s.y > ymax) ymax = s.y;
		}
		printf("ymin=%d ymax=%d\n", (int)ymin, (int)ymax);
		
		int maxSpans = 0;
		for (int i = 0; i < m_chf->width*m_chf->height; ++i)
		{
			maxSpans = rcMax(maxSpans, (int)m_chf->cells[i].count);
		}
		printf("maxSpans = %d\n", maxSpans);
	}*/
	

/*	const float orig[3] = {0,0,0};
	m_navMesh = new dtNavMesh;
	m_navMesh->init(orig, 133.333f,133.333f, 2048, 4096, 4096);

	unsigned char* data = 0;
	int dataSize = 0;
	
	// Tile_-13_-14.bin is basically just the bytes that was output by Detour. It should be loaded at X: -13 and Y: -14.
	
	dataSize = loadBin("Tile_-13_-13.bin", &data);
	if (dataSize > 0)
	{
		m_navMesh->addTileAt(-13,-13, data, dataSize, true);
		dtMeshHeader* header = (dtMeshHeader*)data;
		vcopy(m_bmin, header->boundMin);
		vcopy(m_bmax, header->boundMax);
	}

	dataSize = loadBin("Tile_-13_-14.bin", &data);
	if (dataSize > 0)
	{
		m_navMesh->addTileAt(-13,-14, data, dataSize, true);
	}

	dataSize = loadBin("Tile_-14_-14.bin", &data);
	if (dataSize > 0)
	{
		m_navMesh->addTileAt(-14,-14, data, dataSize, true);
	}
	
	const float halfExtents[3] = {40,100,40};
	const float center[3] = { -1667.9491f, 135.52649f, -1680.6149f };
	dtQueryFilter filter;
	m_ref = m_navMesh->findNearestPoly(center, halfExtents, &filter, 0);

	vcopy(m_halfExtents, halfExtents);
	vcopy(m_center, center);*/
	

	{
		m_cset = rcAllocateContourSet();
		if (m_cset)
		{
			FileIO io;
			if (io.openForRead("PathSet_TMP_NA_PathingTestAReg1_1_2_CS.rc"))
			{
				duReadContourSet(*m_cset, &io);
				
				printf("bmin=(%f,%f,%f) bmax=(%f,%f,%f)\n",
					   m_cset->boundMin[0], m_cset->boundMin[1], m_cset->boundMin[2],
					   m_cset->boundMax[0], m_cset->boundMax[1], m_cset->boundMax[2]);
				printf("cs=%f ch=%f\n", m_cset->cellSize, m_cset->cellHeight);
			}
			else
			{
				printf("could not open test.cset\n");
			}
		}
		else
		{
			printf("Could not alloc cset\n");
		}


/*		if (m_cset)
		{
			m_pmesh = rcAllocatePolyMesh();
			if (m_pmesh)
			{
				rcBuildPolyMesh(m_ctx, *m_cset, 6, *m_pmesh);
			}
		}*/
	}
	
}

Sample_Debug::~Sample_Debug()
{
	rcFreeCompactHeightfield(m_chf);
	rcFreeContourSet(m_cset);
	rcFreePolyMesh(m_pmesh);
}

void Sample_Debug::handleSettings()
{
}

void Sample_Debug::handleTools()
{
}

void Sample_Debug::handleDebugMode()
{
}

void Sample_Debug::handleRender()
{
	if (m_chf)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
//		duDebugDrawCompactHeightfieldSolid(&dd, *m_chf);
	}
		
	if (m_navMesh)
		duDebugDrawNavMesh(&m_dd, *m_navMesh, DU_DRAWNAVMESH_OFFMESHCONS);

	if (m_ref && m_navMesh)
		duDebugDrawNavMeshPoly(&m_dd, *m_navMesh, m_ref, duRGBA(255,0,0,128));

/*	float boundMin[3], boundMax[3];
	rcSubtractVector(boundMin, m_center, m_halfExtents);
	rcAddVector(boundMax, m_center, m_halfExtents);
	duDebugDrawBoxWire(&dd, boundMin[0],boundMin[1],boundMin[2], boundMax[0],boundMax[1],boundMax[2], duRGBA(255,255,255,128), 1.0f);
	duDebugDrawCross(&dd, m_center[0], m_center[1], m_center[2], 1.0f, duRGBA(255,255,255,128), 2.0f);*/

	if (m_cset)
	{
		duDebugDrawRawContours(&m_dd, *m_cset, 0.25f);
		duDebugDrawContours(&m_dd, *m_cset);
	}
	
	if (m_pmesh)
	{
		duDebugDrawPolyMesh(&m_dd, *m_pmesh);
	}
	
	/*
	dd.depthMask(false);
	{
		const float boundMin[3] = {-32.000004f,-11.488281f,-115.343544f};
		const float cellSize = 0.300000f;
		const float cellHeight = 0.200000f;
		const int vertices[] = {
			158,46,336,0,
			157,47,331,0,
			161,53,330,0,
			162,52,335,0,
			158,46,336,0,
			154,46,339,5,
			161,46,365,5,
			171,46,385,5,
			174,46,400,5,
			177,46,404,5,
			177,46,410,5,
			183,46,416,5,
			188,49,416,5,
			193,52,411,6,
			194,53,382,6,
			188,52,376,6,
			188,57,363,6,
			174,57,349,6,
			174,60,342,6,
			168,58,336,6,
			167,59,328,6,
			162,55,324,6,
			159,53,324,5,
			152,46,328,5,
			151,46,336,5,
			154,46,339,5,
			158,46,336,0,
			160,46,340,0,
			164,52,339,0,
			168,55,343,0,
			168,50,351,0,
			182,54,364,0,
			182,47,378,0,
			188,50,383,0,
			188,49,409,0,
			183,46,409,0,
			183,46,403,0,
			180,46,399,0,
			177,46,384,0,
			165,46,359,0,
			160,46,340,0,
		};
		const int verticesCount = sizeof(vertices)/(sizeof(int)*4);

		const unsigned int colln = duRGBA(255,255,255,128);
		dd.begin(DU_DRAW_LINES, 1.0f);
		for (int i = 0, j = verticesCount-1; i < verticesCount; j=i++)
		{
			const int* va = &vertices[j*4];
			const int* vb = &vertices[i*4];
			dd.vertex(boundMin[0]+va[0]*cellSize, boundMin[1]+va[1]*cellHeight+j*0.01f, boundMin[2]+va[2]*cellSize, colln);
			dd.vertex(boundMin[0]+vb[0]*cellSize, boundMin[1]+vb[1]*cellHeight+i*0.01f, boundMin[2]+vb[2]*cellSize, colln);
		}
		dd.end();

		const unsigned int colpt = duRGBA(255,255,255,255);
		dd.begin(DU_DRAW_POINTS, 3.0f);
		for (int i = 0, j = verticesCount-1; i < verticesCount; j=i++)
		{
			const int* va = &vertices[j*4];
			dd.vertex(boundMin[0]+va[0]*cellSize, boundMin[1]+va[1]*cellHeight+j*0.01f, boundMin[2]+va[2]*cellSize, colpt);
		}
		dd.end();

		extern int triangulate(int n, const int* vertices, int* indices, int* triangles);

		static int indices[verticesCount];
		static int triangles[verticesCount*3];
		for (int j = 0; j < verticesCount; ++j)
			indices[j] = j;
			
		static int trianglesCount = 0;
		if (!trianglesCount)
		{
			trianglesCount = triangulate(verticesCount, vertices, &indices[0], &triangles[0]);
			if (trianglesCount < 0) trianglesCount = -trianglesCount;
		}
				
		const unsigned int coltri = duRGBA(255,255,255,64);
		dd.begin(DU_DRAW_TRIS);
		for (int i = 0; i < trianglesCount*3; ++i)
		{
			const int* va = &vertices[indices[triangles[i]]*4];
			dd.vertex(boundMin[0]+va[0]*cellSize, boundMin[1]+va[1]*cellHeight, boundMin[2]+va[2]*cellSize, coltri);
		}
		dd.end();
		
	}
	dd.depthMask(true);*/
}

void Sample_Debug::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample_Debug::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
}

const float* Sample_Debug::getBoundsMin()
{
	if (m_cset)
		return m_cset->boundMin;
	if (m_chf)
		return m_chf->boundMin;
	if (m_navMesh)
		return m_bmin;
	return 0;
}

const float* Sample_Debug::getBoundsMax()
{
	if (m_cset)
		return m_cset->boundMax;
	if (m_chf)
		return m_chf->boundMax;
	if (m_navMesh)
		return m_bmax;
	return 0;
}

void Sample_Debug::handleClick(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample_Debug::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

bool Sample_Debug::handleBuild()
{

	if (m_chf)
	{
		rcFreeContourSet(m_cset);
		m_cset = 0;
		
		// Create contours.
		m_cset = rcAllocateContourSet();
		if (!m_cset)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
			return false;
		}
		if (!rcBuildContours(m_ctx, *m_chf, /*m_cfg.maxSimplificationError*/1.3f, /*m_cfg.maxEdgeLen*/12, *m_cset))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
			return false;
		}
	}
		
	return true;
}
