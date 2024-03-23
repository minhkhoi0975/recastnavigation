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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"


static int getCornerHeight(int x, int y, int i, int direction,
	const rcCompactHeightfield& compactHeightfield,
	bool& isBorderVertex)
{
	const rcCompactSpan& span = compactHeightfield.spans[i];
	int cornerHeight = (int)span.y;
	int dirp = (direction + 1) & 0x3;

	unsigned int regions[4] = { 0,0,0,0 };

	// Combine region and areaId codes in order to prevent
	// border vertices which are in between two areaIds to be removed.
	regions[0] = compactHeightfield.spans[i].regionId | (compactHeightfield.areaIds[i] << 16);

	if (rcGetConnection(span, direction) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirectionOffsetX(direction);
		const int ay = y + rcGetDirectionOffsetY(direction);
		const int ai = (int)compactHeightfield.cells[ax + ay * compactHeightfield.width].index + rcGetConnection(span, direction);
		const rcCompactSpan& as = compactHeightfield.spans[ai];
		cornerHeight = rcMax(cornerHeight, (int)as.y);
		regions[1] = compactHeightfield.spans[ai].regionId | (compactHeightfield.areaIds[ai] << 16);
		if (rcGetConnection(as, dirp) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirectionOffsetX(dirp);
			const int ay2 = ay + rcGetDirectionOffsetY(dirp);
			const int ai2 = (int)compactHeightfield.cells[ax2 + ay2 * compactHeightfield.width].index + rcGetConnection(as, dirp);
			const rcCompactSpan& as2 = compactHeightfield.spans[ai2];
			cornerHeight = rcMax(cornerHeight, (int)as2.y);
			regions[2] = compactHeightfield.spans[ai2].regionId | (compactHeightfield.areaIds[ai2] << 16);
		}
	}
	if (rcGetConnection(span, dirp) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirectionOffsetX(dirp);
		const int ay = y + rcGetDirectionOffsetY(dirp);
		const int ai = (int)compactHeightfield.cells[ax + ay * compactHeightfield.width].index + rcGetConnection(span, dirp);
		const rcCompactSpan& as = compactHeightfield.spans[ai];
		cornerHeight = rcMax(cornerHeight, (int)as.y);
		regions[3] = compactHeightfield.spans[ai].regionId | (compactHeightfield.areaIds[ai] << 16);
		if (rcGetConnection(as, direction) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirectionOffsetX(direction);
			const int ay2 = ay + rcGetDirectionOffsetY(direction);
			const int ai2 = (int)compactHeightfield.cells[ax2 + ay2 * compactHeightfield.width].index + rcGetConnection(as, direction);
			const rcCompactSpan& as2 = compactHeightfield.spans[ai2];
			cornerHeight = rcMax(cornerHeight, (int)as2.y);
			regions[2] = compactHeightfield.spans[ai2].regionId | (compactHeightfield.areaIds[ai2] << 16);
		}
	}

	// Check if the vertex is special edge vertex, these vertices will be removed later.
	for (int j = 0; j < 4; ++j)
	{
		const int a = j;
		const int b = (j + 1) & 0x3;
		const int c = (j + 2) & 0x3;
		const int d = (j + 3) & 0x3;

		// The vertex is a border vertex there are two same exterior cells in a row,
		// followed by two interior cells and none of the regions are out of bounds.
		const bool twoSameExts = (regions[a] & regions[b] & RC_BORDER_REGION) != 0 && regions[a] == regions[b];
		const bool twoInts = ((regions[c] | regions[d]) & RC_BORDER_REGION) == 0;
		const bool intsSameArea = (regions[c] >> 16) == (regions[d] >> 16);
		const bool noZeros = regions[a] != 0 && regions[b] != 0 && regions[c] != 0 && regions[d] != 0;
		if (twoSameExts && twoInts && intsSameArea && noZeros)
		{
			isBorderVertex = true;
			break;
		}
	}

	return cornerHeight;
}

static void walkContour(int x, int y, int i,
	const rcCompactHeightfield& compactHeightfield,
	unsigned char* flags, rcIntArray& points)
{
	// Choose the first non-connected edge
	unsigned char direction = 0;
	while ((flags[i] & (1 << direction)) == 0)
		direction++;

	unsigned char startDir = direction;
	int starti = i;

	const unsigned char area = compactHeightfield.areaIds[i];

	int iterations = 0;
	while (++iterations < 40000)
	{
		if (flags[i] & (1 << direction))
		{
			// Choose the edge corner
			bool isBorderVertex = false;
			bool isAreaBorder = false;
			int px = x;
			int py = getCornerHeight(x, y, i, direction, compactHeightfield, isBorderVertex);
			int pz = y;
			switch (direction)
			{
			case 0: pz++; break;
			case 1: px++; pz++; break;
			case 2: px++; break;
			}
			int r = 0;
			const rcCompactSpan& s = compactHeightfield.spans[i];
			if (rcGetConnection(s, direction) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirectionOffsetX(direction);
				const int ay = y + rcGetDirectionOffsetY(direction);
				const int ai = (int)compactHeightfield.cells[ax + ay * compactHeightfield.width].index + rcGetConnection(s, direction);
				r = (int)compactHeightfield.spans[ai].regionId;
				if (area != compactHeightfield.areaIds[ai])
					isAreaBorder = true;
			}
			if (isBorderVertex)
				r |= RC_BORDER_VERTEX;
			if (isAreaBorder)
				r |= RC_AREA_BORDER;
			points.push(px);
			points.push(py);
			points.push(pz);
			points.push(r);

			flags[i] &= ~(1 << direction); // Remove visited edges
			direction = (direction + 1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirectionOffsetX(direction);
			const int ny = y + rcGetDirectionOffsetY(direction);
			const rcCompactSpan& s = compactHeightfield.spans[i];
			if (rcGetConnection(s, direction) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = compactHeightfield.cells[nx + ny * compactHeightfield.width];
				ni = (int)nc.index + rcGetConnection(s, direction);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			direction = (direction + 3) & 0x3;	// Rotate CCW
		}

		if (starti == i && startDir == direction)
		{
			break;
		}
	}
}

static float distancePtSeg(const int x, const int z,
	const int px, const int pz,
	const int qx, const int qz)
{
	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	float d = pqx * pqx + pqz * pqz;
	float t = pqx * dx + pqz * dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;

	dx = px + t * pqx - x;
	dz = pz + t * pqz - z;

	return dx * dx + dz * dz;
}

static void simplifyContour(rcIntArray& points, rcIntArray& simplified,
	const float maxError, const int maxEdgeLength, const int buildFlags)
{
	// Add initial points.
	bool hasConnections = false;
	for (int i = 0; i < points.size(); i += 4)
	{
		if ((points[i + 3] & RC_CONTOUR_REGION_MASK) != 0)
		{
			hasConnections = true;
			break;
		}
	}

	if (hasConnections)
	{
		// The contour has some portals to other regions.
		// Add a new point to every location where the region changes.
		for (int i = 0, ni = points.size() / 4; i < ni; ++i)
		{
			int ii = (i + 1) % ni;
			const bool differentRegs = (points[i * 4 + 3] & RC_CONTOUR_REGION_MASK) != (points[ii * 4 + 3] & RC_CONTOUR_REGION_MASK);
			const bool areaBorders = (points[i * 4 + 3] & RC_AREA_BORDER) != (points[ii * 4 + 3] & RC_AREA_BORDER);
			if (differentRegs || areaBorders)
			{
				simplified.push(points[i * 4 + 0]);
				simplified.push(points[i * 4 + 1]);
				simplified.push(points[i * 4 + 2]);
				simplified.push(i);
			}
		}
	}

	if (simplified.size() == 0)
	{
		// If there is no connections at all,
		// create some initial points for the simplification process.
		// Find lower-left and upper-right vertices of the contour.
		int lowerLeftX = points[0];
		int lowerLeftY = points[1];
		int lowerLeftZ = points[2];
		int lowerLeftI = 0;
		int upperRightX = points[0];
		int upperRightY = points[1];
		int upperRightZ = points[2];
		int upperRightI = 0;
		for (int i = 0; i < points.size(); i += 4)
		{
			int x = points[i + 0];
			int y = points[i + 1];
			int z = points[i + 2];
			if (x < lowerLeftX || (x == lowerLeftX && z < lowerLeftZ))
			{
				lowerLeftX = x;
				lowerLeftY = y;
				lowerLeftZ = z;
				lowerLeftI = i / 4;
			}
			if (x > upperRightX || (x == upperRightX && z > upperRightZ))
			{
				upperRightX = x;
				upperRightY = y;
				upperRightZ = z;
				upperRightI = i / 4;
			}
		}
		simplified.push(lowerLeftX);
		simplified.push(lowerLeftY);
		simplified.push(lowerLeftZ);
		simplified.push(lowerLeftI);

		simplified.push(upperRightX);
		simplified.push(upperRightY);
		simplified.push(upperRightZ);
		simplified.push(upperRightI);
	}

	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	const int pn = points.size() / 4;
	for (int i = 0; i < simplified.size() / 4; )
	{
		int ii = (i + 1) % (simplified.size() / 4);

		int ax = simplified[i * 4 + 0];
		int az = simplified[i * 4 + 2];
		int ai = simplified[i * 4 + 3];

		int bx = simplified[ii * 4 + 0];
		int bz = simplified[ii * 4 + 2];
		int bi = simplified[ii * 4 + 3];

		// Find maximum deviation from the segment.
		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;

		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		if (bx > ax || (bx == ax && bz > az))
		{
			cinc = 1;
			ci = (ai + cinc) % pn;
			endi = bi;
		}
		else
		{
			cinc = pn - 1;
			ci = (bi + cinc) % pn;
			endi = ai;
			rcSwap(ax, bx);
			rcSwap(az, bz);
		}

		// Tessellate only outer edges or edges between areaIds.
		if ((points[ci * 4 + 3] & RC_CONTOUR_REGION_MASK) == 0 ||
			(points[ci * 4 + 3] & RC_AREA_BORDER))
		{
			while (ci != endi)
			{
				float d = distancePtSeg(points[ci * 4 + 0], points[ci * 4 + 2], ax, az, bx, bz);
				if (d > maxd)
				{
					maxd = d;
					maxi = ci;
				}
				ci = (ci + cinc) % pn;
			}
		}


		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if (maxi != -1 && maxd > (maxError * maxError))
		{
			// Add space for the new point.
			simplified.resize(simplified.size() + 4);
			const int n = simplified.size() / 4;
			for (int j = n - 1; j > i; --j)
			{
				simplified[j * 4 + 0] = simplified[(j - 1) * 4 + 0];
				simplified[j * 4 + 1] = simplified[(j - 1) * 4 + 1];
				simplified[j * 4 + 2] = simplified[(j - 1) * 4 + 2];
				simplified[j * 4 + 3] = simplified[(j - 1) * 4 + 3];
			}
			// Add the point.
			simplified[(i + 1) * 4 + 0] = points[maxi * 4 + 0];
			simplified[(i + 1) * 4 + 1] = points[maxi * 4 + 1];
			simplified[(i + 1) * 4 + 2] = points[maxi * 4 + 2];
			simplified[(i + 1) * 4 + 3] = maxi;
		}
		else
		{
			++i;
		}
	}

	// Split too long edges.
	if (maxEdgeLength > 0 && (buildFlags & (RC_CONTOUR_TESS_WALL_EDGES | RC_CONTOUR_TESS_AREA_EDGES)) != 0)
	{
		for (int i = 0; i < simplified.size() / 4; )
		{
			const int ii = (i + 1) % (simplified.size() / 4);

			const int ax = simplified[i * 4 + 0];
			const int az = simplified[i * 4 + 2];
			const int ai = simplified[i * 4 + 3];

			const int bx = simplified[ii * 4 + 0];
			const int bz = simplified[ii * 4 + 2];
			const int bi = simplified[ii * 4 + 3];

			// Find maximum deviation from the segment.
			int maxi = -1;
			int ci = (ai + 1) % pn;

			// Tessellate only outer edges or edges between areaIds.
			bool tess = false;
			// Wall edges.
			if ((buildFlags & RC_CONTOUR_TESS_WALL_EDGES) && (points[ci * 4 + 3] & RC_CONTOUR_REGION_MASK) == 0)
				tess = true;
			// Edges between areaIds.
			if ((buildFlags & RC_CONTOUR_TESS_AREA_EDGES) && (points[ci * 4 + 3] & RC_AREA_BORDER))
				tess = true;

			if (tess)
			{
				int dx = bx - ax;
				int dz = bz - az;
				if (dx * dx + dz * dz > maxEdgeLength * maxEdgeLength)
				{
					// Round based on the segments in lexilogical order so that the
					// max tesselation is consistent regardless in which direction
					// segments are traversed.
					const int n = bi < ai ? (bi + pn - ai) : (bi - ai);
					if (n > 1)
					{
						if (bx > ax || (bx == ax && bz > az))
							maxi = (ai + n / 2) % pn;
						else
							maxi = (ai + (n + 1) / 2) % pn;
					}
				}
			}

			// If the max deviation is larger than accepted error,
			// add new point, else continue to next segment.
			if (maxi != -1)
			{
				// Add space for the new point.
				simplified.resize(simplified.size() + 4);
				const int n = simplified.size() / 4;
				for (int j = n - 1; j > i; --j)
				{
					simplified[j * 4 + 0] = simplified[(j - 1) * 4 + 0];
					simplified[j * 4 + 1] = simplified[(j - 1) * 4 + 1];
					simplified[j * 4 + 2] = simplified[(j - 1) * 4 + 2];
					simplified[j * 4 + 3] = simplified[(j - 1) * 4 + 3];
				}
				// Add the point.
				simplified[(i + 1) * 4 + 0] = points[maxi * 4 + 0];
				simplified[(i + 1) * 4 + 1] = points[maxi * 4 + 1];
				simplified[(i + 1) * 4 + 2] = points[maxi * 4 + 2];
				simplified[(i + 1) * 4 + 3] = maxi;
			}
			else
			{
				++i;
			}
		}
	}

	for (int i = 0; i < simplified.size() / 4; ++i)
	{
		// The edge vertex flag is take from the current raw point,
		// and the neighbour region is take from the next raw point.
		const int ai = (simplified[i * 4 + 3] + 1) % pn;
		const int bi = simplified[i * 4 + 3];
		simplified[i * 4 + 3] = (points[ai * 4 + 3] & (RC_CONTOUR_REGION_MASK | RC_AREA_BORDER)) | (points[bi * 4 + 3] & RC_BORDER_VERTEX);
	}

}

static int calculateAreaOfPolygonXZPlane(const int* vertices, const int verticesCount)
{
	int area = 0;
	for (int i = 0, j = verticesCount - 1; i < verticesCount; j = i++)
	{
		const int* vi = &vertices[i * 4];
		const int* vj = &vertices[j * 4];
		area += vi[0] * vj[2] - vj[0] * vi[2];
	}
	return (area + 1) / 2;
}

// TODO: these are the same as in RecastMesh.cpp, consider using the same.
// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
inline int next(int i, int n) { return i + 1 < n ? i + 1 : 0; }

inline int area2(const int* a, const int* b, const int* c)
{
	return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
}

//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
inline bool xorb(bool x, bool y)
{
	return !x ^ !y;
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
inline bool left(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) < 0;
}

inline bool leftOn(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
static bool intersectProp(const int* a, const int* b, const int* c, const int* d)
{
	// Eliminate improper cases.
	if (collinear(a, b, c) || collinear(a, b, d) ||
		collinear(c, d, a) || collinear(c, d, b))
		return false;

	return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b));
}

// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segment ab.
static bool between(const int* a, const int* b, const int* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	if (a[0] != b[0])
		return	((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return	((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true if segments ab and cd intersect, properly or improperly.
static bool intersect(const int* a, const int* b, const int* c, const int* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
		between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}

// Checks if two vectors are equal on XZ-plane. 
static bool areEqualOnXZPlane(const int* vector1, const int* vector2)
{
	return vector1[0] == vector2[0] && vector1[2] == vector2[2];
}

static bool intersectSegContour(const int* d0, const int* d1, int i, int n, const int* verts)
{
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i.
		if (i == k || i == k1)
			continue;
		const int* p0 = &verts[k * 4];
		const int* p1 = &verts[k1 * 4];
		if (areEqualOnXZPlane(d0, p0) || areEqualOnXZPlane(d1, p0) || areEqualOnXZPlane(d0, p1) || areEqualOnXZPlane(d1, p1))
			continue;

		if (intersect(d0, d1, p0, p1))
			return true;
	}
	return false;
}

static bool	inCone(int i, int n, const int* verts, const int* pj)
{
	const int* pi = &verts[i * 4];
	const int* pi1 = &verts[next(i, n) * 4];
	const int* pin1 = &verts[prev(i, n) * 4];

	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

static void removeDegenerateSegments(rcIntArray& simplified)
{
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	int pointsCount = simplified.size() / 4;
	for (int i = 0; i < pointsCount; ++i)
	{
		int ni = next(i, pointsCount);

		if (areEqualOnXZPlane(&simplified[i * 4], &simplified[ni * 4]))
		{
			// Degenerate segment, remove.
			for (int j = i; j < simplified.size() / 4 - 1; ++j)
			{
				simplified[j * 4 + 0] = simplified[(j + 1) * 4 + 0];
				simplified[j * 4 + 1] = simplified[(j + 1) * 4 + 1];
				simplified[j * 4 + 2] = simplified[(j + 1) * 4 + 2];
				simplified[j * 4 + 3] = simplified[(j + 1) * 4 + 3];
			}
			simplified.resize(simplified.size() - 4);
			pointsCount--;
		}
	}
}

static bool mergeContours(rcContour& contourA, rcContour& contourB, int ia, int ib)
{
	const int maxVerticesCount = contourA.verticesCount + contourB.verticesCount + 2;
	int* vertices = (int*)rcAllocate(sizeof(int) * maxVerticesCount * 4, RC_ALLOC_PERMANENT);
	if (!vertices)
		return false;

	int verticesCount = 0;

	// Copy contour A.
	for (int i = 0; i <= contourA.verticesCount; ++i)
	{
		int* destination = &vertices[verticesCount * 4];
		const int* source = &contourA.vertices[((ia + i) % contourA.verticesCount) * 4];
		destination[0] = source[0];
		destination[1] = source[1];
		destination[2] = source[2];
		destination[3] = source[3];
		verticesCount++;
	}

	// Copy contour B.
	for (int i = 0; i <= contourB.verticesCount; ++i)
	{
		int* destination = &vertices[verticesCount * 4];
		const int* source = &contourB.vertices[((ib + i) % contourB.verticesCount) * 4];
		destination[0] = source[0];
		destination[1] = source[1];
		destination[2] = source[2];
		destination[3] = source[3];
		verticesCount++;
	}

	rcFree(contourA.vertices);
	contourA.vertices = vertices;
	contourA.verticesCount = verticesCount;

	rcFree(contourB.vertices);
	contourB.vertices = 0;
	contourB.verticesCount = 0;

	return true;
}

struct rcContourHole
{
	rcContour* contour;
	int minx, minz, leftmost;
};

struct rcContourRegion
{
	rcContour* outline;
	rcContourHole* holes;
	int nholes;
};

struct rcPotentialDiagonal
{
	int vert;
	int dist;
};

// Finds the lowest leftmost vertex of a contour.
static void findLeftMostVertex(rcContour* contour, int* minx, int* minz, int* leftmost)
{
	*minx = contour->vertices[0];
	*minz = contour->vertices[2];
	*leftmost = 0;
	for (int i = 1; i < contour->verticesCount; i++)
	{
		const int x = contour->vertices[i * 4 + 0];
		const int z = contour->vertices[i * 4 + 2];
		if (x < *minx || (x == *minx && z < *minz))
		{
			*minx = x;
			*minz = z;
			*leftmost = i;
		}
	}
}

static int compareHoles(const void* va, const void* vb)
{
	const rcContourHole* a = (const rcContourHole*)va;
	const rcContourHole* b = (const rcContourHole*)vb;
	if (a->minx == b->minx)
	{
		if (a->minz < b->minz)
			return -1;
		if (a->minz > b->minz)
			return 1;
	}
	else
	{
		if (a->minx < b->minx)
			return -1;
		if (a->minx > b->minx)
			return 1;
	}
	return 0;
}

static int compareDiagDist(const void* va, const void* vb)
{
	const rcPotentialDiagonal* a = (const rcPotentialDiagonal*)va;
	const rcPotentialDiagonal* b = (const rcPotentialDiagonal*)vb;
	if (a->dist < b->dist)
		return -1;
	if (a->dist > b->dist)
		return 1;
	return 0;
}

static void mergeRegionHoles(rcContext* context, rcContourRegion& region)
{
	// Sort holes from left to right.
	for (int i = 0; i < region.nholes; i++)
		findLeftMostVertex(region.holes[i].contour, &region.holes[i].minx, &region.holes[i].minz, &region.holes[i].leftmost);

	qsort(region.holes, region.nholes, sizeof(rcContourHole), compareHoles);

	int maxVerts = region.outline->verticesCount;
	for (int i = 0; i < region.nholes; i++)
		maxVerts += region.holes[i].contour->verticesCount;

	rcScopedDelete<rcPotentialDiagonal> diags((rcPotentialDiagonal*)rcAllocate(sizeof(rcPotentialDiagonal) * maxVerts, RC_ALLOC_TEMPORARY));
	if (!diags)
	{
		context->log(RC_LOG_WARNING, "mergeRegionHoles: Failed to allocated diags %d.", maxVerts);
		return;
	}

	rcContour* outline = region.outline;

	// Merge holes into the outline one by one.
	for (int i = 0; i < region.nholes; i++)
	{
		rcContour* hole = region.holes[i].contour;

		int index = -1;
		int bestVertex = region.holes[i].leftmost;
		for (int iter = 0; iter < hole->verticesCount; iter++)
		{
			// Find potential diagonals.
			// The 'best' vertex must be in the cone described by 3 consecutive vertices of the outline.
			// ..o j-1
			//   |
			//   |   * best
			//   |
			// j o-----o j+1
			//         :
			int ndiags = 0;
			const int* corner = &hole->vertices[bestVertex * 4];
			for (int j = 0; j < outline->verticesCount; j++)
			{
				if (inCone(j, outline->verticesCount, outline->vertices, corner))
				{
					int dx = outline->vertices[j * 4 + 0] - corner[0];
					int dz = outline->vertices[j * 4 + 2] - corner[2];
					diags[ndiags].vert = j;
					diags[ndiags].dist = dx * dx + dz * dz;
					ndiags++;
				}
			}
			// Sort potential diagonals by distance, we want to make the connection as short as possible.
			qsort(diags, ndiags, sizeof(rcPotentialDiagonal), compareDiagDist);

			// Find a diagonal that is not intersecting the outline not the remaining holes.
			index = -1;
			for (int j = 0; j < ndiags; j++)
			{
				const int* pt = &outline->vertices[diags[j].vert * 4];
				bool intersect = intersectSegContour(pt, corner, diags[i].vert, outline->verticesCount, outline->vertices);
				for (int k = i; k < region.nholes && !intersect; k++)
					intersect |= intersectSegContour(pt, corner, -1, region.holes[k].contour->verticesCount, region.holes[k].contour->vertices);
				if (!intersect)
				{
					index = diags[j].vert;
					break;
				}
			}
			// If found non-intersecting diagonal, stop looking.
			if (index != -1)
				break;
			// All the potential diagonals for the current vertex were intersecting, try next vertex.
			bestVertex = (bestVertex + 1) % hole->verticesCount;
		}

		if (index == -1)
		{
			context->log(RC_LOG_WARNING, "mergeHoles: Failed to find merge points for %p and %p.", region.outline, hole);
			continue;
		}
		if (!mergeContours(*region.outline, *hole, index, bestVertex))
		{
			context->log(RC_LOG_WARNING, "mergeHoles: Failed to merge contours %p and %p.", region.outline, hole);
			continue;
		}
	}
}


/// @par
///
/// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLength
/// parameters control how closely the simplified contours will match the raw contours.
///
/// Simplified contours are generated such that the vertices for portals between areaIds match up.
/// (They are considered mandatory vertices.)
///
/// Setting @p maxEdgeLength to zero will disabled the edge length feature.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocateContourSet, rcCompactHeightfield, rcContourSet, rcConfig
bool rcBuildContours(rcContext* context, const rcCompactHeightfield& compactHeightfield,
	const float maxError, const int maxEdgeLength,
	rcContourSet& contourSet, const int buildFlags)
{
	rcAssert(context);

	const int w = compactHeightfield.width;
	const int h = compactHeightfield.height;
	const int borderSize = compactHeightfield.borderSize;

	rcScopedTimer timer(context, RC_TIMER_BUILD_CONTOURS);

	rcCopyVector(contourSet.boundMin, compactHeightfield.boundMin);
	rcCopyVector(contourSet.boundMax, compactHeightfield.boundMax);
	if (borderSize > 0)
	{
		// If the heightfield was build with bordersize, remove the offset.
		const float pad = borderSize * compactHeightfield.cellSize;
		contourSet.boundMin[0] += pad;
		contourSet.boundMin[2] += pad;
		contourSet.boundMax[0] -= pad;
		contourSet.boundMax[2] -= pad;
	}
	contourSet.cellSize = compactHeightfield.cellSize;
	contourSet.cellHeight = compactHeightfield.cellHeight;
	contourSet.width = compactHeightfield.width - compactHeightfield.borderSize * 2;
	contourSet.height = compactHeightfield.height - compactHeightfield.borderSize * 2;
	contourSet.borderSize = compactHeightfield.borderSize;
	contourSet.maxError = maxError;

	int maxContours = rcMax((int)compactHeightfield.maxRegions, 8);
	contourSet.contours = (rcContour*)rcAllocate(sizeof(rcContour) * maxContours, RC_ALLOC_PERMANENT);
	if (!contourSet.contours)
		return false;
	contourSet.contoursCount = 0;

	rcScopedDelete<unsigned char> flags((unsigned char*)rcAllocate(sizeof(unsigned char) * compactHeightfield.spanCount, RC_ALLOC_TEMPORARY));
	if (!flags)
	{
		context->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", compactHeightfield.spanCount);
		return false;
	}

	context->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

	// Mark boundaries.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				unsigned char res = 0;
				const rcCompactSpan& s = compactHeightfield.spans[i];
				if (!compactHeightfield.spans[i].regionId || (compactHeightfield.spans[i].regionId & RC_BORDER_REGION))
				{
					flags[i] = 0;
					continue;
				}
				for (int dir = 0; dir < 4; ++dir)
				{
					unsigned short r = 0;
					if (rcGetConnection(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirectionOffsetX(dir);
						const int ay = y + rcGetDirectionOffsetY(dir);
						const int ai = (int)compactHeightfield.cells[ax + ay * w].index + rcGetConnection(s, dir);
						r = compactHeightfield.spans[ai].regionId;
					}
					if (r == compactHeightfield.spans[i].regionId)
						res |= (1 << dir);
				}
				flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
			}
		}
	}

	context->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

	rcIntArray verts(256);
	rcIntArray simplified(64);

	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}
				const unsigned short reg = compactHeightfield.spans[i].regionId;
				if (!reg || (reg & RC_BORDER_REGION))
					continue;
				const unsigned char area = compactHeightfield.areaIds[i];

				verts.clear();
				simplified.clear();

				context->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
				walkContour(x, y, i, compactHeightfield, flags, verts);
				context->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

				context->startTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
				simplifyContour(verts, simplified, maxError, maxEdgeLength, buildFlags);
				removeDegenerateSegments(simplified);
				context->stopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);


				// Store region->contour remap info.
				// Create contour.
				if (simplified.size() / 4 >= 3)
				{
					if (contourSet.contoursCount >= maxContours)
					{
						// Allocate more contours.
						// This happens when a region has holes.
						const int oldMax = maxContours;
						maxContours *= 2;
						rcContour* newConts = (rcContour*)rcAllocate(sizeof(rcContour) * maxContours, RC_ALLOC_PERMANENT);
						for (int j = 0; j < contourSet.contoursCount; ++j)
						{
							newConts[j] = contourSet.contours[j];
							// Reset source pointers to prevent data deletion.
							contourSet.contours[j].vertices = 0;
							contourSet.contours[j].rawVertices = 0;
						}
						rcFree(contourSet.contours);
						contourSet.contours = newConts;

						context->log(RC_LOG_WARNING, "rcBuildContours: Expanding max contours from %d to %d.", oldMax, maxContours);
					}

					rcContour* cont = &contourSet.contours[contourSet.contoursCount++];

					cont->verticesCount = simplified.size() / 4;
					cont->vertices = (int*)rcAllocate(sizeof(int) * cont->verticesCount * 4, RC_ALLOC_PERMANENT);
					if (!cont->vertices)
					{
						context->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'verts' (%d).", cont->verticesCount);
						return false;
					}
					memcpy(cont->vertices, &simplified[0], sizeof(int) * cont->verticesCount * 4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->verticesCount; ++j)
						{
							int* v = &cont->vertices[j * 4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}

					cont->rawVerticesCount = verts.size() / 4;
					cont->rawVertices = (int*)rcAllocate(sizeof(int) * cont->rawVerticesCount * 4, RC_ALLOC_PERMANENT);
					if (!cont->rawVertices)
					{
						context->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'rverts' (%d).", cont->rawVerticesCount);
						return false;
					}
					memcpy(cont->rawVertices, &verts[0], sizeof(int) * cont->rawVerticesCount * 4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->rawVerticesCount; ++j)
						{
							int* v = &cont->rawVertices[j * 4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}

					cont->regionId = reg;
					cont->areaId = area;
				}
			}
		}
	}

	// Merge holes if needed.
	if (contourSet.contoursCount > 0)
	{
		// Calculate winding of all polygons.
		rcScopedDelete<signed char> winding((signed char*)rcAllocate(sizeof(signed char) * contourSet.contoursCount, RC_ALLOC_TEMPORARY));
		if (!winding)
		{
			context->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'hole' (%d).", contourSet.contoursCount);
			return false;
		}
		int nholes = 0;
		for (int i = 0; i < contourSet.contoursCount; ++i)
		{
			rcContour& cont = contourSet.contours[i];
			// If the contour is wound backwards, it is a hole.
			winding[i] = calculateAreaOfPolygonXZPlane(cont.vertices, cont.verticesCount) < 0 ? -1 : 1;
			if (winding[i] < 0)
				nholes++;
		}

		if (nholes > 0)
		{
			// Collect outline contour and holes contours per region.
			// We assume that there is one outline and multiple holes.
			const int nregions = compactHeightfield.maxRegions + 1;
			rcScopedDelete<rcContourRegion> regions((rcContourRegion*)rcAllocate(sizeof(rcContourRegion) * nregions, RC_ALLOC_TEMPORARY));
			if (!regions)
			{
				context->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'regions' (%d).", nregions);
				return false;
			}
			memset(regions, 0, sizeof(rcContourRegion) * nregions);

			rcScopedDelete<rcContourHole> holes((rcContourHole*)rcAllocate(sizeof(rcContourHole) * contourSet.contoursCount, RC_ALLOC_TEMPORARY));
			if (!holes)
			{
				context->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'holes' (%d).", contourSet.contoursCount);
				return false;
			}
			memset(holes, 0, sizeof(rcContourHole) * contourSet.contoursCount);

			for (int i = 0; i < contourSet.contoursCount; ++i)
			{
				rcContour& cont = contourSet.contours[i];
				// Positively would contours are outlines, negative holes.
				if (winding[i] > 0)
				{
					if (regions[cont.regionId].outline)
						context->log(RC_LOG_ERROR, "rcBuildContours: Multiple outlines for region %d.", cont.regionId);
					regions[cont.regionId].outline = &cont;
				}
				else
				{
					regions[cont.regionId].nholes++;
				}
			}
			int index = 0;
			for (int i = 0; i < nregions; i++)
			{
				if (regions[i].nholes > 0)
				{
					regions[i].holes = &holes[index];
					index += regions[i].nholes;
					regions[i].nholes = 0;
				}
			}
			for (int i = 0; i < contourSet.contoursCount; ++i)
			{
				rcContour& cont = contourSet.contours[i];
				rcContourRegion& reg = regions[cont.regionId];
				if (winding[i] < 0)
					reg.holes[reg.nholes++].contour = &cont;
			}

			// Finally merge each regions holes into the outline.
			for (int i = 0; i < nregions; i++)
			{
				rcContourRegion& reg = regions[i];
				if (!reg.nholes) continue;

				if (reg.outline)
				{
					mergeRegionHoles(context, reg);
				}
				else
				{
					// The region does not have an outline.
					// This can happen if the contour becaomes selfoverlapping because of
					// too aggressive simplification settings.
					context->log(RC_LOG_ERROR, "rcBuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i);
				}
			}
		}

	}

	return true;
}
