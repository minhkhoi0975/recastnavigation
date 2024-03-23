//
// Copyright (cell) 2009-2010 Mikko Mononen memon@inside.org
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

#include <float.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

namespace
{
	struct LevelStackEntry
	{
		LevelStackEntry(int x_, int y_, int index_) : x(x_), y(y_), index(index_) {}
		int x;
		int y;
		int index;
	};
}  // namespace

static void calculateDistanceField(rcCompactHeightfield& compactHeightfield, unsigned short* source, unsigned short& maxDistance)
{
	const int width = compactHeightfield.width;
	const int height = compactHeightfield.height;

	// Init distance and points.
	for (int i = 0; i < compactHeightfield.spanCount; ++i)
		source[i] = 0xffff;

	// Mark boundary cells.
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + y * width];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				const rcCompactSpan& span = compactHeightfield.spans[i];
				const unsigned char area = compactHeightfield.areaIds[i];

				int neighborCount = 0;
				for (int direction = 0; direction < 4; ++direction)
				{
					if (rcGetConnection(span, direction) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirectionOffsetX(direction);
						const int ay = y + rcGetDirectionOffsetY(direction);
						const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(span, direction);
						if (area == compactHeightfield.areaIds[ai])
							neighborCount++;
					}
				}
				if (neighborCount != 4)
					source[i] = 0;
			}
		}
	}

	// Pass 1
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + y * width];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				const rcCompactSpan& span = compactHeightfield.spans[i];

				if (rcGetConnection(span, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int ax = x + rcGetDirectionOffsetX(0);
					const int ay = y + rcGetDirectionOffsetY(0);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(span, 0);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					if (source[ai] + 2 < source[i])
						source[i] = source[ai] + 2;

					// (-1,-1)
					if (rcGetConnection(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirectionOffsetX(3);
						const int aay = ay + rcGetDirectionOffsetY(3);
						const int aai = (int)compactHeightfield.cells[aax + aay * width].index + rcGetConnection(as, 3);
						if (source[aai] + 3 < source[i])
							source[i] = source[aai] + 3;
					}
				}
				if (rcGetConnection(span, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int ax = x + rcGetDirectionOffsetX(3);
					const int ay = y + rcGetDirectionOffsetY(3);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(span, 3);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					if (source[ai] + 2 < source[i])
						source[i] = source[ai] + 2;

					// (1,-1)
					if (rcGetConnection(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirectionOffsetX(2);
						const int aay = ay + rcGetDirectionOffsetY(2);
						const int aai = (int)compactHeightfield.cells[aax + aay * width].index + rcGetConnection(as, 2);
						if (source[aai] + 3 < source[i])
							source[i] = source[aai] + 3;
					}
				}
			}
		}
	}

	// Pass 2
	for (int y = height - 1; y >= 0; --y)
	{
		for (int x = width - 1; x >= 0; --x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + y * width];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				const rcCompactSpan& span = compactHeightfield.spans[i];

				if (rcGetConnection(span, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int ax = x + rcGetDirectionOffsetX(2);
					const int ay = y + rcGetDirectionOffsetY(2);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(span, 2);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					if (source[ai] + 2 < source[i])
						source[i] = source[ai] + 2;

					// (1,1)
					if (rcGetConnection(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirectionOffsetX(1);
						const int aay = ay + rcGetDirectionOffsetY(1);
						const int aai = (int)compactHeightfield.cells[aax + aay * width].index + rcGetConnection(as, 1);
						if (source[aai] + 3 < source[i])
							source[i] = source[aai] + 3;
					}
				}
				if (rcGetConnection(span, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int ax = x + rcGetDirectionOffsetX(1);
					const int ay = y + rcGetDirectionOffsetY(1);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(span, 1);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					if (source[ai] + 2 < source[i])
						source[i] = source[ai] + 2;

					// (-1,1)
					if (rcGetConnection(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirectionOffsetX(0);
						const int aay = ay + rcGetDirectionOffsetY(0);
						const int aai = (int)compactHeightfield.cells[aax + aay * width].index + rcGetConnection(as, 0);
						if (source[aai] + 3 < source[i])
							source[i] = source[aai] + 3;
					}
				}
			}
		}
	}

	maxDistance = 0;
	for (int i = 0; i < compactHeightfield.spanCount; ++i)
		maxDistance = rcMax(source[i], maxDistance);
}

static unsigned short* boxBlur(rcCompactHeightfield& chf, int threshold,
	unsigned short* source, unsigned short* destination)
{
	const int width = chf.width;
	const int height = chf.height;

	threshold *= 2;

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * width];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned short cd = source[i];
				if (cd <= threshold)
				{
					destination[i] = cd;
					continue;
				}

				int d = (int)cd;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetConnection(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirectionOffsetX(dir);
						const int ay = y + rcGetDirectionOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * width].index + rcGetConnection(s, dir);
						d += (int)source[ai];

						const rcCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir + 1) & 0x3;
						if (rcGetConnection(as, dir2) != RC_NOT_CONNECTED)
						{
							const int ax2 = ax + rcGetDirectionOffsetX(dir2);
							const int ay2 = ay + rcGetDirectionOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2 + ay2 * width].index + rcGetConnection(as, dir2);
							d += (int)source[ai2];
						}
						else
						{
							d += cd;
						}
					}
					else
					{
						d += cd * 2;
					}
				}
				destination[i] = (unsigned short)((d + 5) / 9);
			}
		}
	}
	return destination;
}


static bool floodRegion(int x, int y, int i,
	unsigned short level, unsigned short r,
	rcCompactHeightfield& compactHeightfield,
	unsigned short* sourceRegions, unsigned short* sourceDistances,
	rcTempVector<LevelStackEntry>& stack)
{
	const int width = compactHeightfield.width;

	const unsigned char areaId = compactHeightfield.areaIds[i];

	// Flood fill mark region.
	stack.clear();
	stack.push_back(LevelStackEntry(x, y, i));
	sourceRegions[i] = r;
	sourceDistances[i] = 0;

	unsigned short lev = level >= 2 ? level - 2 : 0;
	int count = 0;

	while (stack.size() > 0)
	{
		LevelStackEntry& back = stack.back();
		int cx = back.x;
		int cy = back.y;
		int ci = back.index;
		stack.pop_back();

		const rcCompactSpan& compactSpan = compactHeightfield.spans[ci];

		// Check if any of the neighbours already have a valid region set.
		unsigned short ar = 0;
		for (int direction = 0; direction < 4; ++direction)
		{
			// 8 connected
			if (rcGetConnection(compactSpan, direction) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirectionOffsetX(direction);
				const int ay = cy + rcGetDirectionOffsetY(direction);
				const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(compactSpan, direction);
				if (compactHeightfield.areaIds[ai] != areaId)
					continue;
				unsigned short nr = sourceRegions[ai];
				if (nr & RC_BORDER_REGION) // Do not take borders into account.
					continue;
				if (nr != 0 && nr != r)
				{
					ar = nr;
					break;
				}

				const rcCompactSpan& as = compactHeightfield.spans[ai];

				const int dir2 = (direction + 1) & 0x3;
				if (rcGetConnection(as, dir2) != RC_NOT_CONNECTED)
				{
					const int ax2 = ax + rcGetDirectionOffsetX(dir2);
					const int ay2 = ay + rcGetDirectionOffsetY(dir2);
					const int ai2 = (int)compactHeightfield.cells[ax2 + ay2 * width].index + rcGetConnection(as, dir2);
					if (compactHeightfield.areaIds[ai2] != areaId)
						continue;
					unsigned short nr2 = sourceRegions[ai2];
					if (nr2 != 0 && nr2 != r)
					{
						ar = nr2;
						break;
					}
				}
			}
		}
		if (ar != 0)
		{
			sourceRegions[ci] = 0;
			continue;
		}

		count++;

		// Expand neighbours.
		for (int direction = 0; direction < 4; ++direction)
		{
			if (rcGetConnection(compactSpan, direction) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirectionOffsetX(direction);
				const int ay = cy + rcGetDirectionOffsetY(direction);
				const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(compactSpan, direction);
				if (compactHeightfield.areaIds[ai] != areaId)
					continue;
				if (compactHeightfield.distancesToBorder[ai] >= lev && sourceRegions[ai] == 0)
				{
					sourceRegions[ai] = r;
					sourceDistances[ai] = 0;
					stack.push_back(LevelStackEntry(ax, ay, ai));
				}
			}
		}
	}

	return count > 0;
}

// Struct to keep track of entries in the region table that have been changed.
struct DirtyEntry
{
	DirtyEntry(int index_, unsigned short region_, unsigned short distance2_)
		: index(index_), region(region_), distance2(distance2_) {}
	int index;
	unsigned short region;
	unsigned short distance2;
};

static void expandRegions(int maxIterations, unsigned short level,
	rcCompactHeightfield& compactHeightfield,
	unsigned short* sourceRegions, unsigned short* sourceDistances,
	rcTempVector<LevelStackEntry>& stack,
	bool fillStack)
{
	const int width = compactHeightfield.width;
	const int height = compactHeightfield.height;

	if (fillStack)
	{
		// Find cells revealed by the raised level.
		stack.clear();
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				const rcCompactCell& compactCell = compactHeightfield.cells[x + y * width];
				for (int i = (int)compactCell.index, ni = (int)(compactCell.index + compactCell.count); i < ni; ++i)
				{
					if (compactHeightfield.distancesToBorder[i] >= level && sourceRegions[i] == 0 && compactHeightfield.areaIds[i] != RC_NULL_AREA)
					{
						stack.push_back(LevelStackEntry(x, y, i));
					}
				}
			}
		}
	}
	else // use cells in the input stack
	{
		// mark all cells which already have a region
		for (int j = 0; j < stack.size(); j++)
		{
			int i = stack[j].index;
			if (sourceRegions[i] != 0)
				stack[j].index = -1;
		}
	}

	rcTempVector<DirtyEntry> dirtyEntries;
	int iterations = 0;
	while (stack.size() > 0)
	{
		int failed = 0;
		dirtyEntries.clear();

		for (int j = 0; j < stack.size(); j++)
		{
			int x = stack[j].x;
			int y = stack[j].y;
			int i = stack[j].index;
			if (i < 0)
			{
				failed++;
				continue;
			}

			unsigned short r = sourceRegions[i];
			unsigned short d2 = 0xffff;
			const unsigned char area = compactHeightfield.areaIds[i];
			const rcCompactSpan& s = compactHeightfield.spans[i];
			for (int dir = 0; dir < 4; ++dir)
			{
				if (rcGetConnection(s, dir) == RC_NOT_CONNECTED) continue;
				const int ax = x + rcGetDirectionOffsetX(dir);
				const int ay = y + rcGetDirectionOffsetY(dir);
				const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(s, dir);
				if (compactHeightfield.areaIds[ai] != area) continue;
				if (sourceRegions[ai] > 0 && (sourceRegions[ai] & RC_BORDER_REGION) == 0)
				{
					if ((int)sourceDistances[ai] + 2 < (int)d2)
					{
						r = sourceRegions[ai];
						d2 = sourceDistances[ai] + 2;
					}
				}
			}
			if (r)
			{
				stack[j].index = -1; // mark as used
				dirtyEntries.push_back(DirtyEntry(i, r, d2));
			}
			else
			{
				failed++;
			}
		}

		// Copy entries that differ between source and destination to keep them in sync.
		for (int i = 0; i < dirtyEntries.size(); i++) {
			int idx = dirtyEntries[i].index;
			sourceRegions[idx] = dirtyEntries[i].region;
			sourceDistances[idx] = dirtyEntries[i].distance2;
		}

		if (failed == stack.size())
			break;

		if (level > 0)
		{
			++iterations;
			if (iterations >= maxIterations)
				break;
		}
	}
}

static void sortCellsByLevel(unsigned short startLevel,
	rcCompactHeightfield& compactHeightfield,
	const unsigned short* sourceRegions,
	unsigned int nbStacks, rcTempVector<LevelStackEntry>* stacks,
	unsigned short loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift
{
	const int w = compactHeightfield.width;
	const int h = compactHeightfield.height;
	startLevel = startLevel >> loglevelsPerStack;

	for (unsigned int j = 0; j < nbStacks; ++j)
		stacks[j].clear();

	// put all cells in the level range into the appropriate stacks
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (compactHeightfield.areaIds[i] == RC_NULL_AREA || sourceRegions[i] != 0)
					continue;

				int level = compactHeightfield.distancesToBorder[i] >> loglevelsPerStack;
				int sId = startLevel - level;
				if (sId >= (int)nbStacks)
					continue;
				if (sId < 0)
					sId = 0;

				stacks[sId].push_back(LevelStackEntry(x, y, i));
			}
		}
	}
}

static void appendStacks(const rcTempVector<LevelStackEntry>& sourceStack, rcTempVector<LevelStackEntry>& destinationStack, const unsigned short* sourceRegions)
{
	for (int j = 0; j < sourceStack.size(); j++)
	{
		int i = sourceStack[j].index;
		if ((i < 0) || (sourceRegions[i] != 0))
			continue;
		destinationStack.push_back(sourceStack[j]);
	}
}

struct rcRegion
{
	inline rcRegion(unsigned short i) :
		spanCount(0),
		id(i),
		areaType(0),
		remap(false),
		visited(false),
		overlap(false),
		connectsToBorder(false),
		ymin(0xffff),
		ymax(0)
	{}

	int spanCount;					// Number of spans belonging to this region
	unsigned short id;				// ID of the region
	unsigned char areaType;			// Are type.
	bool remap;
	bool visited;
	bool overlap;
	bool connectsToBorder;
	unsigned short ymin, ymax;
	rcIntArray connections;
	rcIntArray floors;
};

static void removeAdjacentNeighbours(rcRegion& region)
{
	// Remove adjacent duplicates.
	for (int i = 0; i < region.connections.size() && region.connections.size() > 1; )
	{
		int ni = (i + 1) % region.connections.size();
		if (region.connections[i] == region.connections[ni])
		{
			// Remove duplicate
			for (int j = i; j < region.connections.size() - 1; ++j)
				region.connections[j] = region.connections[j + 1];
			region.connections.pop();
		}
		else
			++i;
	}
}

static void replaceNeighbour(rcRegion& region, unsigned short oldId, unsigned short newId)
{
	bool isNeighborChanged = false;
	for (int i = 0; i < region.connections.size(); ++i)
	{
		if (region.connections[i] == oldId)
		{
			region.connections[i] = newId;
			isNeighborChanged = true;
		}
	}
	for (int i = 0; i < region.floors.size(); ++i)
	{
		if (region.floors[i] == oldId)
			region.floors[i] = newId;
	}
	if (isNeighborChanged)
		removeAdjacentNeighbours(region);
}

static bool canMergeWithRegion(const rcRegion& regionA, const rcRegion& regionB)
{
	if (regionA.areaType != regionB.areaType)
		return false;
	int n = 0;
	for (int i = 0; i < regionA.connections.size(); ++i)
	{
		if (regionA.connections[i] == regionB.id)
			n++;
	}
	if (n > 1)
		return false;
	for (int i = 0; i < regionA.floors.size(); ++i)
	{
		if (regionA.floors[i] == regionB.id)
			return false;
	}
	return true;
}

static void addUniqueFloorRegion(rcRegion& region, int n)
{
	for (int i = 0; i < region.floors.size(); ++i)
		if (region.floors[i] == n)
			return;
	region.floors.push(n);
}

static bool mergeRegions(rcRegion& regionA, rcRegion& regionB)
{
	unsigned short aid = regionA.id;
	unsigned short bid = regionB.id;

	// Duplicate current neighbourhood.
	rcIntArray acon;
	acon.resize(regionA.connections.size());
	for (int i = 0; i < regionA.connections.size(); ++i)
		acon[i] = regionA.connections[i];
	rcIntArray& bcon = regionB.connections;

	// Find insertion point on A.
	int insa = -1;
	for (int i = 0; i < acon.size(); ++i)
	{
		if (acon[i] == bid)
		{
			insa = i;
			break;
		}
	}
	if (insa == -1)
		return false;

	// Find insertion point on B.
	int insb = -1;
	for (int i = 0; i < bcon.size(); ++i)
	{
		if (bcon[i] == aid)
		{
			insb = i;
			break;
		}
	}
	if (insb == -1)
		return false;

	// Merge neighbours.
	regionA.connections.clear();
	for (int i = 0, ni = acon.size(); i < ni - 1; ++i)
		regionA.connections.push(acon[(insa + 1 + i) % ni]);

	for (int i = 0, ni = bcon.size(); i < ni - 1; ++i)
		regionA.connections.push(bcon[(insb + 1 + i) % ni]);

	removeAdjacentNeighbours(regionA);

	for (int j = 0; j < regionB.floors.size(); ++j)
		addUniqueFloorRegion(regionA, regionB.floors[j]);
	regionA.spanCount += regionB.spanCount;
	regionB.spanCount = 0;
	regionB.connections.resize(0);

	return true;
}

static bool isRegionConnectedToBorder(const rcRegion& region)
{
	// Region is connected to border if
	// one of the neighbours is null regionId.
	for (int i = 0; i < region.connections.size(); ++i)
	{
		if (region.connections[i] == 0)
			return true;
	}
	return false;
}

static bool isSolidEdge(rcCompactHeightfield& compactHeightfield, const unsigned short* sourceRegions,
	int x, int y, int i, int direction)
{
	const rcCompactSpan& s = compactHeightfield.spans[i];
	unsigned short r = 0;
	if (rcGetConnection(s, direction) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirectionOffsetX(direction);
		const int ay = y + rcGetDirectionOffsetY(direction);
		const int ai = (int)compactHeightfield.cells[ax + ay * compactHeightfield.width].index + rcGetConnection(s, direction);
		r = sourceRegions[ai];
	}
	if (r == sourceRegions[i])
		return false;
	return true;
}

static void walkContour(int x, int y, int i, int direction,
	rcCompactHeightfield& compactHeightfield,
	const unsigned short* sourceRegions,
	rcIntArray& contour)
{
	int startDir = direction;
	int starti = i;

	const rcCompactSpan& ss = compactHeightfield.spans[i];
	unsigned short currentRegion = 0;
	if (rcGetConnection(ss, direction) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirectionOffsetX(direction);
		const int ay = y + rcGetDirectionOffsetY(direction);
		const int ai = (int)compactHeightfield.cells[ax + ay * compactHeightfield.width].index + rcGetConnection(ss, direction);
		currentRegion = sourceRegions[ai];
	}
	contour.push(currentRegion);

	int iterations = 0;
	while (++iterations < 40000)
	{
		const rcCompactSpan& compactSpan = compactHeightfield.spans[i];

		if (isSolidEdge(compactHeightfield, sourceRegions, x, y, i, direction))
		{
			// Choose the edge corner
			unsigned short r = 0;
			if (rcGetConnection(compactSpan, direction) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirectionOffsetX(direction);
				const int ay = y + rcGetDirectionOffsetY(direction);
				const int ai = (int)compactHeightfield.cells[ax + ay * compactHeightfield.width].index + rcGetConnection(compactSpan, direction);
				r = sourceRegions[ai];
			}
			if (r != currentRegion)
			{
				currentRegion = r;
				contour.push(currentRegion);
			}

			direction = (direction + 1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirectionOffsetX(direction);
			const int ny = y + rcGetDirectionOffsetY(direction);
			if (rcGetConnection(compactSpan, direction) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = compactHeightfield.cells[nx + ny * compactHeightfield.width];
				ni = (int)nc.index + rcGetConnection(compactSpan, direction);
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

	// Remove adjacent duplicates.
	if (contour.size() > 1)
	{
		for (int j = 0; j < contour.size(); )
		{
			int nj = (j + 1) % contour.size();
			if (contour[j] == contour[nj])
			{
				for (int k = j; k < contour.size() - 1; ++k)
					contour[k] = contour[k + 1];
				contour.pop();
			}
			else
				++j;
		}
	}
}

static bool mergeAndFilterRegions(rcContext* context, int minRegionArea, int mergeRegionSize,
	unsigned short& maxRegionId,
	rcCompactHeightfield& compactHeightfield,
	unsigned short* sourceRegions, rcIntArray& overlaps)
{
	const int w = compactHeightfield.width;
	const int h = compactHeightfield.height;

	const int nreg = maxRegionId + 1;
	rcTempVector<rcRegion> regions;
	if (!regions.reserve(nreg)) {
		context->log(RC_LOG_ERROR, "mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}

	// Construct regions
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short)i));

	// Find edge of a region and find connections around the contour.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				unsigned short r = sourceRegions[i];
				if (r == 0 || r >= nreg)
					continue;

				rcRegion& region = regions[r];
				region.spanCount++;

				// Update floors.
				for (int j = (int)c.index; j < ni; ++j)
				{
					if (i == j) continue;
					unsigned short floorId = sourceRegions[j];
					if (floorId == 0 || floorId >= nreg)
						continue;
					if (floorId == r)
						region.overlap = true;
					addUniqueFloorRegion(region, floorId);
				}

				// Have found contour
				if (region.connections.size() > 0)
					continue;

				region.areaType = compactHeightfield.areaIds[i];

				// Check if this cell is next to a border.
				int ndir = -1;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (isSolidEdge(compactHeightfield, sourceRegions, x, y, i, dir))
					{
						ndir = dir;
						break;
					}
				}

				if (ndir != -1)
				{
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContour(x, y, i, ndir, compactHeightfield, sourceRegions, region.connections);
				}
			}
		}
	}

	// Remove too small regions.
	rcIntArray stack(32);
	rcIntArray trace(32);
	for (int i = 0; i < nreg; ++i)
	{
		rcRegion& reg = regions[i];
		if (reg.id == 0 || (reg.id & RC_BORDER_REGION))
			continue;
		if (reg.spanCount == 0)
			continue;
		if (reg.visited)
			continue;

		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		bool connectsToBorder = false;
		int spanCount = 0;
		stack.clear();
		trace.clear();

		reg.visited = true;
		stack.push(i);

		while (stack.size())
		{
			// Pop
			int ri = stack.pop();

			rcRegion& creg = regions[ri];

			spanCount += creg.spanCount;
			trace.push(ri);

			for (int j = 0; j < creg.connections.size(); ++j)
			{
				if (creg.connections[j] & RC_BORDER_REGION)
				{
					connectsToBorder = true;
					continue;
				}
				rcRegion& neireg = regions[creg.connections[j]];
				if (neireg.visited)
					continue;
				if (neireg.id == 0 || (neireg.id & RC_BORDER_REGION))
					continue;
				// Visit
				stack.push(neireg.id);
				neireg.visited = true;
			}
		}

		// If the accumulated regions size is too small, remove it.
		// Do not remove areaIds which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areaIds.
		if (spanCount < minRegionArea && !connectsToBorder)
		{
			// Kill all visited regions.
			for (int j = 0; j < trace.size(); ++j)
			{
				regions[trace[j]].spanCount = 0;
				regions[trace[j]].id = 0;
			}
		}
	}

	// Merge too small regions to neighbour regions.
	int mergeCount = 0;
	do
	{
		mergeCount = 0;
		for (int i = 0; i < nreg; ++i)
		{
			rcRegion& reg = regions[i];
			if (reg.id == 0 || (reg.id & RC_BORDER_REGION))
				continue;
			if (reg.overlap)
				continue;
			if (reg.spanCount == 0)
				continue;

			// Check to see if the region should be merged.
			if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg))
				continue;

			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			int smallest = 0xfffffff;
			unsigned short mergeId = reg.id;
			for (int j = 0; j < reg.connections.size(); ++j)
			{
				if (reg.connections[j] & RC_BORDER_REGION) continue;
				rcRegion& mreg = regions[reg.connections[j]];
				if (mreg.id == 0 || (mreg.id & RC_BORDER_REGION) || mreg.overlap) continue;
				if (mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg))
				{
					smallest = mreg.spanCount;
					mergeId = mreg.id;
				}
			}
			// Found new regionId.
			if (mergeId != reg.id)
			{
				unsigned short oldId = reg.id;
				rcRegion& target = regions[mergeId];

				// Merge neighbours.
				if (mergeRegions(target, reg))
				{
					// Fixup regions pointing to current region.
					for (int j = 0; j < nreg; ++j)
					{
						if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REGION)) continue;
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if (regions[j].id == oldId)
							regions[j].id = mergeId;
						// Replace the current region with the new one if the
						// current regions is neighbour.
						replaceNeighbour(regions[j], oldId, mergeId);
					}
					mergeCount++;
				}
			}
		}
	} while (mergeCount > 0);

	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;       // Skip nil regions.
		if (regions[i].id & RC_BORDER_REGION) continue;    // Skip external regions.
		regions[i].remap = true;
	}

	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;

	// Remap regions.
	for (int i = 0; i < compactHeightfield.spanCount; ++i)
	{
		if ((sourceRegions[i] & RC_BORDER_REGION) == 0)
			sourceRegions[i] = regions[sourceRegions[i]].id;
	}

	// Return regions that we found to be overlapping.
	for (int i = 0; i < nreg; ++i)
		if (regions[i].overlap)
			overlaps.push(regions[i].id);

	return true;
}


static void addUniqueConnection(rcRegion& regions, int n)
{
	for (int i = 0; i < regions.connections.size(); ++i)
		if (regions.connections[i] == n)
			return;
	regions.connections.push(n);
}

static bool mergeAndFilterLayerRegions(rcContext* context, int minRegionArea,
	unsigned short& maxRegionId,
	rcCompactHeightfield& compactHeightfield,
	unsigned short* sourceRegions)
{
	const int w = compactHeightfield.width;
	const int h = compactHeightfield.height;

	const int nreg = maxRegionId + 1;
	rcTempVector<rcRegion> regions;

	// Construct regions
	if (!regions.reserve(nreg)) {
		context->log(RC_LOG_ERROR, "mergeAndFilterLayerRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short)i));

	// Find region neighbours and overlapping regions.
	rcIntArray lregs(32);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * w];

			lregs.clear();

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = compactHeightfield.spans[i];
				const unsigned char area = compactHeightfield.areaIds[i];
				const unsigned short ri = sourceRegions[i];
				if (ri == 0 || ri >= nreg) continue;
				rcRegion& reg = regions[ri];

				reg.spanCount++;
				reg.areaType = area;

				reg.ymin = rcMin(reg.ymin, s.y);
				reg.ymax = rcMax(reg.ymax, s.y);

				// Collect all region layers.
				lregs.push(ri);

				// Update neighbours
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetConnection(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirectionOffsetX(dir);
						const int ay = y + rcGetDirectionOffsetY(dir);
						const int ai = (int)compactHeightfield.cells[ax + ay * w].index + rcGetConnection(s, dir);
						const unsigned short rai = sourceRegions[ai];
						if (rai > 0 && rai < nreg && rai != ri)
							addUniqueConnection(reg, rai);
						if (rai & RC_BORDER_REGION)
							reg.connectsToBorder = true;
					}
				}

			}

			// Update overlapping regions.
			for (int i = 0; i < lregs.size() - 1; ++i)
			{
				for (int j = i + 1; j < lregs.size(); ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcRegion& ri = regions[lregs[i]];
						rcRegion& rj = regions[lregs[j]];
						addUniqueFloorRegion(ri, lregs[j]);
						addUniqueFloorRegion(rj, lregs[i]);
					}
				}
			}

		}
	}

	// Create 2D layers from regions.
	unsigned short layerId = 1;

	for (int i = 0; i < nreg; ++i)
		regions[i].id = 0;

	// Merge montone regions to create non-overlapping areaIds.
	rcIntArray stack(32);
	for (int i = 1; i < nreg; ++i)
	{
		rcRegion& root = regions[i];
		// Skip already visited.
		if (root.id != 0)
			continue;

		// Start search.
		root.id = layerId;

		stack.clear();
		stack.push(i);

		while (stack.size() > 0)
		{
			// Pop front
			rcRegion& reg = regions[stack[0]];
			for (int j = 0; j < stack.size() - 1; ++j)
				stack[j] = stack[j + 1];
			stack.resize(stack.size() - 1);

			const int ncons = (int)reg.connections.size();
			for (int j = 0; j < ncons; ++j)
			{
				const int nei = reg.connections[j];
				rcRegion& regn = regions[nei];
				// Skip already visited.
				if (regn.id != 0)
					continue;
				// Skip if different areaId type, do not connect regions with different areaId type.
				if (reg.areaType != regn.areaType)
					continue;
				// Skip if the neighbour is overlapping root region.
				bool overlap = false;
				for (int k = 0; k < root.floors.size(); k++)
				{
					if (root.floors[k] == nei)
					{
						overlap = true;
						break;
					}
				}
				if (overlap)
					continue;

				// Deepen
				stack.push(nei);

				// Mark layer regionId
				regn.id = layerId;
				// Merge current layers to root.
				for (int k = 0; k < regn.floors.size(); ++k)
					addUniqueFloorRegion(root, regn.floors[k]);
				root.ymin = rcMin(root.ymin, regn.ymin);
				root.ymax = rcMax(root.ymax, regn.ymax);
				root.spanCount += regn.spanCount;
				regn.spanCount = 0;
				root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
			}
		}

		layerId++;
	}

	// Remove small regions
	for (int i = 0; i < nreg; ++i)
	{
		if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder)
		{
			unsigned short reg = regions[i].id;
			for (int j = 0; j < nreg; ++j)
				if (regions[j].id == reg)
					regions[j].id = 0;
		}
	}

	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;				// Skip nil regions.
		if (regions[i].id & RC_BORDER_REGION) continue;    // Skip external regions.
		regions[i].remap = true;
	}

	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;

	// Remap regions.
	for (int i = 0; i < compactHeightfield.spanCount; ++i)
	{
		if ((sourceRegions[i] & RC_BORDER_REGION) == 0)
			sourceRegions[i] = regions[sourceRegions[i]].id;
	}

	return true;
}



/// @par
/// 
/// This is usually the second to the last step in creating a fully built
/// compact heightfield.  This step is required before regions are built
/// using #rcBuildRegions or #rcBuildRegionsMonotone.
/// 
/// After this step, the distance data is available via the rcCompactHeightfield::maxDistanceToBorder
/// and rcCompactHeightfield::distancesToBorder fields.
///
/// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildDistanceField(rcContext* context, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_DISTANCEFIELD);

	if (compactHeightfield.distancesToBorder)
	{
		rcFree(compactHeightfield.distancesToBorder);
		compactHeightfield.distancesToBorder = 0;
	}

	unsigned short* src = (unsigned short*)rcAllocate(sizeof(unsigned short) * compactHeightfield.spanCount, RC_ALLOC_TEMPORARY);
	if (!src)
	{
		context->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'src' (%d).", compactHeightfield.spanCount);
		return false;
	}
	unsigned short* dst = (unsigned short*)rcAllocate(sizeof(unsigned short) * compactHeightfield.spanCount, RC_ALLOC_TEMPORARY);
	if (!dst)
	{
		context->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'dst' (%d).", compactHeightfield.spanCount);
		rcFree(src);
		return false;
	}

	unsigned short maxDist = 0;

	{
		rcScopedTimer timerDist(context, RC_TIMER_BUILD_DISTANCEFIELD_DIST);

		calculateDistanceField(compactHeightfield, src, maxDist);
		compactHeightfield.maxDistanceToBorder = maxDist;
	}

	{
		rcScopedTimer timerBlur(context, RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

		// Blur
		if (boxBlur(compactHeightfield, 1, src, dst) != src)
			rcSwap(src, dst);

		// Store distance.
		compactHeightfield.distancesToBorder = src;
	}

	rcFree(dst);

	return true;
}

static void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regionId,
	rcCompactHeightfield& compactHeightfield, unsigned short* sourceRegions)
{
	const int w = compactHeightfield.width;
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (compactHeightfield.areaIds[i] != RC_NULL_AREA)
					sourceRegions[i] = regionId;
			}
		}
	}
}


static constexpr unsigned short RC_NULL_NEI = 0xffff;

struct rcSweepSpan
{
	unsigned short rowId;				// row regionId
	unsigned short regionId;			// region regionId
	unsigned short samplesCount;		// number samples
	unsigned short neighborRegionId;	// neighbour regionId
};

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an areaId that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps 
/// reduce unnecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::regionId fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegionsMonotone(rcContext* context, rcCompactHeightfield& compactHeightfield,
	const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_REGIONS);

	const int width = compactHeightfield.width;
	const int height = compactHeightfield.height;
	unsigned short id = 1;

	rcScopedDelete<unsigned short> sourceRegionIds((unsigned short*)rcAllocate(sizeof(unsigned short) * compactHeightfield.spanCount, RC_ALLOC_TEMPORARY));
	if (!sourceRegionIds)
	{
		context->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", compactHeightfield.spanCount);
		return false;
	}
	memset(sourceRegionIds, 0, sizeof(unsigned short) * compactHeightfield.spanCount);

	const int nsweeps = rcMax(compactHeightfield.width, compactHeightfield.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAllocate(sizeof(rcSweepSpan) * nsweeps, RC_ALLOC_TEMPORARY));
	if (!sweeps)
	{
		context->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}


	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int borderWidth = rcMin(width, borderSize);
		const int borderHeight = rcMin(height, borderSize);
		// Paint regions
		paintRectRegion(0, borderWidth, 0, height, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
		paintRectRegion(width - borderWidth, width, 0, height, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
		paintRectRegion(0, width, 0, borderHeight, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
		paintRectRegion(0, width, height - borderHeight, height, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
	}

	compactHeightfield.borderSize = borderSize;

	rcIntArray prev(256);

	// Sweep one line at a time.
	for (int y = borderSize; y < height - borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id + 1);
		memset(&prev[0], 0, sizeof(int) * id);
		unsigned short rid = 1;

		for (int x = borderSize; x < width - borderSize; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * width];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = compactHeightfield.spans[i];
				if (compactHeightfield.areaIds[i] == RC_NULL_AREA) continue;

				// -x
				unsigned short previd = 0;
				if (rcGetConnection(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirectionOffsetX(0);
					const int ay = y + rcGetDirectionOffsetY(0);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(s, 0);
					if ((sourceRegionIds[ai] & RC_BORDER_REGION) == 0 && compactHeightfield.areaIds[i] == compactHeightfield.areaIds[ai])
						previd = sourceRegionIds[ai];
				}

				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rowId = previd;
					sweeps[previd].samplesCount = 0;
					sweeps[previd].neighborRegionId = 0;
				}

				// -y
				if (rcGetConnection(s, 3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirectionOffsetX(3);
					const int ay = y + rcGetDirectionOffsetY(3);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(s, 3);
					if (sourceRegionIds[ai] && (sourceRegionIds[ai] & RC_BORDER_REGION) == 0 && compactHeightfield.areaIds[i] == compactHeightfield.areaIds[ai])
					{
						unsigned short nr = sourceRegionIds[ai];
						if (!sweeps[previd].neighborRegionId || sweeps[previd].neighborRegionId == nr)
						{
							sweeps[previd].neighborRegionId = nr;
							sweeps[previd].samplesCount++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].neighborRegionId = RC_NULL_NEI;
						}
					}
				}

				sourceRegionIds[i] = previd;
			}
		}

		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].neighborRegionId != RC_NULL_NEI && sweeps[i].neighborRegionId != 0 &&
				prev[sweeps[i].neighborRegionId] == (int)sweeps[i].samplesCount)
			{
				sweeps[i].regionId = sweeps[i].neighborRegionId;
			}
			else
			{
				sweeps[i].regionId = id++;
			}
		}

		// Remap IDs
		for (int x = borderSize; x < width - borderSize; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * width];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (sourceRegionIds[i] > 0 && sourceRegionIds[i] < rid)
					sourceRegionIds[i] = sweeps[sourceRegionIds[i]].regionId;
			}
		}
	}


	{
		rcScopedTimer timerFilter(context, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcIntArray overlaps;
		compactHeightfield.maxRegions = id;
		if (!mergeAndFilterRegions(context, minRegionArea, mergeRegionArea, compactHeightfield.maxRegions, compactHeightfield, sourceRegionIds, overlaps))
			return false;

		// Monotone partitioning does not generate overlapping regions.
	}

	// Store the result out.
	for (int i = 0; i < compactHeightfield.spanCount; ++i)
		compactHeightfield.spans[i].regionId = sourceRegionIds[i];

	return true;
}

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an areaId that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors. 
/// @p mergeRegionArea helps reduce unnecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::regionId fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegions(rcContext* context, rcCompactHeightfield& compactHeightfield,
	const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_REGIONS);

	const int width = compactHeightfield.width;
	const int height = compactHeightfield.height;

	rcScopedDelete<unsigned short> buffer((unsigned short*)rcAllocate(sizeof(unsigned short) * compactHeightfield.spanCount * 2, RC_ALLOC_TEMPORARY));
	if (!buffer)
	{
		context->log(RC_LOG_ERROR, "rcBuildRegions: Out of memory 'tmp' (%d).", compactHeightfield.spanCount * 4);
		return false;
	}

	context->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	constexpr int LOG_NB_STACKS = 3;
	constexpr int NB_STACKS = 1 << LOG_NB_STACKS;
	rcTempVector<LevelStackEntry> lvlStacks[NB_STACKS];
	for (int i = 0; i < NB_STACKS; ++i)
		lvlStacks[i].reserve(256);

	rcTempVector<LevelStackEntry> stack;
	stack.reserve(256);

	unsigned short* sourceRegionIds = buffer;
	unsigned short* sourceDistances = buffer + compactHeightfield.spanCount;

	memset(sourceRegionIds, 0, sizeof(unsigned short) * compactHeightfield.spanCount);
	memset(sourceDistances, 0, sizeof(unsigned short) * compactHeightfield.spanCount);

	unsigned short regionId = 1;
	unsigned short level = (compactHeightfield.maxDistanceToBorder + 1) & ~1;

	// TODO: Figure better formula, expandIters defines how much the 
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
//	const int expandIters = 4 + walkableRadius * 2;
	const int expandIters = 8;

	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(width, borderSize);
		const int bh = rcMin(height, borderSize);

		// Paint regions
		paintRectRegion(0, bw, 0, height, regionId | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); regionId++;
		paintRectRegion(width - bw, width, 0, height, regionId | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); regionId++;
		paintRectRegion(0, width, 0, bh, regionId | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); regionId++;
		paintRectRegion(0, width, height - bh, height, regionId | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); regionId++;
	}

	compactHeightfield.borderSize = borderSize;

	int sId = -1;
	while (level > 0)
	{
		level = level >= 2 ? level - 2 : 0;
		sId = (sId + 1) & (NB_STACKS - 1);

		//		context->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if (sId == 0)
			sortCellsByLevel(level, compactHeightfield, sourceRegionIds, NB_STACKS, lvlStacks, 1);
		else
			appendStacks(lvlStacks[sId - 1], lvlStacks[sId], sourceRegionIds); // copy left overs from last level

		//		context->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			rcScopedTimer timerExpand(context, RC_TIMER_BUILD_REGIONS_EXPAND);

			// Expand current regions until no empty connected cells found.
			expandRegions(expandIters, level, compactHeightfield, sourceRegionIds, sourceDistances, lvlStacks[sId], false);
		}

		{
			rcScopedTimer timerFloor(context, RC_TIMER_BUILD_REGIONS_FLOOD);

			// Mark new regions with IDs.
			for (int j = 0; j < lvlStacks[sId].size(); j++)
			{
				LevelStackEntry current = lvlStacks[sId][j];
				int x = current.x;
				int y = current.y;
				int i = current.index;
				if (i >= 0 && sourceRegionIds[i] == 0)
				{
					if (floodRegion(x, y, i, level, regionId, compactHeightfield, sourceRegionIds, sourceDistances, stack))
					{
						if (regionId == 0xFFFF)
						{
							context->log(RC_LOG_ERROR, "rcBuildRegions: Region ID overflow");
							return false;
						}

						regionId++;
					}
				}
			}
		}
	}

	// Expand current regions until no empty connected cells found.
	expandRegions(expandIters * 8, 0, compactHeightfield, sourceRegionIds, sourceDistances, stack, true);

	context->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	{
		rcScopedTimer timerFilter(context, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcIntArray overlaps;
		compactHeightfield.maxRegions = regionId;
		if (!mergeAndFilterRegions(context, minRegionArea, mergeRegionArea, compactHeightfield.maxRegions, compactHeightfield, sourceRegionIds, overlaps))
			return false;

		// If overlapping regions were found during merging, split those regions.
		if (overlaps.size() > 0)
		{
			context->log(RC_LOG_ERROR, "rcBuildRegions: %d overlapping regions.", overlaps.size());
		}
	}

	// Write the result out.
	for (int i = 0; i < compactHeightfield.spanCount; ++i)
		compactHeightfield.spans[i].regionId = sourceRegionIds[i];

	return true;
}


bool rcBuildLayerRegions(rcContext* context, rcCompactHeightfield& compactHeightfield, const int borderSize, const int minRegionArea)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_REGIONS);

	const int width = compactHeightfield.width;
	const int height = compactHeightfield.height;
	unsigned short id = 1;

	rcScopedDelete<unsigned short> sourceRegionIds((unsigned short*)rcAllocate(sizeof(unsigned short) * compactHeightfield.spanCount, RC_ALLOC_TEMPORARY));
	if (!sourceRegionIds)
	{
		context->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'src' (%d).", compactHeightfield.spanCount);
		return false;
	}
	memset(sourceRegionIds, 0, sizeof(unsigned short) * compactHeightfield.spanCount);

	const int nsweeps = rcMax(compactHeightfield.width, compactHeightfield.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAllocate(sizeof(rcSweepSpan) * nsweeps, RC_ALLOC_TEMPORARY));
	if (!sweeps)
	{
		context->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}


	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(width, borderSize);
		const int bh = rcMin(height, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, height, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
		paintRectRegion(width - bw, width, 0, height, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
		paintRectRegion(0, width, 0, bh, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
		paintRectRegion(0, width, height - bh, height, id | RC_BORDER_REGION, compactHeightfield, sourceRegionIds); id++;
	}

	compactHeightfield.borderSize = borderSize;

	rcIntArray prev(256);

	// Sweep one line at a time.
	for (int y = borderSize; y < height - borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id + 1);
		memset(&prev[0], 0, sizeof(int) * id);
		unsigned short rid = 1;

		for (int x = borderSize; x < width - borderSize; ++x)
		{
			const rcCompactCell& compactCell = compactHeightfield.cells[x + y * width];

			for (int i = (int)compactCell.index, ni = (int)(compactCell.index + compactCell.count); i < ni; ++i)
			{
				const rcCompactSpan& compactSpan = compactHeightfield.spans[i];
				if (compactHeightfield.areaIds[i] == RC_NULL_AREA) continue;

				// -x
				unsigned short previd = 0;
				if (rcGetConnection(compactSpan, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirectionOffsetX(0);
					const int ay = y + rcGetDirectionOffsetY(0);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(compactSpan, 0);
					if ((sourceRegionIds[ai] & RC_BORDER_REGION) == 0 && compactHeightfield.areaIds[i] == compactHeightfield.areaIds[ai])
						previd = sourceRegionIds[ai];
				}

				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rowId = previd;
					sweeps[previd].samplesCount = 0;
					sweeps[previd].neighborRegionId = 0;
				}

				// -y
				if (rcGetConnection(compactSpan, 3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirectionOffsetX(3);
					const int ay = y + rcGetDirectionOffsetY(3);
					const int ai = (int)compactHeightfield.cells[ax + ay * width].index + rcGetConnection(compactSpan, 3);
					if (sourceRegionIds[ai] && (sourceRegionIds[ai] & RC_BORDER_REGION) == 0 && compactHeightfield.areaIds[i] == compactHeightfield.areaIds[ai])
					{
						unsigned short nr = sourceRegionIds[ai];
						if (!sweeps[previd].neighborRegionId || sweeps[previd].neighborRegionId == nr)
						{
							sweeps[previd].neighborRegionId = nr;
							sweeps[previd].samplesCount++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].neighborRegionId = RC_NULL_NEI;
						}
					}
				}

				sourceRegionIds[i] = previd;
			}
		}

		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].neighborRegionId != RC_NULL_NEI && sweeps[i].neighborRegionId != 0 &&
				prev[sweeps[i].neighborRegionId] == (int)sweeps[i].samplesCount)
			{
				sweeps[i].regionId = sweeps[i].neighborRegionId;
			}
			else
			{
				sweeps[i].regionId = id++;
			}
		}

		// Remap IDs
		for (int x = borderSize; x < width - borderSize; ++x)
		{
			const rcCompactCell& compactCell = compactHeightfield.cells[x + y * width];

			for (int i = (int)compactCell.index, ni = (int)(compactCell.index + compactCell.count); i < ni; ++i)
			{
				if (sourceRegionIds[i] > 0 && sourceRegionIds[i] < rid)
					sourceRegionIds[i] = sweeps[sourceRegionIds[i]].regionId;
			}
		}
	}


	{
		rcScopedTimer timerFilter(context, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge monotone regions to layers and remove small regions.
		compactHeightfield.maxRegions = id;
		if (!mergeAndFilterLayerRegions(context, minRegionArea, compactHeightfield.maxRegions, compactHeightfield, sourceRegionIds))
			return false;
	}


	// Store the result out.
	for (int i = 0; i < compactHeightfield.spanCount; ++i)
		compactHeightfield.spans[i].regionId = sourceRegionIds[i];

	return true;
}
