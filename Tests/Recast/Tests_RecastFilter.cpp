#include <stdio.h>
#include <string.h>
#include <vector>

#include "catch2/catch_all.hpp"

#include "Recast.h"
#include "RecastAlloc.h"

TEST_CASE("rcFilterLowHangingWalkableObstacles", "[recast, filtering]")
{
	rcContext context;
	int walkableHeight = 5;

	rcHeightfield heightfield;
	heightfield.width = 1;
	heightfield.height = 1;
	heightfield.boundMin[0] = 0;
	heightfield.boundMin[1] = 0;
	heightfield.boundMin[2] = 0;
	heightfield.boundMax[0] = 1;
	heightfield.boundMax[1] = 1;
	heightfield.boundMax[2] = 1;
	heightfield.cellSize = 1;
	heightfield.cellHeight = 1;
	heightfield.spans = (rcSpan**)rcAllocate(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERMANENT);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("Span with no spans above it is unchanged")
	{
		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);

		rcFree(span);
	}

	SECTION("Span with span above that is higher than walkableHeight is unchanged")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 1 + walkableHeight;
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that nothing has changed.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		// Check again but with a more clearance
		secondSpan->smin += 10;
		secondSpan->smax += 10;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that nothing has changed.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Marks low obstacles walkable if they're below the walkableClimb")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 1 + (walkableHeight - 1);
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that the second span was changed to walkable.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == 1);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Low obstacle that overlaps the walkableClimb distance is not changed")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 2 + (walkableHeight - 1);
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that the second span was changed to walkable.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Only the first of multiple, low obstacles are marked walkable")
	{
		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcSpan* previousSpan = span;
		for (int i = 0; i < 9; ++i)
		{
			rcSpan* nextSpan = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
			nextSpan->area = RC_NULL_AREA;
			nextSpan->next = NULL;
			nextSpan->smin = previousSpan->smax + (walkableHeight - 1);
			nextSpan->smax = nextSpan->smin + 1;
			previousSpan->next = nextSpan;
			previousSpan = nextSpan;
		}

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		rcSpan* currentSpan = heightfield.spans[0];
		for (int i = 0; i < 10; ++i)
		{
			REQUIRE(currentSpan != NULL);
			// only the first and second spans should be marked as walkabl
			REQUIRE(currentSpan->area == (i <= 1 ? 1 : RC_NULL_AREA));
			currentSpan = currentSpan->next;
		}

		std::vector<rcSpan*> toFree;
		span = heightfield.spans[0];
		for (int i = 0; i < 10; ++i)
		{
			toFree.push_back(span);
			span = span->next;
		}

		for (int i = 0; i < 10; ++i)
		{
			rcFree(toFree[i]);
		}
	}
}

TEST_CASE("rcFilterLedgeSpans", "[recast, filtering]")
{
	rcContext context;
	int walkableClimb = 5;
	int walkableHeight = 10;

	rcHeightfield heightfield;
	heightfield.width = 10;
	heightfield.height = 10;
	heightfield.boundMin[0] = 0;
	heightfield.boundMin[1] = 0;
	heightfield.boundMin[2] = 0;
	heightfield.boundMax[0] = 10;
	heightfield.boundMax[1] = 1;
	heightfield.boundMax[2] = 10;
	heightfield.cellSize = 1;
	heightfield.cellHeight = 1;
	heightfield.spans = (rcSpan**)rcAllocate(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERMANENT);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("Edge spans are marked unwalkable")
	{
		// Create a flat plane.
		for (int x = 0; x < heightfield.width; ++x)
		{
			for (int z = 0; z < heightfield.height; ++z)
			{
				rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
				span->area = 1;
				span->next = NULL;
				span->smin = 0;
				span->smax = 1;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		for (int x = 0; x < heightfield.width; ++x)
		{
			for (int z = 0; z < heightfield.height; ++z)
			{
				rcSpan* span = heightfield.spans[x + z * heightfield.width];
				REQUIRE(span != NULL);

				if (x == 0 || z == 0 || x == 9 || z == 9)
				{
					REQUIRE(span->area == RC_NULL_AREA);
				}
				else
				{
					REQUIRE(span->area == 1);
				}

				REQUIRE(span->next == NULL);
				REQUIRE(span->smin == 0);
				REQUIRE(span->smax == 1);
			}
		}

		// Free all the heightfield spans
		for (int x = 0; x < heightfield.width; ++x)
		{
			for (int z = 0; z < heightfield.height; ++z)
			{
				rcFree(heightfield.spans[x + z * heightfield.width]);
			}
		}
	}
}

TEST_CASE("rcFilterWalkableLowHeightSpans", "[recast, filtering]")
{
	rcContext context;
	int walkableHeight = 5;

	rcHeightfield heightfield;
	heightfield.width = 1;
	heightfield.height = 1;
	heightfield.boundMin[0] = 0;
	heightfield.boundMin[1] = 0;
	heightfield.boundMin[2] = 0;
	heightfield.boundMax[0] = 1;
	heightfield.boundMax[1] = 1;
	heightfield.boundMax[2] = 1;
	heightfield.cellSize = 1;
	heightfield.cellHeight = 1;
	heightfield.spans = (rcSpan**)rcAllocate(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERMANENT);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("span nothing above is unchanged")
	{
		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);

		rcFree(span);
	}

	SECTION("span with lots of room above is unchanged")
	{
		rcSpan* overheadSpan = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		overheadSpan->area = RC_NULL_AREA;
		overheadSpan->next = NULL;
		overheadSpan->smin = 10;
		overheadSpan->smax = 11;

		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = overheadSpan;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(overheadSpan);
		rcFree(span);
	}

	SECTION("Span with low hanging obstacle is marked as unwalkable")
	{
		rcSpan* overheadSpan = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		overheadSpan->area = RC_NULL_AREA;
		overheadSpan->next = NULL;
		overheadSpan->smin = 3;
		overheadSpan->smax = 4;

		rcSpan* span = (rcSpan*)rcAllocate(sizeof(rcSpan), RC_ALLOC_PERMANENT);
		span->area = 1;
		span->next = overheadSpan;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == RC_NULL_AREA);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(overheadSpan);
		rcFree(span);
	}
}