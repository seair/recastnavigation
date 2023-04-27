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
#include "RecastAssert.h"

#include <stdlib.h>

// 可攀登高度的并且下面的span可行走的，也标记为可行走(直立楼梯)
void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			rcSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				const bool walkable = span->area != RC_NULL_AREA;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if (!walkable && previousWasWalkable)
				{
					if (rcAbs((int)span->smax - (int)previousSpan->smax) <= walkableClimb)
					{
						span->area = previousArea;
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWasWalkable = walkable;
				previousArea = span->area;
			}
		}
	}
}

/*
悬崖陡坡检测
定义邻居体素可达的条件为：min(top, ntop) - max(bot, nbot) > walkableHeight。则对于某个体素，其所有可达的邻居体素中：

1.如果存在(bot - nbot > walkableClimb)，则将该体素修正为不可行走。

2.如果max(nbot) - min(nbot) > walkableClimb，则将该体素修正为不可行走
*/
void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb,
                        rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
	
	// Mark border spans.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}
				// 上一个span的顶部作为bottom
				const int bot = (int)(span->smax);
				// 下一个span的底部作为top，如果没有下一个，用MAX_HEIGHT
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;

				// Find neighbours minimum height.
				int minNeighborHeight = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				// 可到达邻居最小和最大高度
				int accessibleNeighborMinHeight = span->smax;
				int accessibleNeighborMaxHeight = span->smax;
				// 这里y轴和z轴的定义有点和前面不一致了，其实应该是dx和dz
				// 遍历四个方向邻居找到最小、最大高度
				for (int direction = 0; direction < 4; ++direction)
				{
					// 邻居x坐标
					int dx = x + rcGetDirOffsetX(direction);
					// 邻居z坐标
					int dy = z + rcGetDirOffsetY(direction);
					// Skip neighbours which are out of bounds.
					// 已超出范围，更新最矮邻居(相当于直接标为不可走)
					if (dx < 0 || dy < 0 || dx >= xSize || dy >= zSize)
					{
						minNeighborHeight = rcMin(minNeighborHeight, -walkableClimb - bot);
						continue;
					}

					// 邻居spanFrom minus infinity to the first span.
					const rcSpan* neighborSpan = heightfield.spans[dx + dy * xSize];
					int neighborBot = -walkableClimb;// 第一个邻居span的bottom初始化为-walkableClimb
					int neighborTop = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHT;// 第一个邻居span的top
					
					// Skip neighbour if the gap between the spans is too small.
					// 两span间空隙大于可行走高度,才更新最小高度
					if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
					{
						minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);
					}

					// Rest of the spans.遍历该体素下所有span
					for (neighborSpan = heightfield.spans[dx + dy * xSize]; neighborSpan; neighborSpan = neighborSpan->next)
					{
						neighborBot = (int)neighborSpan->smax;
						neighborTop = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHT;
						
						// Skip neighbour if the gap between the spans is too small.
						// 高度大于身高，可行走
						if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
						{
							// 更新最小高度
							minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);

							// 可攀爬，更新可行走邻居数据accessible Find min/max accessible neighbour height. 
							if (rcAbs(neighborBot - bot) <= walkableClimb)
							{
								if (neighborBot < accessibleNeighborMinHeight) accessibleNeighborMinHeight = neighborBot;
								if (neighborBot > accessibleNeighborMaxHeight) accessibleNeighborMaxHeight = neighborBot;
							}

						}
					}
				}

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				if (minNeighborHeight < -walkableClimb)// 最矮邻居低于walkableClimb，悬崖
				{
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.邻居间距离太大，陡坡，不可行走
				else if ((accessibleNeighborMaxHeight - accessibleNeighborMinHeight) > walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

// 过滤高度不足站立的span，碰头
void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);
	
	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int bot = (int)(span->smax);
				const int top = span->next ? (int)(span->next->smin) : MAX_HEIGHT;
				if ((top - bot) <= walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}
