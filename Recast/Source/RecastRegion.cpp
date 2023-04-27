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

// 计算每个span距离和最大距离
static void calculateDistanceField(rcCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
{
	const int w = chf.width;
	const int h = chf.height;
	
	// Init distance and points.初始化为0xffff
	for (int i = 0; i < chf.spanCount; ++i)
		src[i] = 0xffff;
	
	// Mark boundary cells.边界标记
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];
				
				int nc = 0;
				for (int dir = 0; dir < 4; ++dir)// 遍历四个方向
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)// 联通
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						// 对应span的area相同
						if (area == chf.areas[ai])
							nc++;
					}
				}
				if (nc != 4)// 有方向不连通或者在不同area，标为0(边界)
					src[i] = 0;
			}
		}
	}
	
			
	// Pass 1，第一轮，从左下到右上扫描
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			// 遍历span
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)// 连通
				{
					// (-1,0)左边
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])// 半径为1
						src[i] = src[ai]+2;
					
					// (-1,-1)左下
					if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 3);
						if (src[aai]+3 < src[i])// 8的平方根约等于3
							src[i] = src[aai]+3;
					}
				}
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)下面
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (1,-1)右下
					if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 2);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}
	
	// Pass 2第二轮，从右上到左下扫描
	for (int y = h-1; y >= 0; --y)
	{
		for (int x = w-1; x >= 0; --x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 2);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (1,1)
					if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 1);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 1);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (-1,1)
					if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 0);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}	
	
	// 获取最大距离
	maxDist = 0;
	for (int i = 0; i < chf.spanCount; ++i)
		maxDist = rcMax(src[i], maxDist);
	
}

// 均值模糊
static unsigned short* boxBlur(rcCompactHeightfield& chf, int thr,
							   unsigned short* src, unsigned short* dst)
{
	const int w = chf.width;
	const int h = chf.height;
	
	thr *= 2;
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)// 遍历span
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned short cd = src[i];
				if (cd <= thr)
				{
					dst[i] = cd;
					continue;
				}

				int d = (int)cd;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						d += (int)src[ai];
						
						const rcCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir+1) & 0x3;
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
							d += (int)src[ai2];
						}
						else
						{
							d += cd;
						}
					}
					else
					{
						d += cd*2;
					}
				}
				// 自己和8个邻居的均值
				dst[i] = (unsigned short)((d+5)/9);
			}
		}
	}
	return dst;
}

// 泛洪标记新的水源点
static bool floodRegion(int x, int y, int i,
						unsigned short level, unsigned short r,
						rcCompactHeightfield& chf,
						unsigned short* srcReg, unsigned short* srcDist,
						rcTempVector<LevelStackEntry>& stack)
{
	const int w = chf.width;
	
	const unsigned char area = chf.areas[i];
	
	// Flood fill mark region.
	stack.clear();
	stack.push_back(LevelStackEntry(x, y, i));
	// 标记当前span的区域为r
	srcReg[i] = r;
	// 本身就是水源地，dist设为0
	srcDist[i] = 0;
	
	// 下一个级别level
	unsigned short lev = level >= 2 ? level-2 : 0;
	int count = 0;
	
	// 处理邻居
	while (stack.size() > 0)
	{
		LevelStackEntry& back = stack.back();
		int cx = back.x;
		int cy = back.y;
		int ci = back.index;
		stack.pop_back();
		
		const rcCompactSpan& cs = chf.spans[ci];
		
		// Check if any of the neighbours already have a valid region set.
		// 确认下是否8个邻居已经有合法的区域
		unsigned short ar = 0;
		for (int dir = 0; dir < 4; ++dir)// 遍历8个邻居
		{
			// 8 connected
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)// 轴邻居
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				// area不相等不考虑
				if (chf.areas[ai] != area)
					continue;
				unsigned short nr = srcReg[ai];
				// 边界邻居不考虑
				if (nr & RC_BORDER_REG) // Do not take borders into account.
					continue;
				if (nr != 0 && nr != r)// 不等于r的其它区域id
				{
					ar = nr;
					break;
				}
				
				const rcCompactSpan& as = chf.spans[ai];
				
				const int dir2 = (dir+1) & 0x3;
				if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)// 对角邻居
				{
					const int ax2 = ax + rcGetDirOffsetX(dir2);
					const int ay2 = ay + rcGetDirOffsetY(dir2);
					const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
					if (chf.areas[ai2] != area)
						continue;
					unsigned short nr2 = srcReg[ai2];
					if (nr2 != 0 && nr2 != r)// 不等于r的其它区域id
					{
						ar = nr2;
						break;
					}
				}				
			}
		}
		if (ar != 0)// 如果有邻居已经有其它合法区域id，不需要分配新的区域
		{
			srcReg[ci] = 0;
			continue;
		}
		
		count++;
		
		// Expand neighbours.
		// 如果轴邻居也是当前级别，也压入栈，继续扩大水源点
		for (int dir = 0; dir < 4; ++dir)
		{
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;
				if (chf.dist[ai] >= lev && srcReg[ai] == 0)
				{
					srcReg[ai] = r;
					srcDist[ai] = 0;
					stack.push_back(LevelStackEntry(ax, ay, ai));
				}
			}
		}
	}
	
	// 返回是否已经分配新的区域
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

// 扩展旧的水源点区域，向水源四周蔓延
static void expandRegions(
	int maxIter, // 扩展次数，一般意味着一个水源点能向外扩展多少个span
	unsigned short level, // 最小level
	rcCompactHeightfield& chf, // 高度场
	unsigned short* srcReg, // 区域数组
	unsigned short* srcDist, // 距离数组
	rcTempVector<LevelStackEntry>& stack, // 水源栈
	bool fillStack) // 是否重新填充水源栈
{
	const int w = chf.width;
	const int h = chf.height;

	if (fillStack)// 重新填充stack
	{
		// Find cells revealed by the raised level.
		stack.clear();
		for (int y = 0; y < h; ++y)// 遍历把所有可以行走的dist大于level的填充到stack
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x+y*w];
				for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
				{
					if (chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA)
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
		// 把已经有分区的cell的index置为不用标记，避免重复标记
		for (int j=0; j<stack.size(); j++)
		{
			int i = stack[j].index;
			if (srcReg[i] != 0)
				stack[j].index = -1;
		}
	}

	// 脏entry，先把这一轮要修改的记到这里，循环结束再一起标记
	// 以免影响maxIter的效果
	rcTempVector<DirtyEntry> dirtyEntries;
	int iter = 0;
	while (stack.size() > 0)// 遍历栈
	{
		// 记录stack里已经标记区域的span数量
		int failed = 0;
		// 清理dirtyEntries
		dirtyEntries.clear();
		
		for (int j = 0; j < stack.size(); j++)// 遍历stack打标志
		{
			int x = stack[j].x;
			int y = stack[j].y;
			int i = stack[j].index;
			if (i < 0)// 已标记
			{
				failed++;
				continue;
			}
			
			unsigned short r = srcReg[i];
			unsigned short d2 = 0xffff;
			const unsigned char area = chf.areas[i];
			const rcCompactSpan& s = chf.spans[i];
			for (int dir = 0; dir < 4; ++dir)// 看四个方向连通的邻居
			{
				// 不连通的邻居跳过
				if (rcGetCon(s, dir) == RC_NOT_CONNECTED) continue;

				// 获取邻居
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
				// 邻居area不等(动态阻挡或者不可行走)跳过
				if (chf.areas[ai] != area) continue;
				// 如果邻居已经有标记而且不是边界，考虑使用邻居的分区作为直接的分区
				if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0)
				{
					if ((int)srcDist[ai]+2 < (int)d2)// 找到邻居中最小的距离和分区
					{
						r = srcReg[ai];// 把邻居区域id记到r
						d2 = srcDist[ai]+2;// d2记为邻居距离+2（一般已经是距离的2的整数倍了）
					}
				}
			}
			if (r)// 如果有邻居已经标志（一般也是最小的r）
			{
				stack[j].index = -1; // 标记为已经分区 mark as used
				dirtyEntries.push_back(DirtyEntry(i, r, d2));// 先压入dirtyEntry
			}
			else// 邻居都未标记，先标记为failed，等待下一轮迭代
			{
				failed++;
			}
		}
		
		// Copy entries that differ between src and dst to keep them in sync.
		// 根据dirtyEntries设置分区和距离
		for (int i = 0; i < dirtyEntries.size(); i++) {// 遍历处理dirtyEntry
			int idx = dirtyEntries[i].index;
			srcReg[idx] = dirtyEntries[i].region;// 分区
			srcDist[idx] = dirtyEntries[i].distance2; // 距离
		}
		
		// 这一轮没有标记新的index,认为处理完成
		if (failed == stack.size())
			break;
		
		if (level > 0)// 如果level，则不管如何一直迭代到
		{
			++iter;// 遍历8次
			if (iter >= maxIter)
				break;
		}
	}
}


// 把cell按离边界距离按级别压入对应栈，不在这一批次的先不处理
static void sortCellsByLevel(unsigned short startLevel,
							  rcCompactHeightfield& chf,
							  const unsigned short* srcReg,
							  unsigned int nbStacks, rcTempVector<LevelStackEntry>* stacks,
							  unsigned short loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift
{
	const int w = chf.width;
	const int h = chf.height;
	startLevel = startLevel >> loglevelsPerStack;

	for (unsigned int j=0; j<nbStacks; ++j)
		stacks[j].clear();

	// put all cells in the level range into the appropriate stacks
	for (int y = 0; y < h; ++y)// 遍历所有cell
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)// 遍历span
			{
				if (chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0)// 不可达和边界跳过
					continue;

				// 计算级别
				int level = chf.dist[i] >> loglevelsPerStack;
				// 根据级别计算应该放在的栈下标
				int sId = startLevel - level;
				if (sId >= (int)nbStacks)// 不是这一批次，先留着
					continue;
				if (sId < 0)
					sId = 0;

				// 压入对于级别的stack
				stacks[sId].push_back(LevelStackEntry(x, y, i));
			}
		}
	}
}

// 把合法的entry从srcStack拷到dstStack
static void appendStacks(const rcTempVector<LevelStackEntry>& srcStack,
						 rcTempVector<LevelStackEntry>& dstStack,
						 const unsigned short* srcReg)
{
	for (int j=0; j<srcStack.size(); j++)
	{
		int i = srcStack[j].index;
		if ((i < 0) || (srcReg[i] != 0))
			continue;
		dstStack.push_back(srcStack[j]);
	}
}

// 区域结构
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

static void removeAdjacentNeighbours(rcRegion& reg)
{
	// Remove adjacent duplicates.
	for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1; )
	{
		int ni = (i+1) % reg.connections.size();
		if (reg.connections[i] == reg.connections[ni])
		{
			// Remove duplicate
			for (int j = i; j < reg.connections.size()-1; ++j)
				reg.connections[j] = reg.connections[j+1];
			reg.connections.pop();
		}
		else
			++i;
	}
}

static void replaceNeighbour(rcRegion& reg, unsigned short oldId, unsigned short newId)
{
	bool neiChanged = false;
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == oldId)
		{
			reg.connections[i] = newId;
			neiChanged = true;
		}
	}
	for (int i = 0; i < reg.floors.size(); ++i)
	{
		if (reg.floors[i] == oldId)
			reg.floors[i] = newId;
	}
	if (neiChanged)
		removeAdjacentNeighbours(reg);
}

// 区域a能否合并到区域b
static bool canMergeWithRegion(const rcRegion& rega, const rcRegion& regb)
{
	if (rega.areaType != regb.areaType)// areaType不同不能合并
		return false;
	int n = 0;
	for (int i = 0; i < rega.connections.size(); ++i)
	{
		if (rega.connections[i] == regb.id)
			n++;
	}
	if (n > 1)// 有多个span与区域相邻，不能合并（为什么？）
		return false;
	for (int i = 0; i < rega.floors.size(); ++i)// 区域a其中一层是区域b，不能合并（为什么）
	{
		if (rega.floors[i] == regb.id)
			return false;
	}
	return true;
}

static void addUniqueFloorRegion(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.floors.size(); ++i)
		if (reg.floors[i] == n)
			return;
	reg.floors.push(n);
}

// 把区域b合并到区域a
static bool mergeRegions(rcRegion& rega, rcRegion& regb)
{
	unsigned short aid = rega.id;
	unsigned short bid = regb.id;
	
	// Duplicate current neighbourhood.
	// 拷贝区域a当前邻居
	rcIntArray acon;
	acon.resize(rega.connections.size());
	for (int i = 0; i < rega.connections.size(); ++i)
		acon[i] = rega.connections[i];
	rcIntArray& bcon = regb.connections;
	
	// Find insertion point on A.
	// 找到插入点（与b区域相连的connection下标）
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
	// 找到b区域的插入点
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
	rega.connections.clear();
	// 先把a除了b以外原来的所有邻居push
	for (int i = 0, ni = acon.size(); i < ni-1; ++i)
		rega.connections.push(acon[(insa+1+i) % ni]);
	// 再把b除了a以外原来的所有邻居push	
	for (int i = 0, ni = bcon.size(); i < ni-1; ++i)
		rega.connections.push(bcon[(insb+1+i) % ni]);
	
	// 删除重复邻居
	removeAdjacentNeighbours(rega);
	
	// 把b的楼层加入a(过滤重复)
	for (int j = 0; j < regb.floors.size(); ++j)
		addUniqueFloorRegion(rega, regb.floors[j]);

	// 清空b区域数据
	rega.spanCount += regb.spanCount;
	regb.spanCount = 0;
	regb.connections.resize(0);

	return true;
}

static bool isRegionConnectedToBorder(const rcRegion& reg)
{
	// Region is connected to border if
	// one of the neighbours is null id.
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == 0)
			return true;
	}
	return false;
}

static bool isSolidEdge(rcCompactHeightfield& chf, const unsigned short* srcReg,
						int x, int y, int i, int dir)
{
	const rcCompactSpan& s = chf.spans[i];
	unsigned short r = 0;
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		r = srcReg[ai];
	}
	if (r == srcReg[i])
		return false;
	return true;
}

// 遍历统计每个区域的所有邻居区域
static void walkContour(int x, int y, int i, int dir,
						rcCompactHeightfield& chf,
						const unsigned short* srcReg,
						rcIntArray& cont)
{
	int startDir = dir;
	int starti = i;

	const rcCompactSpan& ss = chf.spans[i];
	unsigned short curReg = 0;
	if (rcGetCon(ss, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(ss, dir);
		// 邻居的区域id
		curReg = srcReg[ai];
	}
	// 把邻居区域id计入cont(是不是放在if里面更合理？)
	cont.push(curReg);
			
	int iter = 0;
	while (++iter < 40000)// 40000这个数字是怎么得来的？，这样的算法是为了保证同一邻居区域都是连续的吗？
	{
		const rcCompactSpan& s = chf.spans[i];
		
		if (isSolidEdge(chf, srcReg, x, y, i, dir))// 边界邻居
		{
			// Choose the edge corner
			unsigned short r = 0;
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
				r = srcReg[ai];
			}
			if (r != curReg)// 区域不同，计入边界列表
			{
				curReg = r;
				cont.push(curReg);
			}
			
			dir = (dir+1) & 0x3;  // 顺时针旋转 Rotate CW
		}
		else// 非边界邻居，以邻居为中心，逆时针继续寻找边界
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = chf.cells[nx+ny*chf.width];
				// ni是邻居的span_id
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	//逆时针旋转 Rotate CCW
		}
		
		// 节点回到了原地，方向也回到了原方向，结束
		if (starti == i && startDir == dir)
		{
			break;
		}
	}

	// Remove adjacent duplicates.
	// 删除重复邻居区域
	if (cont.size() > 1)
	{
		for (int j = 0; j < cont.size(); )
		{
			int nj = (j+1) % cont.size();
			if (cont[j] == cont[nj])
			{
				for (int k = j; k < cont.size()-1; ++k)
					cont[k] = cont[k+1];
				cont.pop();
			}
			else
				++j;
		}
	}
}

// 合并和过滤区域
static bool mergeAndFilterRegions(rcContext* ctx, int minRegionArea, int mergeRegionSize,
								  unsigned short& maxRegionId,
								  rcCompactHeightfield& chf,
								  unsigned short* srcReg, rcIntArray& overlaps)
{
	const int w = chf.width;
	const int h = chf.height;
	
	// 申请构造区域数组
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}
	// Construct regions
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find edge of a region and find connections around the contour.
	// 找到区域的边缘
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)// 遍历span
			{
				unsigned short r = srcReg[i];// 区域id
				if (r == 0 || r >= nreg)
					continue;
				
				// 增加区域span数量
				rcRegion& reg = regions[r];
				reg.spanCount++;
				
				// Update floors.
				// 更新楼层信息
				for (int j = (int)c.index; j < ni; ++j)
				{
					// 当前层跳过
					if (i == j) continue;

					// 该span的区域
					unsigned short floorId = srcReg[j];
					// 边界或0区域跳过
					if (floorId == 0 || floorId >= nreg)
						continue;
					// 如果同一层，记录重叠标志
					if (floorId == r)
						reg.overlap = true;

					// 记录span的各个楼层区域id(按从下到上)
					addUniqueFloorRegion(reg, floorId);
				}
				
				// Have found contour
				// 已经有邻居集合，跳过
				if (reg.connections.size() > 0)
					continue;
				
				// 记录area
				reg.areaType = chf.areas[i];
				
				// Check if this cell is next to a border.
				int ndir = -1;
				for (int dir = 0; dir < 4; ++dir)// 检查span是不是有边界(其它区域)邻居
				{
					if (isSolidEdge(chf, srcReg, x, y, i, dir))// 有边界邻居
					{
						// 记录方向
						ndir = dir;
						break;
					}
				}
				
				if (ndir != -1)// 有其它区域邻居
				{
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					// 遍历统计每个区域的所有邻居区域
					walkContour(x, y, i, ndir, chf, srcReg, reg.connections);
				}
			}
		}
	}

	// Remove too small regions.
	// 删除过小区域（统计所有连通区域的span数量）
	rcIntArray stack(32);
	rcIntArray trace(32);
	for (int i = 0; i < nreg; ++i)
	{
		rcRegion& reg = regions[i];
		if (reg.id == 0 || (reg.id & RC_BORDER_REG))// 边界区域
			continue;                       
		if (reg.spanCount == 0)// 数量为0
			continue;
		if (reg.visited)// 已访问
			continue;
		
		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		bool connectsToBorder = false;
		int spanCount = 0;
		stack.clear();
		trace.clear();

		// 把起始区域压入栈
		reg.visited = true;
		stack.push(i);
		
		while (stack.size())// 栈不为空
		{
			// Pop
			int ri = stack.pop();
			
			rcRegion& creg = regions[ri];

			spanCount += creg.spanCount;
			trace.push(ri);

			// 遍历所有邻居区域
			for (int j = 0; j < creg.connections.size(); ++j)
			{
				if (creg.connections[j] & RC_BORDER_REG)// 邻居区域是边界
				{
					connectsToBorder = true;
					continue;
				}
				// 邻居区域
				rcRegion& neireg = regions[creg.connections[j]];
				if (neireg.visited)// 邻居区域已访问
					continue;
				if (neireg.id == 0 || (neireg.id & RC_BORDER_REG))// 邻居区域是边界
					continue;
				// Visit
				// 压入邻居区域
				stack.push(neireg.id);
				// 标记已访问，避免重复统计
				neireg.visited = true;
			}
		}
		
		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		if (spanCount < minRegionArea && !connectsToBorder)// 连通的区域span数量太小，删除所有连通区域，连通边界区域不删除
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
	// 把过小的区域合并到最小的邻居区域
	int mergeCount = 0 ;
	do
	{
		mergeCount = 0;
		for (int i = 0; i < nreg; ++i)// 遍历区域
		{
			rcRegion& reg = regions[i];
			if (reg.id == 0 || (reg.id & RC_BORDER_REG))
				continue;
			if (reg.overlap)
				continue;
			if (reg.spanCount == 0)
				continue;
			
			// Check to see if the region should be merged.已经足够大，边界区域不合并
			if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg))
				continue;
			
			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			int smallest = 0xfffffff;
			unsigned short mergeId = reg.id;
			// 遍历找出最小的可以合并的邻居区域
			for (int j = 0; j < reg.connections.size(); ++j)
			{
				if (reg.connections[j] & RC_BORDER_REG) continue;
				rcRegion& mreg = regions[reg.connections[j]];
				if (mreg.id == 0 || (mreg.id & RC_BORDER_REG) || mreg.overlap) continue;
				if (mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg))
				{
					smallest = mreg.spanCount;
					mergeId = mreg.id;
				}
			}
			// Found new id.
			if (mergeId != reg.id)// 可以合并
			{
				unsigned short oldId = reg.id;
				rcRegion& target = regions[mergeId];
				
				// Merge neighbours.
				if (mergeRegions(target, reg))// 合并区域
				{
					// Fixup regions pointing to current region.
					for (int j = 0; j < nreg; ++j)
					{
						if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REG)) continue;
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if (regions[j].id == oldId)
							regions[j].id = mergeId;
						// Replace the current region with the new one if the
						// current regions is neighbour.
						// 把邻居里所有的旧区域id更新
						replaceNeighbour(regions[j], oldId, mergeId);
					}
					mergeCount++;
				}
			}
		}
	}
	while (mergeCount > 0);
	
	// Compress region Ids.
	// 压缩区域id
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;       // Skip nil regions.区域已被干掉
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.外部区域
		regions[i].remap = true; // 标记remap
	}
	
	// 重新生成区域id
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
	// 重新映射新分区id到srcReg
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}

	// Return regions that we found to be overlapping.
	// 把由重叠区域记录到overlaps数组(多个span在同一层)
	for (int i = 0; i < nreg; ++i)
		if (regions[i].overlap)
			overlaps.push(regions[i].id);

	return true;
}


static void addUniqueConnection(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.connections.size(); ++i)
		if (reg.connections[i] == n)
			return;
	reg.connections.push(n);
}

static bool mergeAndFilterLayerRegions(rcContext* ctx, int minRegionArea,
									   unsigned short& maxRegionId,
									   rcCompactHeightfield& chf,
									   unsigned short* srcReg)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	
	// Construct regions
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterLayerRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find region neighbours and overlapping regions.
	rcIntArray lregs(32);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];

			lregs.clear();
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned short ri = srcReg[i];
				if (ri == 0 || ri >= nreg) continue;
				rcRegion& reg = regions[ri];
				
				reg.spanCount++;
				
				reg.ymin = rcMin(reg.ymin, s.y);
				reg.ymax = rcMax(reg.ymax, s.y);
				
				// Collect all region layers.
				lregs.push(ri);
				
				// Update neighbours
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const unsigned short rai = srcReg[ai];
						if (rai > 0 && rai < nreg && rai != ri)
							addUniqueConnection(reg, rai);
						if (rai & RC_BORDER_REG)
							reg.connectsToBorder = true;
					}
				}
				
			}
			
			// Update overlapping regions.
			for (int i = 0; i < lregs.size()-1; ++i)
			{
				for (int j = i+1; j < lregs.size(); ++j)
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

	// Merge montone regions to create non-overlapping areas.
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
			for (int j = 0; j < stack.size()-1; ++j)
				stack[j] = stack[j+1];
			stack.resize(stack.size()-1);
			
			const int ncons = (int)reg.connections.size();
			for (int j = 0; j < ncons; ++j)
			{
				const int nei = reg.connections[j];
				rcRegion& regn = regions[nei];
				// Skip already visited.
				if (regn.id != 0)
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
					
				// Mark layer id
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
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
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
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}
	
	return true;
}



/// @par
/// 
/// This is usually the second to the last step in creating a fully built
/// compact heightfield.  This step is required before regions are built
/// using #rcBuildRegions or #rcBuildRegionsMonotone.
/// 
/// After this step, the distance data is available via the rcCompactHeightfield::maxDistance
/// and rcCompactHeightfield::dist fields.
///
/// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
/// 创建距离场，计算每个span离边界span，结果均值模糊（与rcErodeWalkableArea有所不同）最小距离
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_DISTANCEFIELD);
	
	if (chf.dist)
	{
		rcFree(chf.dist);
		chf.dist = 0;
	}
	
	unsigned short* src = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!src)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	unsigned short* dst = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!dst)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'dst' (%d).", chf.spanCount);
		rcFree(src);
		return false;
	}
	
	unsigned short maxDist = 0;

	{
		rcScopedTimer timerDist(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST);
		// 计算每个span距离和最大距离
		calculateDistanceField(chf, src, maxDist);
		chf.maxDistance = maxDist;
	}

	{
		rcScopedTimer timerBlur(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

		// Blur
		// blur平滑，和周围8个邻居的均值平滑
		if (boxBlur(chf, 1, src, dst) != src)
			rcSwap(src, dst);

		// Store distance.
		chf.dist = src;
	}
	
	rcFree(dst);
	
	return true;
}

static void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regId,
							rcCompactHeightfield& chf, unsigned short* srcReg)
{
	const int w = chf.width;	
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.areas[i] != RC_NULL_AREA)
					srcReg[i] = regId;
			}
		}
	}
}


static const unsigned short RC_NULL_NEI = 0xffff;

struct rcSweepSpan
{
	unsigned short rid;	// row id
	unsigned short id;	// region id
	unsigned short ns;	// number samples
	unsigned short nei;	// neighbour id
};

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps 
/// reduce unecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
							const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcIntArray prev(256);

	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}


	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcIntArray overlaps;
		chf.maxRegions = id;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// Monotone partitioning does not generate overlapping regions.
	}
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors. 
/// @p mergeRegionArea helps reduce unecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
/// 分水岭分区
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
					const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedDelete<unsigned short> buf((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount*2, RC_ALLOC_TEMP));
	if (!buf)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegions: Out of memory 'tmp' (%d).", chf.spanCount*4);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	const int LOG_NB_STACKS = 3;
	const int NB_STACKS = 1 << LOG_NB_STACKS;
	// 8个栈，每个256
	rcTempVector<LevelStackEntry> lvlStacks[NB_STACKS];
	for (int i=0; i<NB_STACKS; ++i)
		lvlStacks[i].reserve(256);

	// 临时栈，新水源地使用
	rcTempVector<LevelStackEntry> stack;
	stack.reserve(256);
	
	// 区域id数组
	unsigned short* srcReg = buf;
	// 离水源点距离数组
	unsigned short* srcDist = buf+chf.spanCount;
	
	memset(srcReg, 0, sizeof(unsigned short)*chf.spanCount);
	memset(srcDist, 0, sizeof(unsigned short)*chf.spanCount);
	// 区域id
	unsigned short regionId = 1;// 从1开始
	// 奇数时，从maxDistance+1开始；偶数时，从maxDistance开始
	unsigned short level = (chf.maxDistance+1) & ~1;

	// TODO: Figure better formula, expandIters defines how much the 
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
//	const int expandIters = 4 + walkableRadius * 2;
	// 贪心指标
	const int expandIters = 8;

	if (borderSize > 0)// 边界处理
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		
		// Paint regions
		// 四条边界regionId加上border标志
		paintRectRegion(0, bw, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(w-bw, w, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, 0, bh, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, h-bh, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
	}

	chf.borderSize = borderSize;
	// 栈id
	int sId = -1;
	// 逐层水淹
	while (level > 0)
	{
		// 每2格一层
		level = level >= 2 ? level-2 : 0;
		// 每8个level一批，减少内存占用
		// 栈id
		sId = (sId+1) & (NB_STACKS-1);

//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if (sId == 0)// 把cell按离边界距离按级别压入对应栈，不在这一批次的先不处理
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1);
		else // 把上一个级别的水源点拷到当前级别
			appendStacks(lvlStacks[sId-1], lvlStacks[sId], srcReg); // copy left overs from last level

//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			rcScopedTimer timerExpand(ctx, RC_TIMER_BUILD_REGIONS_EXPAND);

			// Expand current regions until no empty connected cells found.
			// 扩展旧的水源点区域，向水源四周蔓延
			expandRegions(expandIters, level, chf, srcReg, srcDist, lvlStacks[sId], false);
		}
		
		{
			rcScopedTimer timerFloor(ctx, RC_TIMER_BUILD_REGIONS_FLOOD);

			// Mark new regions with IDs.
			// 剩余未扩展的水源点，分配新的水源点区域
			for (int j = 0; j<lvlStacks[sId].size(); j++)// 遍历每个水源点
			{
				LevelStackEntry current = lvlStacks[sId][j];
				int x = current.x;
				int y = current.y;
				int i = current.index;
				if (i >= 0 && srcReg[i] == 0)
				{
					// 泛洪标记新的水源点
					if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack))
					{
						if (regionId == 0xFFFF)// region数量不足
						{
							ctx->log(RC_LOG_ERROR, "rcBuildRegions: Region ID overflow");
							return false;
						}
						
						regionId++;// 自增regionId
					}
				}
			}
		}
	}
	
	// Expand current regions until no empty connected cells found.
	// 剩余的0级的填充stack并扩展区域，扩展因子变为64(其实64应该没生效，因为level是0)
	expandRegions(expandIters*8, 0, chf, srcReg, srcDist, stack, true);
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out smalle regions.
		rcIntArray overlaps;
		chf.maxRegions = regionId;
		// 合并过滤区域
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// If overlapping regions were found during merging, split those regions.
		if (overlaps.size() > 0)// 重叠区域
		{
			ctx->log(RC_LOG_ERROR, "rcBuildRegions: %d overlapping regions.", overlaps.size());
		}
	}
		
	// Write the result out.
	// 记录到span的reg字段
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}


bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf,
						 const int borderSize, const int minRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);
	
	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcIntArray prev(256);
	
	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}
				
				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}
				
				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}
	
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge monotone regions to layers and remove small regions.
		chf.maxRegions = id;
		if (!mergeAndFilterLayerRegions(ctx, minRegionArea, chf.maxRegions, chf, srcReg))
			return false;
	}
	
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}
