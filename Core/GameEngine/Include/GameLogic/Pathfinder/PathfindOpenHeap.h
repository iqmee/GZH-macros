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

#include <vector>

class PathfindCell;

/**
 * Cache-friendly, deterministic 4-ary indexed min-heap for A* open list management.
 * Stores totalCost inline alongside a monotonic sequence number so comparisons avoid
 * pointer chasing and break cost ties deterministically across all platforms.
 */
class PathfindOpenHeap
{
public:
	struct Entry
	{
		UnsignedInt totalCost;
		UnsignedInt sequence;
		PathfindCell *cell;
	};

	PathfindOpenHeap();

	void reset();

	Bool empty() const { return m_entries.empty(); }
	size_t size() const { return m_entries.size(); }

	PathfindCell *getHead() const { return m_entries.empty() ? nullptr : m_entries[0].cell; }
	PathfindCell *getAt(size_t index) const { return m_entries[index].cell; }

	void push(PathfindCell *cell);
	void remove(PathfindCell *cell);
	Int releaseAll();

private:
	static Bool isLess(const Entry &a, const Entry &b)
	{
		return (a.totalCost < b.totalCost) || (a.totalCost == b.totalCost && a.sequence < b.sequence);
	}

	void siftUp(size_t index);
	void siftDown(size_t index);

	std::vector<Entry> m_entries;
	UnsignedInt m_nextSequence;
};
