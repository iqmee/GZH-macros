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

#include "PreRTS.h"
#include "GameLogic/Pathfinder/PathfindCell.h"
#include "GameLogic/Pathfinder/PathfindOpenHeap.h"

namespace
{
	constexpr size_t INITIAL_HEAP_RESERVE = 4096;
}

PathfindOpenHeap::PathfindOpenHeap()
	: m_nextSequence(0)
{
	m_entries.reserve(INITIAL_HEAP_RESERVE);
}

void PathfindOpenHeap::reset()
{
	m_entries.clear();
	m_nextSequence = 0;
}

void PathfindOpenHeap::push(PathfindCell *cell)
{
	DEBUG_ASSERTCRASH(cell && cell->hasInfo(), ("PathfindOpenHeap::push requires allocated cell info."));
	DEBUG_ASSERTCRASH(!cell->getOpen() && !cell->getClosed(), ("PathfindOpenHeap::push cell already open or closed."));

	cell->setOpenFlag(true);
	cell->setClosedFlag(false);

	Entry entry;
	entry.totalCost = cell->getTotalCost();
	entry.sequence = m_nextSequence++;
	entry.cell = cell;

	const size_t insertIndex = m_entries.size();
	m_entries.push_back(entry);
	siftUp(insertIndex);
}

void PathfindOpenHeap::remove(PathfindCell *cell)
{
	DEBUG_ASSERTCRASH(cell && cell->hasInfo(), ("PathfindOpenHeap::remove requires allocated cell info."));
	DEBUG_ASSERTCRASH(!cell->getClosed() && cell->getOpen(), ("PathfindOpenHeap::remove cell not on open heap."));

	const Int heapIndex = cell->getHeapIndex();
	DEBUG_ASSERTCRASH(heapIndex >= 0 && static_cast<size_t>(heapIndex) < m_entries.size(), ("Invalid heap index on open cell."));
	if (heapIndex < 0 || static_cast<size_t>(heapIndex) >= m_entries.size())
	{
		cell->setHeapIndex(-1);
		cell->setOpenFlag(false);
		return;
	}

	const size_t removeIndex = static_cast<size_t>(heapIndex);
	const size_t lastIndex = m_entries.size() - 1;

	cell->setHeapIndex(-1);
	cell->setOpenFlag(false);

	if (removeIndex == lastIndex)
	{
		m_entries.pop_back();
		return;
	}

	Entry replacement = m_entries[lastIndex];
	m_entries.pop_back();
	m_entries[removeIndex] = replacement;
	replacement.cell->setHeapIndex(static_cast<Int>(removeIndex));

	if (removeIndex > 0 && isLess(m_entries[removeIndex], m_entries[(removeIndex - 1) >> 2]))
	{
		siftUp(removeIndex);
	}
	else
	{
		siftDown(removeIndex);
	}
}

Int PathfindOpenHeap::releaseAll()
{
	const Int count = static_cast<Int>(m_entries.size());
	for (size_t i = 0; i < m_entries.size(); ++i)
	{
		PathfindCell *cur = m_entries[i].cell;
		DEBUG_ASSERTCRASH(cur && cur->hasInfo(), ("Dangling cell in open heap."));
		if (cur && cur->hasInfo())
		{
			cur->setHeapIndex(-1);
			cur->setOpenFlag(false);
			cur->releaseInfo();
		}
	}
	m_entries.clear();
	m_nextSequence = 0;
	return count;
}

void PathfindOpenHeap::siftUp(size_t index)
{
	Entry moving = m_entries[index];
	while (index > 0)
	{
		const size_t parent = (index - 1) >> 2;
		if (!isLess(moving, m_entries[parent]))
		{
			break;
		}
		m_entries[index] = m_entries[parent];
		m_entries[index].cell->setHeapIndex(static_cast<Int>(index));
		index = parent;
	}
	m_entries[index] = moving;
	moving.cell->setHeapIndex(static_cast<Int>(index));
}

void PathfindOpenHeap::siftDown(size_t index)
{
	const size_t count = m_entries.size();
	Entry moving = m_entries[index];

	for (;;)
	{
		const size_t firstChild = (index << 2) + 1;
		if (firstChild >= count)
		{
			break;
		}

		size_t bestChild = firstChild;
		const size_t lastChild = (firstChild + 4 < count) ? (firstChild + 4) : count;
		for (size_t child = firstChild + 1; child < lastChild; ++child)
		{
			if (isLess(m_entries[child], m_entries[bestChild]))
			{
				bestChild = child;
			}
		}

		if (!isLess(m_entries[bestChild], moving))
		{
			break;
		}

		m_entries[index] = m_entries[bestChild];
		m_entries[index].cell->setHeapIndex(static_cast<Int>(index));
		index = bestChild;
	}

	m_entries[index] = moving;
	moving.cell->setHeapIndex(static_cast<Int>(index));
}
