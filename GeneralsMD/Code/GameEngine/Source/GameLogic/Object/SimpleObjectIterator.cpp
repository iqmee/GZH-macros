/*
**	Command & Conquer Generals Zero Hour(tm)
**	Copyright 2025 Electronic Arts Inc.
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

////////////////////////////////////////////////////////////////////////////////
//																																						//
//  (c) 2001-2003 Electronic Arts Inc.																				//
//																																						//
////////////////////////////////////////////////////////////////////////////////

// SimpleObjectIterator
// Contiguous inline-buffered object iterator with deterministic stable_sort.
// Original Author: Steven Johnson, September 2001
#include "PreRTS.h"	// This must go first in EVERY cpp file in the GameEngine

#include <algorithm>

#include "GameLogic/ObjectIter.h"

#include "Common/ThingTemplate.h"
#include "GameLogic/Object.h"

//=============================================================================
SimpleObjectIterator::SimpleObjectIterator()
	: m_clumpCount(0)
	, m_curIndex(0)
{
}

//=============================================================================
SimpleObjectIterator::~SimpleObjectIterator()
{
	makeEmpty();
}

//=============================================================================
void SimpleObjectIterator::insert(Object *obj, Real numeric)
{
	DEBUG_ASSERTCRASH(obj, ("sorry, no nulls allowed here"));

	ClumpEntry entry;
	entry.m_obj = obj;
	entry.m_numeric = numeric;

	if (m_clumpCount < INLINE_ENTRY_CAPACITY)
	{
		m_inlineEntries[m_clumpCount] = entry;
	}
	else if (m_clumpCount == INLINE_ENTRY_CAPACITY)
	{
		m_overflowEntries.reserve(INLINE_ENTRY_CAPACITY * 2);
		m_overflowEntries.assign(m_inlineEntries, m_inlineEntries + INLINE_ENTRY_CAPACITY);
		m_overflowEntries.push_back(entry);
	}
	else
	{
		m_overflowEntries.push_back(entry);
	}

	++m_clumpCount;
}

//=============================================================================
Object *SimpleObjectIterator::nextWithNumeric(Real *num)
{
	if (num)
		*num = 0.0f;

	if (m_curIndex < m_clumpCount)
	{
		const ClumpEntry &entry = entryData()[m_curIndex++];
		if (num)
			*num = entry.m_numeric;
		return entry.m_obj;
	}

	return nullptr;
}

//=============================================================================
void SimpleObjectIterator::reset()
{
	m_curIndex = 0;
}

//=============================================================================
void SimpleObjectIterator::makeEmpty()
{
	m_overflowEntries.clear();
	m_clumpCount = 0;
	m_curIndex = 0;
}

//=============================================================================
void SimpleObjectIterator::sort(IterOrderType order)
{
	if (m_clumpCount <= 1 || order == ITER_FASTEST)
	{
		reset();
		return;
	}

	ClumpEntry *data = entryData();
	ClumpEntry *end = data + m_clumpCount;

	switch (order)
	{
		case ITER_SORTED_NEAR_TO_FAR:
			std::stable_sort(data, end, compareNearToFar);
			break;
		case ITER_SORTED_FAR_TO_NEAR:
			std::stable_sort(data, end, compareFarToNear);
			break;
		case ITER_SORTED_CHEAP_TO_EXPENSIVE:
			std::stable_sort(data, end, compareCheapToExpensive);
			break;
		case ITER_SORTED_EXPENSIVE_TO_CHEAP:
			std::stable_sort(data, end, compareExpensiveToCheap);
			break;
		default:
			break;
	}

	// always reset after sorting, to prevent weirdness
	reset();
}

//-----------------------------------------------------------------------------
Bool SimpleObjectIterator::compareNearToFar(const ClumpEntry &a, const ClumpEntry &b)
{
	return a.m_numeric < b.m_numeric;
}

//-----------------------------------------------------------------------------
Bool SimpleObjectIterator::compareFarToNear(const ClumpEntry &a, const ClumpEntry &b)
{
	return a.m_numeric > b.m_numeric;
}

//-----------------------------------------------------------------------------
Bool SimpleObjectIterator::compareCheapToExpensive(const ClumpEntry &a, const ClumpEntry &b)
{
	return a.m_obj->getTemplate()->friend_getBuildCost() <
		b.m_obj->getTemplate()->friend_getBuildCost();
}

//-----------------------------------------------------------------------------
Bool SimpleObjectIterator::compareExpensiveToCheap(const ClumpEntry &a, const ClumpEntry &b)
{
	return a.m_obj->getTemplate()->friend_getBuildCost() >
		b.m_obj->getTemplate()->friend_getBuildCost();
}
