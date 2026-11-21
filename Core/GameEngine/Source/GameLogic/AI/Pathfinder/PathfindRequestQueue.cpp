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
#include "Common/Xfer.h"
#include "GameLogic/Pathfinder/PathfindRequestQueue.h"

namespace
{
	constexpr size_t INITIAL_RING_CAPACITY = 2048;
	constexpr size_t INITIAL_IDSET_CAPACITY = 4096;
}

PathfindRequestQueue::PathfindRequestQueue()
	: m_ring(INITIAL_RING_CAPACITY, INVALID_ID)
	, m_idSet(INITIAL_IDSET_CAPACITY, INVALID_ID)
	, m_head(0)
	, m_tail(0)
	, m_count(0)
{
}

void PathfindRequestQueue::reset()
{
	m_head = 0;
	m_tail = 0;
	m_count = 0;
	std::fill(m_ring.begin(), m_ring.end(), INVALID_ID);
	std::fill(m_idSet.begin(), m_idSet.end(), INVALID_ID);
}

size_t PathfindRequestQueue::hashID(ObjectID id)
{
	UnsignedInt x = static_cast<UnsignedInt>(id);
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return static_cast<size_t>(x);
}

Bool PathfindRequestQueue::contains(ObjectID id) const
{
	if (id == INVALID_ID || m_count == 0)
	{
		return false;
	}

	const size_t mask = m_idSet.size() - 1;
	size_t pos = hashID(id) & mask;
	while (m_idSet[pos] != INVALID_ID)
	{
		if (m_idSet[pos] == id)
		{
			return true;
		}
		pos = (pos + 1) & mask;
	}
	return false;
}

Bool PathfindRequestQueue::enqueue(ObjectID id)
{
	if (id == INVALID_ID)
	{
		return false;
	}

	if (contains(id))
	{
		return true;
	}

	if (m_count >= m_ring.size())
	{
		growRing();
	}

	if ((m_count + 1) * 2 > m_idSet.size())
	{
		growIdSet();
	}

	const size_t mask = m_ring.size() - 1;
	m_ring[m_tail] = id;
	m_tail = (m_tail + 1) & mask;
	++m_count;
	insertID(id);
	return true;
}

Bool PathfindRequestQueue::dequeue(ObjectID &outID)
{
	if (m_count == 0)
	{
		outID = INVALID_ID;
		return false;
	}

	const size_t mask = m_ring.size() - 1;
	outID = m_ring[m_head];
	m_ring[m_head] = INVALID_ID;
	m_head = (m_head + 1) & mask;
	--m_count;
	eraseID(outID);
	return true;
}

void PathfindRequestQueue::crc(Xfer *xfer)
{
	UnsignedInt count = static_cast<UnsignedInt>(m_count);
	xfer->xferUnsignedInt(&count);

	const size_t mask = m_ring.size() - 1;
	for (size_t i = 0; i < m_count; ++i)
	{
		ObjectID id = m_ring[(m_head + i) & mask];
		xfer->xferObjectID(&id);
	}
}

void PathfindRequestQueue::growRing()
{
	const size_t oldCap = m_ring.size();
	const size_t oldMask = oldCap - 1;
	std::vector<ObjectID> newRing(oldCap * 2, INVALID_ID);

	for (size_t i = 0; i < m_count; ++i)
	{
		newRing[i] = m_ring[(m_head + i) & oldMask];
	}

	m_ring.swap(newRing);
	m_head = 0;
	m_tail = m_count;
}

void PathfindRequestQueue::growIdSet()
{
	std::vector<ObjectID> oldSet(m_idSet.size() * 2, INVALID_ID);
	m_idSet.swap(oldSet);

	for (size_t i = 0; i < oldSet.size(); ++i)
	{
		if (oldSet[i] != INVALID_ID)
		{
			insertID(oldSet[i]);
		}
	}
}

void PathfindRequestQueue::insertID(ObjectID id)
{
	const size_t mask = m_idSet.size() - 1;
	size_t pos = hashID(id) & mask;
	while (m_idSet[pos] != INVALID_ID)
	{
		if (m_idSet[pos] == id)
		{
			return;
		}
		pos = (pos + 1) & mask;
	}
	m_idSet[pos] = id;
}

void PathfindRequestQueue::eraseID(ObjectID id)
{
	const size_t mask = m_idSet.size() - 1;
	size_t pos = hashID(id) & mask;
	while (m_idSet[pos] != INVALID_ID)
	{
		if (m_idSet[pos] == id)
		{
			// Backward-shift deletion avoids tombstones and maintains O(1) probe lengths indefinitely.
			size_t hole = pos;
			size_t next = (hole + 1) & mask;
			while (m_idSet[next] != INVALID_ID)
			{
				const size_t ideal = hashID(m_idSet[next]) & mask;
				if (((next - ideal) & mask) >= ((next - hole) & mask))
				{
					m_idSet[hole] = m_idSet[next];
					hole = next;
				}
				next = (next + 1) & mask;
			}
			m_idSet[hole] = INVALID_ID;
			return;
		}
		pos = (pos + 1) & mask;
	}
}
