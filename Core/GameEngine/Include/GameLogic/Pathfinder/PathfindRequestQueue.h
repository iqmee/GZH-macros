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
#include "Common/GameType.h"

class Xfer;

/**
 * Unbounded, deterministic FIFO request queue for pathfinding with O(1) duplicate suppression.
 * Replaces the fixed 512-slot circular array and O(N) linear duplicate scan so 1,000+ units
 * can request paths simultaneously without dropping requests or stalling the CPU.
 */
class PathfindRequestQueue
{
public:
	PathfindRequestQueue();

	void reset();

	Bool empty() const { return m_count == 0; }
	size_t size() const { return m_count; }
	size_t capacity() const { return m_ring.size(); }
	ObjectID peekAt(size_t index) const
	{
		return (index < m_count) ? m_ring[(m_head + index) & (m_ring.size() - 1)] : INVALID_ID;
	}

	Bool contains(ObjectID id) const;
	Bool enqueue(ObjectID id);
	Bool dequeue(ObjectID &outID);

	void crc(Xfer *xfer);

private:
	static size_t hashID(ObjectID id);

	void growRing();
	void growIdSet();
	void insertID(ObjectID id);
	void eraseID(ObjectID id);

	std::vector<ObjectID> m_ring;
	std::vector<ObjectID> m_idSet;
	size_t m_head;
	size_t m_tail;
	size_t m_count;
};
