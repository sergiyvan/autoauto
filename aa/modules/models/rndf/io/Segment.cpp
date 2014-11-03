#include "Segment.h"

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

namespace io
{

Segment::Segment()
	: id(0)
{}

Segment::~Segment()
{}

Segment::Segment(const Segment & s)
{
	id      = s.id;
	lanes   = s.lanes;
	name    = s.name;
}

Segment & Segment::operator=(const Segment & s)
{
	if (&s == this) {
		return *this;
	}

	id      = s.id;
	lanes   = s.lanes;
	name    = s.name;

	return *this;
}

void Segment::addLane(uint laneID)
{
	if (lanes.size() < laneID) {
		lanes.resize(laneID);
	}

	Lane & l = lanes[laneID - 1];

	l.id = laneID;
	l.segment = id;
}

}

}

}

}

}

