#include "Spot.h"
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

Spot::Spot()
	: id(0)
	, zone(0)
	, width(0)
{}

Spot::~Spot()
{}

Spot::Spot(const Spot & s)
	: id(s.id)
	, zone(s.zone)
	, width(s.width)
	, waypoints(s.waypoints)
{}

Spot & Spot::operator=(const Spot & s)
{
	if (&s == this) {
		return *this;
	}

	id    = s.id;
	zone = s.zone;
	width = s.width;
	waypoints  = s.waypoints;

	return *this;
}

}

}

}

}

}

