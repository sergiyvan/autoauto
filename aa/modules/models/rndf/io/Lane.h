#pragma once

#include <vector>
#include <map>
#include "Waypoint.h"

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

class Lane
{
public:
	typedef double flt;
	Lane();
	Lane(const Lane & l);
	~Lane();
	Lane & operator=(const Lane & l);

	void addWaypoint(uint waypointID, flt latit, flt longit);

	uint 	segment;
	uint	id;

	std::vector<Waypoint>	waypoints;
	BoundaryType			left_boundary;
	BoundaryType			right_boundary;
	bool					zone_lane;
	bool					isHidden;
};

}

}

}

}

}

