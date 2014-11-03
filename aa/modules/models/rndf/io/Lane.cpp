#include "Lane.h"
#include "BoundaryType.h"
using namespace ::math;

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

Lane::Lane()
	:  segment(0)
	, id(0)
	, left_boundary(LANE_BOUNDARY_BROKEN_WHITE)
	, right_boundary(LANE_BOUNDARY_BROKEN_WHITE)
	, zone_lane(false)
	, isHidden(false)
{
}

Lane::~Lane()
{
	waypoints.clear();
}

Lane::Lane(const Lane & l)
{
	segment             = l.segment;
	id             = l.id;
	waypoints      = l.waypoints;
	left_boundary       = l.left_boundary;
	right_boundary      = l.right_boundary;
	zone_lane			= l.zone_lane;
	isHidden = l.isHidden;
}

Lane & Lane::operator=(const Lane & l)
{
	if (&l == this) {
		return *this;
	}

	segment             = l.segment;
	id             = l.id;
	waypoints      = l.waypoints;
	left_boundary       = l.left_boundary;
	right_boundary      = l.right_boundary;
	zone_lane			= l.zone_lane;
	isHidden = l.isHidden;
	return *this;
}


void Lane::addWaypoint(uint waypointID, flt latit, flt longit)
{
	if (waypoints.size() < waypointID) {
		waypoints.resize(waypointID);
	}

	Waypoint & wp = waypoints[waypointID - 1];

	wp.id = waypointID;
	wp.latitude    = latit;
	wp.longitude   = longit;
	wp.zoneLaneWP = zone_lane;
}

}

}

}

}

}

