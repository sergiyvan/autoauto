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

Waypoint::Waypoint()
	: id(0)
	, latitude(0.0)
	, longitude(0.0)
	, checkpoint_id(0)
	, stopsign(false)
	, trafficLight(false)
	, trafficLightTwoPhases(false)
	, giveWay(false)
	, isPerimeterPoint(false)
	, isSpot(false)
	, zoneLaneWP(false)
	, decisionPoint(false)
	, laneWidth(8)
	, vertex_is_set(false)
{}

Waypoint::~Waypoint()
{}

Waypoint::Waypoint(const Waypoint & w)
	: id(w.id)
	, latitude(w.latitude)
	, longitude(w.longitude)
	, checkpoint_id(w.checkpoint_id)
	, stopsign(w.stopsign)
	, trafficLight(w.trafficLight)
	, giveWay(w.giveWay)
	, trafficLightTwoPhases(w.trafficLightTwoPhases)
	, decisionPoint(w.decisionPoint)
	, isPerimeterPoint(w.isPerimeterPoint)
	, isSpot(w.isSpot)
	, zoneLaneWP(w.zoneLaneWP)
	, laneWidth(w.laneWidth)
	, exit_to(w.exit_to)
	, vertex(w.vertex)
	, vertex_is_set(w.vertex_is_set)
{
}

Waypoint & Waypoint::operator=(const Waypoint & w)
{
	if (&w == this) {
		return *this;
	}

	id                      = w.id;
	latitude		= w.latitude;
	longitude		= w.longitude;
	checkpoint_id           = w.checkpoint_id;
	stopsign		= w.stopsign;
	trafficLight		= w.trafficLight;
	trafficLightTwoPhases	= w.trafficLightTwoPhases;
	giveWay                 = w.giveWay;
	decisionPoint           = w.decisionPoint;
	isPerimeterPoint	= w.isPerimeterPoint;
	isSpot			= w.isSpot;
	zoneLaneWP		= w.zoneLaneWP;
	laneWidth               = w.laneWidth;
	exit_to			= w.exit_to;
	vertex			= w.vertex;
	vertex_is_set           = w.vertex_is_set;

	return *this;
}

void Waypoint::addExit(uint segment, uint lane, uint waypoint)
{
	WaypointID wpid;

	wpid.segment  = segment;
	wpid.lane     = lane;
	wpid.waypoint = waypoint;
	exit_to.push_back(wpid);
}

}


}


}


}


}


