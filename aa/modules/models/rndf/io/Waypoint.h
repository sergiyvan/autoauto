#pragma once
#include <aa/modules/models/rndf/RndfGraph.h>	// vertex_descr
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

struct WaypointID {
	unsigned int segment;
	unsigned int lane;
	unsigned int waypoint;
};

class Waypoint
{
public:
	typedef double flt;
	Waypoint();
	Waypoint(const Waypoint & w);
	~Waypoint();

	Waypoint & operator=(const Waypoint & w);
	void addExit(unsigned int segment, unsigned int lane, unsigned int waypoint);

	unsigned int	id;
	flt	latitude;
	flt	longitude;
	/* Optional information */
	unsigned int	checkpoint_id;
	bool	stopsign;
	bool    trafficLight;
	bool    trafficLightTwoPhases;
	bool	isPerimeterPoint;
	bool	isSpot;
	bool	zoneLaneWP;
	bool    decisionPoint;
	bool    giveWay;
	flt   laneWidth;

	std::vector<WaypointID> exit_to;

	bool vertex_is_set;
	vertex_descr vertex;
};

}

}

}

}

}

