#pragma once
#include <boost/array.hpp>
#include "Waypoint.h"
#include <aa/modules/models/rndf/RndfGraph.h>
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

class Spot
{
public:
	typedef ::math::flt flt;
	Spot();
	Spot(const Spot & s);
	~Spot();

	Spot & operator=(const Spot & s);

	uint id;
	uint zone;
	flt width;      //given in ft in rndf
	boost::array<Waypoint, 2u>	waypoints;

	vertex_descr vertex;
};

}

}

}

}

}

