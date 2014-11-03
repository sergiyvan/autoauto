#pragma once
#include <string>
#include <vector>
#include "Spot.h"
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

class Zone
{
public:
	Zone();
	Zone(const Zone & z);
	~Zone();

	uint					id;
	std::vector<Spot>	spots;
	std::string			name;
	std::vector<Waypoint>	perimeterpoints;
};

}

}

}

}

}

