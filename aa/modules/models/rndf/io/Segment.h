#pragma once

#include <string>
#include <vector>
#include <map>
#include "Lane.h"

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

class Segment
{
public:
	Segment();
	Segment(const Segment & s);
	~Segment();

	Segment & operator=(const Segment & s);
	void addLane(uint laneId);

	uint               id;
	std::vector<Lane> lanes;
	std::string       name;
};

}

}

}

}

}

