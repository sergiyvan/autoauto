#pragma once

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

enum Location {
	LOCATION_UNKOWN		= 0,
	OFF_LANE			= 1,
	NEAR_LANE			= 2,
	SPARSELY_ON_LANE	= 3,
	LARGELY_ON_LANE		= 4,
	IN_ZONE				= 5,
	ON_PLAN				= 6,
	FIRST_ON_PLAN		= 7
};

}
}
}
}



