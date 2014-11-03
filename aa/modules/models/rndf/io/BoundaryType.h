#pragma once
#include <string>
#include <vector>

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

enum BoundaryType {
	LANE_BOUNDARY_BROKEN_WHITE = 0,
	LANE_BOUNDARY_SOLID_WHITE,
	LANE_BOUNDARY_SOLID_YELLOW,
	LANE_BOUNDARY_DOUBLE_YELLOW,
	LANE_BOUNDARY_NONE,
	LANE_BOUNDARY_CURB,
	LANE_BOUNDARY_BROKEN_YELLOW,
	LANE_BOUNDARY_MAX
};

static std::string const boundaryType_str[] = { "broken_white", "solid_white", "solid_yellow", "double_yellow", "no boundary", "curb", "broken_yellow" };

BoundaryType string2boundaryType(std::string const & s);

}
}
}
}
}
