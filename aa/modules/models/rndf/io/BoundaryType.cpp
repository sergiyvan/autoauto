#include "BoundaryType.h"

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

BoundaryType string2boundaryType(std::string const & s)
{
	unsigned int numberOfElements = sizeof(boundaryType_str) / sizeof(boundaryType_str[0]);

	for (unsigned int i = 0; i < numberOfElements; ++i) {
		if (s == boundaryType_str[i]) {
			return (BoundaryType) i;
		}
	}

	return LANE_BOUNDARY_NONE;
}

}

}

}

}

}


