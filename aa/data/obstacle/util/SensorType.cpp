#include "SensorType.h"

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

std::string sensorTypeString(SensorType type)
{
	std::stringstream s;

	switch (type) {
	case SENSOR_TYPE_IBEO:
		s << "Ibeo";
		break;
	case SENSOR_TYPE_RADAR:
		s << "Radar";
		break;
	default:
		s << "Unknown";
	}

	return s.str();
}


}
}
}
}
