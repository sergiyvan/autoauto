#pragma once

#include <math/Types.h>

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

// Binary Type
enum SensorType {
	SENSOR_TYPE_UNKNOWN		= 0,
	SENSOR_TYPE_IBEO		= 1,
	SENSOR_TYPE_VELODYNE            = 2,
	SENSOR_TYPE_RADAR		= 4,
	SENSOR_TYPE_CAMERA		= 8,
	SENSOR_TYPE_ULTRASONIC          = 16,
	SENSOR_TYPE_RADAR_SMS           = 32,
	SENSOR_TYPE_ALL			= 63
};

std::string sensorTypeString(SensorType type);

}
}
}
}

