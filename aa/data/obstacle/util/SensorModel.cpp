#include "SensorModel.h"



std::ostream & operator <<(std::ostream & lhs, const aa::data::obstacle::util::SensorModel & ob)
{
	lhs << sensorModelString(ob);
	return lhs;
}
RTT::Logger & operator <<(RTT::Logger & lhs, const aa::data::obstacle::util::SensorModel & ob)
{
	lhs << sensorModelString(ob);
	return lhs;
}

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

std::string sensorModelString(SensorModel model)
{
	switch (model) {
	case SENSOR_MODEL_ALASCA:
		return "SENSOR_MODEL_ALASCA";
		break;
	case SENSOR_MODEL_LUX:
		return "SENSOR_MODEL_LUX";
		break;
	case SENSOR_MODEL_VELODYNE:
		return "SENSOR_MODEL_VELODYNE";
		break;
	case SENSOR_MODEL_CAMERA:
		return "SENSOR_MODEL_CAMERA";
		break;
	case SENSOR_MODEL_TRW_FRONT:
		return "SENSOR_MODEL_TRW_FRONT";
		break;
	case SENSOR_MODEL_ULTRASONIC:
		return "SENSOR_MODEL_ULTRASONIC";
		break;
	case SENSOR_MODEL_TRW_REAR:
		return "SENSOR_MODEL_TRW_REAR";
		break;
	case SENSOR_MODEL_TRW_LEFT:
		return "SENSOR_MODEL_TRW_LEFT";
		break;
	case SENSOR_MODEL_TRW_RIGHT:
		return "SENSOR_MODEL_TRW_RIGHT";
		break;
	case SENSOR_MODEL_HELLA:
		return "SENSOR_MODEL_HELLA";
		break;
	case SENSOR_MODEL_SMS:
		return "SENSOR_MODEL_SMS";
		break;
	default:
		return "SENSOR_MODEL_UNKNOWN";
	}
}


}
}
}
}
