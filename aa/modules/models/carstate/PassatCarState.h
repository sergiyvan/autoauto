#pragma once

#include <modules/models/carstate/CarState.h>
#include <aa/modules/io/passat/PassatCanMessages.h>
#include <math/Types.h>
#include <core/TimedData.h>
#include <util/Ports.h>

namespace aa
{
namespace modules
{
namespace models
{
namespace carstate
{

struct PassatCarState
		: public ::modules::models::carstate::CarState {
	typedef ::math::flt flt;

	PassatCarState()
		: desiredSteerMomentum(0)
	{}

	io::passat::TimedWatchdogStatus watchdogStatus;
	io::passat::TimedWatchdogBatteryStatus watchdogBatteryStatus;
	io::passat::TimedBrakeStatus1 brakeStatus1;
	io::passat::TimedBrakeStatus2 brakeStatus2;
	io::passat::TimedThrottleStatus throttleStatus;
	io::passat::TimedGearStatus gearStatus;
	io::passat::TimedSteerAssist3Status steerAssist3Status;
	io::passat::TimedSignalWipersStatus signalWipersStatus;
	io::passat::TimedWheelSpeeds wheelSpeeds;
	io::passat::TimedPathPulse pathPulse;
	io::passat::TimedLight1Status light1Status;
	io::passat::TimedMotor1Status motor1Status;
	io::passat::TimedAbsEsp absEsp;
	io::passat::TimedYawAndBrakePressure yawAndBrakePressure;

	flt desiredSteerMomentum;
};

typedef TimedData<PassatCarState> TimedPassatCarState;

}
}
}
}



namespace RTT
{
extern template class InputPort<aa::modules::models::carstate::TimedPassatCarState>;
extern template class OutputPort<aa::modules::models::carstate::TimedPassatCarState>;
}

#include <modules/io/logger/LogPort.h>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
namespace serialization
{
template<class Archive>
void serialize(Archive & ar,aa::modules::models::carstate::TimedPassatCarState & o, const unsigned int version)
{
    if (version == 0) {
        int a = (int)o.lastMessageId;
        float f;
        ar & make_nvp("lastMessageId", a);

        //drive messages
        ar & make_nvp("gasPositionInput", o.gasPositionInput);
        ar & make_nvp("gasPosition", o.gasPosition);
        ar & make_nvp("wheelPositionInput", o.wheelPositionInput);
        ar & make_nvp("wheelPosition", o.wheelPosition);

        ar & make_nvp("isPBrakeRequest", a);
        ar & make_nvp("isClutchRequest", a);
        ar & make_nvp("isEvalConnected", a);
        ar & make_nvp("isEvalActive", a);
        ar & make_nvp("isValidCoilPulse", a);
        ar & make_nvp("brakelightActive", a);
        ar & make_nvp("parkinglightActive", a);
        ar & make_nvp("steeringServoEngaged", a);
        ar & make_nvp("remoteOffSwitchActive", a);
        ar & make_nvp("ignitionActive", a);
        ar & make_nvp("speed", f);
        ar & make_nvp("throttleClutchEngaged", a);
        ar & make_nvp("bat1_voltage", f);
        ar & make_nvp("bat2_voltage", f);


        //icenter messages
        ar & make_nvp("isInTestmode", a);
        ar & make_nvp("isReqSteeringSide1", a);
        ar & make_nvp("isReqSteeringSide2", a);
        ar & make_nvp("isReqGasBrake1", a);
        ar & make_nvp("isReqGasBrake2", a);


        //error messages
        ar & make_nvp("eStopActive", a);
        //drive messages
        ar & make_nvp("watchdogStatus", o.watchdogStatus);
        ar & make_nvp("watchdogBatteryStatus", o.watchdogBatteryStatus);
        ar & make_nvp("brakeStatus1", o.brakeStatus1);
        ar & make_nvp("brakeStatus2", o.brakeStatus2);
        ar & make_nvp("throttleStatus", o.throttleStatus);
        ar & make_nvp("gearStatus", o.gearStatus);
        ar & make_nvp("steerAssist3Status", o.steerAssist3Status);
        ar & make_nvp("signalWipersStatus", o.signalWipersStatus);
        ar & make_nvp("steerAssist3Status", o.wheelSpeeds);
        ar & make_nvp("signalWipersStatus", o.pathPulse);
        ar & make_nvp("signalWipersStatus", o.light1Status);
        ar & make_nvp("signalWipersStatus", o.motor1Status);
        ar & make_nvp("signalWipersStatus", o.absEsp);
        ar & make_nvp("signalWipersStatus", o.yawAndBrakePressure);
        ar & make_nvp("signalWipersStatus", o.desiredSteerMomentum);

    }

}

} // namespace serialization
} // namespace boost

//version
BOOST_CLASS_VERSION(aa::modules::models::carstate::TimedPassatCarState, 1)
