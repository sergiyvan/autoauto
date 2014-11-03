#include "CarStateGenerator.h"

#include <rtt/Logger.hpp>
#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <QDataStream>
#include <QFile>
#include <iostream>
#include <boost/graph/graph_concepts.hpp>

namespace aa
{
namespace modules
{
namespace models
{
namespace carstate
{

REGISTERTASKCONTEXT(CarStateGenerator);

using namespace std;
using namespace aa::modules::io::passat;
using RTT::Logger;
using namespace ::math;
using ::modules::models::carstate::CarState;


/**
 *
 * \param name
 */
CarStateGenerator::CarStateGenerator(string const & name)
	: util::RtTaskContext(name)
	, mPacketCount("PacketCount", 0)

	//read ports (dodge)
	, mCanMessageIn("CanMessageIn")
	, mActualHeadlightsIn("ActualHeadlightsIn")
	, mActualWipersIn("ActualWipersIn")
	, mActualSirenIn("ActualSirenIn")
	, mActualTurnSignalIn("ActualTurnSignalIn")
	, mActualShiftPosIn("ActualShiftPosIn")

	//read ports (passat)
	, mWatchdogStatusIn("WatchdogStatusIn")
	, mWatchdogBatteryStatusIn("WatchdogBatteryStatusIn")
	, mBrakeStatus1In("BrakeStatus1In")
	, mBrakeStatus2In("BrakeStatus2In")
	, mThrottleStatusIn("ThrottleStatusIn")
	, mGearStatusIn("GearStatusIn")
	, mSteerAssist3StatusIn("SteerAssist3StatusIn")
	, mSignalWipersStatusIn("SignalWipersStatusIn")
	, mWheelSpeedsIn("WheelSpeedsIn")
	, mPathPulseIn("PathPulseIn")
	, mLight1StatusIn("Light1StatusIn")
	, mMotor1StatusIn("Motor1StatusIn")
	, mAbsEspIn("AbsEspIn")
	, mYawAndBrakePressureIn("YawAndBrakePressureIn")


	//write ports
	, mCarStateOut("CarStateOut")
	, mPassatCarStateOut("PassatCarStateOut")

	//members
{
	ports()->addPort(mCanMessageIn);
	ports()->addPort(mActualHeadlightsIn);
	ports()->addPort(mActualWipersIn);
	ports()->addPort(mActualSirenIn);
	ports()->addPort(mActualTurnSignalIn);
	ports()->addPort(mActualShiftPosIn);

	ports()->addPort(mWatchdogStatusIn);
	ports()->addPort(mWatchdogBatteryStatusIn);
	ports()->addPort(mBrakeStatus1In);
	ports()->addPort(mBrakeStatus2In);
	ports()->addPort(mThrottleStatusIn);
	ports()->addPort(mGearStatusIn);
	ports()->addPort(mSteerAssist3StatusIn);
	ports()->addPort(mSignalWipersStatusIn);
	ports()->addPort(mWheelSpeedsIn);
	ports()->addPort(mPathPulseIn);
	ports()->addPort(mLight1StatusIn);
	ports()->addPort(mMotor1StatusIn);

	ports()->addPort(mCarStateOut);
	ports()->addPort(mPassatCarStateOut);
	ports()->addPort(mAbsEspIn);
	ports()->addPort(mYawAndBrakePressureIn);

	addAttribute(mPacketCount);

}

CarStateGenerator::~CarStateGenerator()
{}

bool CarStateGenerator::startHook()
{
	Logger::In in("CarStateGenerator");

	OPTIONAL_PORT(mCarStateOut);


	//passat
	OPTIONAL_PORT(mPassatCarStateOut);

	if (mPassatCarStateOut.connected()) {
		logInfo() << "CarStateGenerator will be setup for Passat CarState";

		OPTIONAL_PORT(mWatchdogStatusIn);
		OPTIONAL_PORT(mWatchdogBatteryStatusIn);
		OPTIONAL_PORT(mBrakeStatus1In);
		OPTIONAL_PORT(mBrakeStatus2In);
		OPTIONAL_PORT(mThrottleStatusIn);
		OPTIONAL_PORT(mGearStatusIn);
		OPTIONAL_PORT(mSteerAssist3StatusIn);
		OPTIONAL_PORT(mSignalWipersStatusIn);
		OPTIONAL_PORT(mWheelSpeedsIn);
		OPTIONAL_PORT(mPathPulseIn);
		OPTIONAL_PORT(mLight1StatusIn);
		OPTIONAL_PORT(mMotor1StatusIn);
		OPTIONAL_PORT(mAbsEspIn);
		OPTIONAL_PORT(mYawAndBrakePressureIn);
	}

	return true;
}

void CarStateGenerator::updateHook()
{
	Logger::In in("CarStateGenerator");


	processPassatCanMessage();

	mCarStateOut.write(mCurCarState);
	mPassatCarStateOut.write(mCurPassatCarState);
	mPacketCount.set(mPacketCount.get() + 1);

}

void CarStateGenerator::stopHook()
{
}



void CarStateGenerator::processPassatCanMessage()
{
	if (mWatchdogStatusIn.connected()) {
		TimedWatchdogStatus data;

		if (RTT::NewData == mWatchdogStatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.watchdogStatus.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mWatchdogBatteryStatusIn.connected()) {
		TimedWatchdogBatteryStatus data;

		if (RTT::NewData == mWatchdogBatteryStatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.watchdogBatteryStatus.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mBrakeStatus1In.connected()) {
		TimedBrakeStatus1 data;

		if (RTT::NewData == mBrakeStatus1In.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.brakeStatus1.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mBrakeStatus2In.connected()) {
		TimedBrakeStatus2 data;

		if (RTT::NewData == mBrakeStatus2In.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.brakeStatus2.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mThrottleStatusIn.connected()) {
		TimedThrottleStatus data;

		if (RTT::NewData == mThrottleStatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.throttleStatus.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mGearStatusIn.connected()) {
		TimedGearStatus data;

		if (RTT::NewData == mGearStatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.gearStatus.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mSteerAssist3StatusIn.connected()) {
		TimedSteerAssist3Status data;

		if (RTT::NewData == mSteerAssist3StatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.steerAssist3Status.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mSignalWipersStatusIn.connected()) {
		TimedSignalWipersStatus data;

		if (RTT::NewData == mSignalWipersStatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.signalWipersStatus.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mWheelSpeedsIn.connected()) {
		TimedWheelSpeeds data;

		if (RTT::NewData == mWheelSpeedsIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.wheelSpeeds.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mPathPulseIn.connected()) {
		TimedPathPulse data;

		if (RTT::NewData == mPathPulseIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.pathPulse.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mLight1StatusIn.connected()) {
		TimedLight1Status data;

		if (RTT::NewData == mLight1StatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.light1Status.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mMotor1StatusIn.connected()) {
		TimedMotor1Status data;

		if (RTT::NewData == mMotor1StatusIn.read(data)) {
			data >> mCurCarState;
			data >> mCurPassatCarState;

            mCurPassatCarState.motor1Status.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mAbsEspIn.connected()) {
		TimedAbsEsp data;

		if (RTT::NewData == mAbsEspIn.read(data)) {
			data >> mCurPassatCarState;

            mCurPassatCarState.absEsp.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}

	if (mYawAndBrakePressureIn.connected()) {
		TimedYawAndBrakePressure data;

		if (RTT::NewData == mYawAndBrakePressureIn.read(data)) {
			data >> mCurPassatCarState;

            mCurPassatCarState.yawAndBrakePressure.adoptTimeStamp(data);
            mCurPassatCarState.adoptTimeStamp(data);
            mCurCarState.adoptTimeStamp(data);
        }
	}


//	std::cout << "1actualPressure: " << data.actualPressure() << std::endl;
//	std::cout << "2actualPressure: " << data.actualPressure() << std::endl;
//	std::cout << "2pressureDemand: " << data.pressureDemand() << std::endl;
//	std::cout << "gasPedal1: " << data.voltageGasPedal1() << std::endl;
//	std::cout << "gasPedal2: " << data.voltageGasPedal2() << std::endl;

	//merge brake pressure and throttle voltage to normalized speed value
	flt brakePressure = 0.0;
	flt throttleVoltage = 0.0;

	if (mBrakeStatus1In.connected()) {
		TimedBrakeStatus1 stat1;
		mBrakeStatus1In.read(stat1);
		brakePressure = stat1.actualPressure();
	}
	else if (mBrakeStatus2In.connected()) {
		TimedBrakeStatus2 stat2;
		mBrakeStatus2In.read(stat2);
		brakePressure = stat2.actualPressure();
	}

	if (mThrottleStatusIn.connected()) {
		TimedThrottleStatus stat;
		mThrottleStatusIn.read(stat);
//		throttleVoltage = stat.voltageGasPedal1();
		throttleVoltage = stat.voltageMotor1();
//		throttleVoltage = stat.voltageGasPedal2();
	}

	flt normSpeed = math::rangeCut<flt>(-1.0, throttleBrakeToNormalizedSpeed(throttleVoltage, brakePressure, 0.75, 4.0, 0.0, 127.5), 1.0);

	mCurCarState.gasPosition = normSpeed;
	mCurPassatCarState.gasPosition = normSpeed;
}


}
}
}
}
