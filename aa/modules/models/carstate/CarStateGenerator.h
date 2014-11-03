#pragma once

#include <util/RtTaskContext.h>
#include <rtt/Attribute.hpp>
#include <rtt/os/TimeService.hpp>
#include <QByteArray>
#include <QTextStream>
#include <QFile>

#include <modules/io/anagate/AnaGateMessages.h>
#include <aa/modules/io/passat/PassatCanMessages.h>

#include <modules/models/carstate/CarState.h>
#include "PassatCarState.h"

#include "StateOperator.h"

namespace aa
{
namespace modules
{
namespace models
{

namespace carstate
{

class CarStateGenerator
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;
	explicit CarStateGenerator(std::string const & name);
	virtual ~CarStateGenerator();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

protected:
	/** @name Attributes: */
	//@{
	/// Counts the number of successfully created CarStates
	RTT::Attribute<int> mPacketCount;
	//@}

	/** @name InputPorts: */
	//@{
	//from AnagateCanbus
	RTT::InputPort< TimedData< ::modules::io::anagate::CanMessage> > mCanMessageIn;

	//from EMCSerial
	RTT::InputPort<int> mActualHeadlightsIn;
	RTT::InputPort<int> mActualWipersIn;
	RTT::InputPort<int> mActualSirenIn;
	RTT::InputPort<int> mActualTurnSignalIn;
	RTT::InputPort<int> mActualShiftPosIn;

	//from passat
	RTT::InputPort< aa::modules::io::passat::TimedWatchdogStatus > mWatchdogStatusIn;
	RTT::InputPort< aa::modules::io::passat::TimedWatchdogBatteryStatus > mWatchdogBatteryStatusIn;
	RTT::InputPort< aa::modules::io::passat::TimedBrakeStatus1 > mBrakeStatus1In;
	RTT::InputPort< aa::modules::io::passat::TimedBrakeStatus2 > mBrakeStatus2In;
	RTT::InputPort< aa::modules::io::passat::TimedThrottleStatus > mThrottleStatusIn; //not received, use motor1status
	RTT::InputPort< aa::modules::io::passat::TimedGearStatus > mGearStatusIn;
	RTT::InputPort< aa::modules::io::passat::TimedSteerAssist3Status > mSteerAssist3StatusIn;
	RTT::InputPort< aa::modules::io::passat::TimedSignalWipersStatus > mSignalWipersStatusIn;
	RTT::InputPort< aa::modules::io::passat::TimedWheelSpeeds > mWheelSpeedsIn;
	RTT::InputPort< aa::modules::io::passat::TimedPathPulse > mPathPulseIn;
	RTT::InputPort< aa::modules::io::passat::TimedLight1Status > mLight1StatusIn;
	RTT::InputPort< aa::modules::io::passat::TimedMotor1Status > mMotor1StatusIn;
	RTT::InputPort< aa::modules::io::passat::TimedAbsEsp > mAbsEspIn;
	RTT::InputPort< aa::modules::io::passat::TimedYawAndBrakePressure > mYawAndBrakePressureIn;

	//@}

	/** @name OutputPorts: */
	//@{
	RTT::OutputPort< ::modules::models::carstate::TimedCarState > mCarStateOut;
	RTT::OutputPort< aa::modules::models::carstate::TimedPassatCarState > mPassatCarStateOut;
	//@}

	/** @name Attributes: */
	//@{
	//@}

	/** @name Properties: */
	//@{
	//@}

private:
	void processPassatCanMessage();
	void processEmcSerialData();

	TimedCanMessage mCurrentMsg;
	TimeStamp mLastEStopPacket;

	::modules::models::carstate::TimedCarState mCurCarState;
	aa::modules::models::carstate::TimedPassatCarState mCurPassatCarState;

};

}


}


}


}


