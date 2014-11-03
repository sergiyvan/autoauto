#pragma once

#include <util/RtTaskContext.h>
#include <core/TimedData.h>
#include <util/AutoPort.h>

#include <aa/modules/nav/controller/data/ControllerData.h>

#include <modules/models/egostate/EgoState.h>
#include <modules/models/carstate/CarState.h>
#include <aa/modules/models/carstate/PassatCarState.h>
#include <modules/models/carstate/AuxDevicesData.h>


typedef TimedData< PlainDataHolder< ::math::flt > > TimedDouble;

#ifdef DYNAMIC_PORTS
struct GatewaySpeedDesc;
struct GatewaySteerDesc;
#endif

namespace aa
{
namespace modules
{
namespace io
{
namespace controllergateway
{

class ControllerGateway
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;
	explicit ControllerGateway(std::string const & name);
	virtual ~ControllerGateway();

	virtual bool startHook();


	virtual void updateHook();
	virtual void stopHook();
	virtual void errorHook();


protected:

	/** @name InputPorts: */
	//@{
	RTT::InputPort< TimedEgoState > mEgoStateIn;
	RTT::InputPort< ::modules::models::carstate::TimedCarState > mCarStateIn;
    RTT::InputPort< aa::modules::models::carstate::TimedPassatCarState > mPassatCarStateIn;
	RTT::InputPort< aa::modules::nav::controller::data::TimedControllerData > mControllerDataIn;

	RTT::InputPort< bool > mControllerActivationRequestIn;
	RTT::InputPort< TimedDouble > mControllerSpeedIn;
	RTT::InputPort< flt > mControllerSteerIn;
	RTT::InputPort< int > mControllerGearIn;
	RTT::InputPort< AuxDevicesData > mControllerAuxDevicesIn;

	RTT::InputPort< bool > mJoystickActivationRequestIn;
	RTT::InputPort< TimedDouble > mJoystickSpeedIn;
	RTT::InputPort< flt > mJoystickSteerIn;
	RTT::InputPort< int > mJoystickGearIn;
	RTT::InputPort< AuxDevicesData > mJoystickAuxDevicesIn;

	RTT::InputPort< bool > mRemoteControlActivationRequestIn;
	RTT::InputPort< TimedDouble > mRemoteControlSpeedIn;
	RTT::InputPort< flt > mRemoteControlSteerIn;
	RTT::InputPort< int > mRemoteControlGearIn;
	RTT::InputPort< AuxDevicesData > mRemoteControlAuxDevicesIn;

	RTT::InputPort< bool > mEyeControlActivationRequestIn;
	RTT::InputPort< TimedDouble > mEyeControlSpeedIn;
	RTT::InputPort< flt > mEyeControlSteerIn;

	RTT::InputPort< bool > mBrainControlActivationRequestIn;
	RTT::InputPort< TimedDouble > mBrainControlSpeedIn;
	RTT::InputPort< flt > mBrainControlSteerIn;

	//@}

	/** @name OutputPorts: */
	//@{
	RTT::OutputPort< bool > mActivationRequestOut;
	RTT::OutputPort< TimedDouble > mSpeedOut;
	RTT::OutputPort< flt > mSteerOut;
	RTT::OutputPort< int > mGearOut;
	RTT::OutputPort< AuxDevicesData > mAuxDevicesOut;


	RTT::OutputPort< aa::modules::nav::controller::data::TimedControllerData > mControllerDataOut;
	//@}

	/** @name Properties: general properties */

	/** @name Attributes: general attributes */
	RTT::Attribute<int> mCounter;


	RTT::TaskContext * mStateMachine;

private:

	aa::modules::nav::controller::data::TimedControllerData mCurControllerData;
	TimedEgoState mCurEgoState;
	::modules::models::carstate::TimedCarState mCurCarState;
    aa::modules::models::carstate::TimedPassatCarState mCurPassatCarState;

	std::deque<aa::modules::nav::controller::data::TimedControllerData> mRecentControllerData;

	TimeStamp mLastUpdate;

	//statistics
	TimeStamp mAutonomousRunStart;
	int mNumMeasurements;
	flt mAutonomousRunAvgSpeed;	//in m/s
	flt mAutonomousRunDistance;	//in m


	/** Speed and Steer Control Mode commands */
	bool switchToManualMode();
	bool switchToControllerMode();
	bool switchToJoystickMode();
	bool switchToRemoteControlMode();
	bool switchToEyeControlMode();
	bool switchToBrainControlMode();


	bool printOutput() const;


	//helper function
	std::string toTimeString(flt s) const;

};

}
}
}
}
