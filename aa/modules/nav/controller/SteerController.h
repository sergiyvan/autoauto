/*! \file SteerController.h
 *  \brief Contains module for SteeringController, reads current angle from sensor can, writes wanted torque to ADAM or EVA
 *  \author Daniel Göhring
 */

#pragma once
#include <util/RtTaskContext.h>
#include <core/TimedData.h>
#include <iostream>
#include <fstream>

#include <deque>
#include <aa/modules/io/passat/PassatCanMessages.h>
#include <math/Types.h>


class SteerController
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;

	explicit SteerController(std::string const & name);
	virtual ~SteerController();

	virtual bool startHook();
	virtual void updateHook();


	virtual void stopHook();

	// for impulse response
	TimeStamp now, mTestBegin;
	bool mInTest;
	flt accDerivLmt;		//limits the steering wheel angle change over time, depending on speed

//	int count;



protected:
	/** @name OutputPorts: */
	//@{
	RTT::OutputPort<flt> mDesiredMomentumToAnalogOut; // from expected -10 to 10, sent to steering controller
	RTT::OutputPort<flt> mNormalizedSteeringAngleOut; // from -1 to 1, sent to steering controller


	//@}

	/** @name InputPorts: */
	//@{
	RTT::InputPort<aa::modules::io::passat::TimedWatchdogStatus > mWatchdogStatusIn; //watchdog package from controller can
	RTT::InputPort<aa::modules::io::passat::TimedSteerAssist3Status > mSteerAssist3StatusIn; //steering angle from sensor can
	RTT::InputPort<flt> mSteerAssist3WheelSpeedIn; //steering speed from sensor can
	RTT::InputPort<aa::modules::io::passat::TimedSteeringWheelSpeed > mSteeringWheelSpeedIn; //steering wheel speed from sensor can
	RTT::InputPort<flt> mWantedSteeringAngleIn;										// wanted steering angle position
	RTT::InputPort<flt> mNormalisedWantedSteeringAngleIn;								// normalized wanted steering angle
	RTT::InputPort<aa::modules::io::passat::TimedWheelSpeeds> mWheelSpeedsIn;				// velocities of the wheels
	//@}


	RTT::Property<flt> mKP;
	RTT::Property<flt> mKI;
	RTT::Property<flt> mKD;

	RTT::Property<flt> mWantedSteerAngle;
	RTT::Property<flt> mCurrentSteerAngle;

	RTT::Property<flt> mMomentum;
	RTT::Property<flt> mSteeringDelay;

	RTT::Property<flt> mMaxSpeed;
	RTT::Property<flt> mMaxLateralAcceleration;

	RTT::Property<flt> mSteerSpeedLimit;
	RTT::Property<flt> mSteerSpeedInterval;
	flt mLimitSteerMomentumAtSpeed;

	RTT::Property<flt> mLimitCentrifugalChangeValue;  // = scalar in [mMaxLateralAcceleration] per second
	RTT::Property<bool> mLimitCentrifugalChangeFlag;
	RTT::Property<flt> mLimitSteerIntegral;


	// properties for impulse Response
	RTT::Property<bool> mStartTest;
	RTT::Property<flt> mTestMomentum;
	RTT::Property<int> mTestDuration; //in ms
	RTT::Property<bool> mDriverOverrideSteer; //in ms




	flt getControlOutput(flt mWantedSteerAngle, flt mCurrentSteerAngle);
	flt getMaxSteeringWheelAngle();



private:

	flt mOldError;
	flt mNewError;
	flt mIntegral;

	static const flt MAXSTEERINGWHEELANGLEVALID;
	static const flt MAXSTEERINGWHEELANGLESECURE;
	static const flt MAXSTEERINGWHEELANGLE;

	static const flt STANDSPEED;
	static const flt ROLLSPEED;

	static const flt STEERMOMENTUMATSTANDING;

	static const flt WHEELBASE;
	static const flt MAXWHEELANGLE;

	flt mExecSteeringAngle;

	flt maxWheelAngleAtGivenSpeed;


	std::ofstream os;


};
