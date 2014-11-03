/*! \file SteerController.h
 *  \brief Contains module for SteeringController, reads current angle from sensor can, writes wanted torque to ADAM or EVA, shall be executed with 100 Hz
 *  \author Daniel Göhring
 */

#include "SteerController.h"
#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <fstream>
#include <math/AutoMath.h>




















REGISTERTASKCONTEXT2(SteerController, "SteerController");

using namespace ::math;
using namespace aa::modules::io::passat;

const flt SteerController::MAXSTEERINGWHEELANGLE = 530;		//for geometry calculations, max virtual wheel angle = 26.42166 deg for 535
const flt SteerController::MAXSTEERINGWHEELANGLESECURE = 500; // 500 //here no positive momentum is given if speed is positive
const flt SteerController::MAXSTEERINGWHEELANGLEVALID = 495; //495 //Here the wanted angle is cropped

const flt SteerController::WHEELBASE = 2.709;
const flt SteerController::MAXWHEELANGLE = 33.894;// std: 0.175 //veraltet 26.42166;


const flt SteerController::STANDSPEED = 0;	//meters per second
const flt SteerController::ROLLSPEED = 0.3;	//meters per second

const flt SteerController::STEERMOMENTUMATSTANDING = 10;	// percent times ten


SteerController::SteerController(std::string const & name)
	: util::RtTaskContext(name)
	, mWatchdogStatusIn("WatchdogStatusIn")
	, mSteerAssist3StatusIn("SteerAssist3StatusIn")
	, mSteerAssist3WheelSpeedIn("SteerAssist3WheelSpeedIn")
	, mSteeringWheelSpeedIn("SteeringWheelSpeedIn")
	, mWantedSteeringAngleIn("WantedSteeringAngleIn")
	, mNormalisedWantedSteeringAngleIn("NormalizedWantedSteeringAngleIn")
	, mWheelSpeedsIn("WheelSpeedsIn")

	, mDesiredMomentumToAnalogOut("DesiredMomentumToAnalogOut")
	, mNormalizedSteeringAngleOut("NormalizedSteeringAngleOut")

	, mKP("Kp", "P-Value for PID Controller", 0.2)	 // 0.2
	, mKI("Ki", "I-Value for PID Controller", 0.02) // 0.02
	, mKD("Kd", "D-Value for PID Controller", 0.01) // 0.01

	// smoother braindriver
//        , mKP("Kp", "P-Value for PID Controller", 0.1)	 // 0.2
//        , mKI("Ki", "I-Value for PID Controller", 0.005) // 0.02
//        , mKD("Kd", "D-Value for PID Controller", 0.0) // 0.01

	, mMaxSpeed("MaxSpeed", "MaxSpeed in deg per second", 90)
	, mMaxLateralAcceleration("MaxLatAcc", "Max Lateral Acceleration when turning in [m/s^2]", 3.0)  //changed from 5 on 24/01/2011
	, mLimitCentrifugalChangeValue("LimitCentrifugalChangeValue", "Limits the change of lateral acceleration, in [m/s^3]", 5.0)
	, mLimitCentrifugalChangeFlag("LimitCentrifugalChange", "Limit Lateral Acc. Change, yes or no", false)
	, mLimitSteerIntegral("LimitSteerIntegral", "Limits the Integral Part of the PID", 500.0)

	, mWantedSteerAngle("WantedSteerAngle", "Wanted Steer Angle in Deg", 0)
	, mCurrentSteerAngle("CurrentSteerAngle", "Current Steer Angle in Deg", 0)

	, mMomentum("Momentum", "Momentum, Output Value from PID Controller", 0)
	, mSteeringDelay("SteeringDelay", "Delay from SteerMotor to Steer reaction", 0.12) // 0.12

	, mOldError(0)
	, mNewError(0)
	, mIntegral(0)
	, mInTest(false)

	, mStartTest("StartTest", "Flag to start the impulse response test", false)
	, mTestMomentum("TestMomentum", "Momentum for impulse response test", 0.0)
	, mTestDuration("TestDuration", "Duration for the test in ms", 0)
	, mDriverOverrideSteer("DriverOverrideSteer", "Sets Mometum to zero, if true", false)

	, mSteerSpeedLimit("SteerSpeedLimit", "SteerSpeedLimit where momentum is limited in deg/s", 300)
	, mSteerSpeedInterval("SteerSpeedInterval" , "SteerSpeedInterval in which momentum in controlled in deg/s", 100)
	, mLimitSteerMomentumAtSpeed(0.0)


//	, mSpeedWritePort("SpeedPort")
{
	ports()->addPort(mWatchdogStatusIn);
	ports()->addPort(mSteerAssist3StatusIn);
	ports()->addPort(mSteerAssist3WheelSpeedIn);
	ports()->addPort(mSteeringWheelSpeedIn);
	ports()->addPort(mWantedSteeringAngleIn);
	ports()->addPort(mNormalisedWantedSteeringAngleIn);
	ports()->addPort(mWheelSpeedsIn);
	ports()->addPort(mDesiredMomentumToAnalogOut);
	ports()->addPort(mNormalizedSteeringAngleOut);


	addProperty(mKP);
	addProperty(mKI);
	addProperty(mKD);

	addProperty(mMaxSpeed);
	addProperty(mMaxLateralAcceleration);
	addProperty(mLimitCentrifugalChangeValue);
	addProperty(mLimitCentrifugalChangeFlag);
	addProperty(mLimitSteerIntegral);

	addProperty(mWantedSteerAngle);
	addProperty(mCurrentSteerAngle);

	addProperty(mMomentum);
	addProperty(mSteeringDelay);

	//for test impulse response
	addProperty(mStartTest);
	addProperty(mTestMomentum);
	addProperty(mTestDuration);
	addProperty(mDriverOverrideSteer);

	addProperty(mSteerSpeedLimit);
	addProperty(mSteerSpeedInterval);

}

SteerController::~SteerController()
{
}



bool SteerController::startHook()
{
	RTT::Logger::In in("SteerController");

	REQUIRED_PORT(mWatchdogStatusIn);
	REQUIRED_PORT(mSteerAssist3StatusIn);
	REQUIRED_PORT(mDesiredMomentumToAnalogOut);

	mTestBegin.stamp();
	accDerivLmt = 0;

	return true;
}


void SteerController::updateHook()
{
	RTT::Logger::In in("SteerController");

	assert(getPeriod() > 0.0);

	if (getPeriod() > 0.04) {
		std::cout << "WARNING SteerController is executed with less than 25 Hz";
	}

	now.stamp();
	TimedSteerAssist3Status sas;
	mSteerAssist3StatusIn.read(sas);


	mNormalizedSteeringAngleOut.write((-1.0)*sas.steerAngle() / SteerController::MAXSTEERINGWHEELANGLE);
	mCurrentSteerAngle.value() = sas.steerAngle();

	//Wanted: From Controller, Current: From Ports
	//Todo: Check, if Normalized or real Steering Angle was requested

	if (mWantedSteeringAngleIn.connected()) {
		mWantedSteeringAngleIn.read(mExecSteeringAngle);
	}
	else if (mNormalisedWantedSteeringAngleIn.connected()) {
		flt normalisedWantedSteeringAngle = 0.0;
		mNormalisedWantedSteeringAngleIn.read(normalisedWantedSteeringAngle);
		mExecSteeringAngle = ((-1.0) * normalisedWantedSteeringAngle * SteerController::MAXSTEERINGWHEELANGLE);
	}
	else {
		mExecSteeringAngle = 0;
	}

	//Debug
	//mExecSteeringAngle = mWantedSteerAngle;

	mDesiredMomentumToAnalogOut.write(getControlOutput(mExecSteeringAngle, mCurrentSteerAngle));




	//if true, no momentum to eva
	aa::modules::io::passat::TimedWatchdogStatus watchdogStatus;
	mWatchdogStatusIn.read(watchdogStatus);

	if ((mDriverOverrideSteer) || (watchdogStatus.watchdogState() != 12)) {  //10 is about 0.1 sec in 100 Hz mode
		//std::cout<<"STEERCONTROLLER DISABLED";
		mDesiredMomentumToAnalogOut.write(0.0);
	}


	//Driver Intervention
	//else {std::cout<<"STEERCONTROLLER ENABLED";}


	/***Test Impulse Response***/
//	mDesiredMomentumToAnalogOut.write((flt)getImpulseResponseOutput());
}

void SteerController::stopHook()
{

}


flt SteerController::getControlOutput(flt wantedSteerAngle, flt currentSteerAngle)
{
//flt absMaxDynamicMomentum;
	TimedWheelSpeeds wheelSpeeds;
	mWheelSpeedsIn.read(wheelSpeeds);
	flt mAvgFrontWheelSpeeds = (wheelSpeeds.speedFrontL() + wheelSpeeds.speedFrontR()) / 2.0;
	flt steerWheelSpeed = 0.0;
	mSteerAssist3WheelSpeedIn.read(steerWheelSpeed);

//	if (fabs(mAvgFrontWheelSpeeds > ROLLSPEED)) {
//		absMaxDynamicMomentum = STEERMOMENTUMATROLLING;
//	}
//	else {
//		absMaxDynamicMomentum =
//			STEERMOMENTUMATSTANDING + (mAvgFrontWheelSpeeds * (STEERMOMENTUMATROLLING - STEERMOMENTUMATSTANDING) /
//									   (ROLLSPEED - STANDSPEED));
//	}

	//steering range limiter - to protect wheel
	wantedSteerAngle = math::rangeCut(-SteerController::MAXSTEERINGWHEELANGLEVALID, wantedSteerAngle, SteerController::MAXSTEERINGWHEELANGLEVALID);

	//steering range limiter for given speed
	wantedSteerAngle = math::rangeCut(-getMaxSteeringWheelAngle(), wantedSteerAngle, getMaxSteeringWheelAngle());

	flt predictedSteerAngle = currentSteerAngle + mSteeringDelay * steerWheelSpeed * getPeriod() / 0.01;

	//Begin limit accDerivation
	//

	/*
	if (mLimitCentrifugalChangeFlag) {
		if ((fabs(mAvgFrontWheelSpeeds) == 0.0) || (mMaxLateralAcceleration == 0)) {
			accDerivLmt = wantedSteerAngle;   //if almost standing
		}
		else {
			if (wantedSteerAngle < (accDerivLmt - getPeriod() * mLimitCentrifugalChangeValue / mMaxLateralAcceleration * getMaxSteeringWheelAngle())) {
				accDerivLmt -= (getPeriod() * mLimitCentrifugalChangeValue / mMaxLateralAcceleration * getMaxSteeringWheelAngle());
			}
			else if (wantedSteerAngle > (accDerivLmt + getPeriod() * mLimitCentrifugalChangeValue / mMaxLateralAcceleration * getMaxSteeringWheelAngle())) {
				accDerivLmt += (getPeriod() * mLimitCentrifugalChangeValue / mMaxLateralAcceleration * getMaxSteeringWheelAngle());
			}
			else {
				accDerivLmt = wantedSteerAngle;
			}
		}

		wantedSteerAngle = accDerivLmt;

		//std::cout<<"In AccChangeFlag";
	}

	//End limit
	 */

	mNewError = wantedSteerAngle - predictedSteerAngle;
	mIntegral = math::rangeCut(flt(-1.0 * mLimitSteerIntegral), mIntegral += mNewError, flt(mLimitSteerIntegral));

	mMomentum = mKP * mNewError + mKI * getPeriod() / 0.01 * mIntegral + mKD * 0.01 / getPeriod() * (mNewError - mOldError);

//	if (mMaxSpeed < fabs(steerWheelSpeed)) {mMomentum = 0.0;}		//Check if slower speed is smooth and more precise

// 	mMomentum = math::rangeCut(-absMaxDynamicMomentum, (flt)mMomentum, absMaxDynamicMomentum); //Clip Momentum, standing to 7, rolling to 5


	mOldError = mNewError;


	// to avoid mechanical damage to the steering unit - do not steer into the steering limiter
	if ((mMomentum > 0) && (steerWheelSpeed >= 0) && ((currentSteerAngle > SteerController::MAXSTEERINGWHEELANGLESECURE))) {
		mMomentum.set(0);
		logWarning() << "Reached Steering Limit Left";
	}
	else if ((mMomentum < 0) && (steerWheelSpeed <= 0) && ((currentSteerAngle < (-1.0 * SteerController::MAXSTEERINGWHEELANGLESECURE)))) {
		mMomentum.set(0);
		logWarning() << "Reached Steering Limit Right";
	}

	mMomentum = math::rangeCut(-STEERMOMENTUMATSTANDING, mMomentum.get(), STEERMOMENTUMATSTANDING);

	///Daniel 04.02.2011

//        mLimitSteerMomentumAtSpeed = STEERMOMENTUMATSTANDING;
//        if (((steerWheelSpeed > mSteerSpeedLimit) && (wantedSteerAngle > currentSteerAngle)) ||
//            ((steerWheelSpeed < -mSteerSpeedLimit) && (wantedSteerAngle < currentSteerAngle))) {
//            if (fabs(steerWheelSpeed) > (mSteerSpeedLimit + mSteerSpeedInterval)) {
//                mMomentum = 0.0;
//                std::cout<<"Limiting Steer Speed to Zero";
//            }
//            else {
//                if (mMomentum > 0) {
//                    mMomentum =  STEERMOMENTUMATSTANDING - (steerWheelSpeed - mSteerSpeedLimit) / (mSteerSpeedInterval) * STEERMOMENTUMATSTANDING;
//                } else {
//                    mMomentum = -STEERMOMENTUMATSTANDING - (steerWheelSpeed - mSteerSpeedLimit) / (mSteerSpeedInterval) * STEERMOMENTUMATSTANDING;
//                }
//                std::cout<<"Limiting Steer Speed " << steerWheelSpeed << " to " << mMomentum;
//            }
//        }


	/////////////////////////////////////////////

//        std::cout << "Integral: " << mIntegral << ", WheelSpeed: " << steerWheelSpeed
//                  << ", Momentum: " << mMomentum << std::endl;

	return mMomentum;
}


flt SteerController::getMaxSteeringWheelAngle()
{
	TimedWheelSpeeds wheelSpeeds;
	mWheelSpeedsIn.read(wheelSpeeds);
	flt frontWheelSpeedAvg = (wheelSpeeds.speedFrontL() + wheelSpeeds.speedFrontR()) / 2;

	if ((fabs(frontWheelSpeedAvg) != 0.0) &&
			(fabs(WHEELBASE * mMaxLateralAcceleration / (frontWheelSpeedAvg * frontWheelSpeedAvg)) < 1)) {

		maxWheelAngleAtGivenSpeed = asin(WHEELBASE * mMaxLateralAcceleration / (frontWheelSpeedAvg * frontWheelSpeedAvg));

		//std::cout<<"MaxWheelAngle " << maxWheelAngleAtGivenSpeed << " WheelSpeed AVG " << frontWheelSpeedAvg;

		maxWheelAngleAtGivenSpeed = maxWheelAngleAtGivenSpeed * (180.0 / math::PI);

		return (maxWheelAngleAtGivenSpeed / MAXWHEELANGLE * MAXSTEERINGWHEELANGLE);
	}
	else {
		return MAXSTEERINGWHEELANGLE;
	}
}



