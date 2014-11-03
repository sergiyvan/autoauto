#pragma once
#include "AbstractController.h"
#include <core/TimedData.h>
#include <iostream>
#include <deque>

#include <aa/modules/io/passat/PassatCanMessages.h>

#include <modules/models/egostate/EgoState.h>
#include <modules/models/carstate/CarState.h>

#include "Plan.h"
#include "data/ControllerData.h"


/**
 * This controller, derived by Abstract Controller, implement methods to follow the planned trajectory,
 * therefore calculating a steer angle (controlSteering)
 * and generating a control output for throttle and brake (controlThrottleBrake)
 * and setting the desired gear
 *
 * \author Daniel Göhring
 *
 */


namespace aa
{
namespace modules
{
namespace nav
{

namespace controller
{

class ControllerPassat
	: public AbstractController
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	explicit ControllerPassat(std::string const & name);
	virtual ~ControllerPassat();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	int count;

protected:
	flt mSteer;
	flt mSpeed;
	RTT::Property<flt> mLimitThrottleBrakeIntegral;

	int mGear;
	int mGearSet;
	int mGearWish;

	RTT::Property<flt> mPhaseDur;
	RTT::Property<flt> mWheelAmplitude;

	RTT::Property<flt> mKpa;
	RTT::Property<flt> mKia;
	RTT::Property<flt> mKda;

	RTT::Property<flt> mKps;
	RTT::Property<flt> mKis;
	RTT::Property<flt> mKds;
	flt mDynamicHandbrake;
	RTT::Property<flt> mThrottleOutputScale;

	RTT::Property<flt> mKpSteer;
	RTT::Property<flt> mKiSteer;
	RTT::Property<flt> mKdSteer;
	RTT::Property<flt> mKpSteerLowSpeedValue;
	RTT::Property<flt> mKpSteerHighSpeedValue;
	RTT::Property<flt> mKiSteerLowSpeedValue;
	RTT::Property<flt> mKiSteerHighSpeedValue;
	RTT::Property<flt> mKdSteerLowSpeedValue;
	RTT::Property<flt> mKdSteerHighSpeedValue;
	RTT::Property<bool> mUseSimplePidSetForSteer;

	RTT::Property<flt> mLimitSteerIntegral;

	RTT::Property<flt> mSoftBrake;
	flt mMinBrake;
	flt mMaxThrottle;
	RTT::Property<flt> mMinBrakeComfort;
	RTT::Property<flt> mMaxThrottleComfort;
	RTT::Property<flt> mMinBrakeSporty;
	RTT::Property<flt> mMaxThrottleSporty;
	RTT::Property<flt> mMinBrakeUltimate;
	RTT::Property<flt> mMaxThrottleUltimate;
	RTT::Property<flt> mStartRollingThrottleValue;
	RTT::Property<flt> mFullThrottleWheelSpeed;
	RTT::Property<flt> mFullBrakeWheelSpeed;

	RTT::Property<flt> mMaxAbsSteer;
	RTT::Property<flt> mBrakeAcceleration;
	RTT::Property<flt> mMaxCurveAcceleration;
	RTT::Property<flt> mTargetSpeed;

	RTT::Property<flt> mSteerAmplifyer;
	RTT::Property<flt> mPreviewDistance;
	RTT::Property<flt> mStaticPreviewSpeed;
	RTT::Property<flt> mDynamicPreviewSpeedScale;
	RTT::Property<flt> mPredictionTimeCarModelForSteer;

	//Obstacle Avoidance
	RTT::Property<flt> mObstacleDistToLeft;
	RTT::Property<flt> mObstacleDistToRight;
	RTT::Property<flt> mMinObstacleDist;
	RTT::Property<flt> mMaxObstacleAvoidanceDistance;
	RTT::Property<bool> mAvoidSideObstacles;

	RTT::Property<bool> mOverrideFlag;
	RTT::Property<bool> mLimitSteer;
	RTT::Property<flt> mTolerance;
	RTT::Property<bool> mLogDataToFile;
	RTT::Property<flt> mMaxLateralAcceleration;
	RTT::Property<flt> mOverrideValue;
	RTT::Property<bool> mDout;

	RTT::Property<flt> mCarDirAngleOffsetDeg;
	RTT::Property<bool> mUseFrontWheelsForSteerController;
	RTT::Property<bool> mIMUOnlineCalibrate;

	//EgoState Jumps
	RTT::Property<bool> mAvoidEgoStateJumps;
	RTT::Property<flt> mAvoidEgoStateJumpsUpperThreshold;
	RTT::Property<flt> mAvoidEgoStateJumpsLowerThreshold;
	RTT::Property<flt> mAvoidEgoStateJumpsTimeToInterpolate;
	flt mCurrentTimeToInterpolate;
	TimedEgoState mPredictedEgoState;
	TimedEgoState mLastEgoState;
    Vec3 mAccumulatedOffset;
    Vec3 mDegradeSubtrahendOfAccumulatedOffset;





	TimeStamp now;

	//Learning Properties Start
	RTT::Property<bool> mLearnHumanControllerParameters;
	RTT::Property<bool> mApplyHumanControllerParameters;
	//Learning Properties End

	//NN Properties Start
	RTT::Property<bool> mLearnNNSteering;
	RTT::Property<bool> mApplyNNSteering;
	//NN Properties End

	//STEP 6: get a steering position for wanted speed
	/** calculate a steering position
	* @return a value between -1 and 1 indicating steering position */
	flt controlSteering(Plan_ptr plan);

private:

	///required methods by parent AbstractController


	//STEP 4: get gear position
	/** return wanted gear position
	* @return a value conforming to 1=P, 2=R, 4=N, 8=D (reserved: -1=error and 0=between gear) */
	int controlGear();


	//STEP 5: get a throttle position for wanted speed
	/** calculate a throttle position given current and wanted speed (uses either linear, pid controller)
	* @param curSpeed current speed of the car in m/s
	* @param wantedSpeed desired speed of the car in m/s
	* @return a value between -1 and 1 indicating gas/brake pedal position */
	flt controlThrottleBrake(flt curSpeed, flt wantedSpeed);




	flt calculateSideObstaclesAvoidance();

	/**Routine for online calibration of IMU */
	flt onlineCalibrateIMU();

	//Learning Part Methods Start
	/** initialize parameters */
	void initializeHumanControllerLearning();

	/** learn human driving parameters */
	void learnHumanControllerParameters();

	/** create learned human driving parameters for controller*/
	void applyHumanControllerParameters();
	//Learning Part Methods End


	//NN Learning Part Methods Starts
	void initializeNNSteering();
    void learnNNSteering(Vec2 p0, Vec2 p1, Vec2 p2, Vec3 carPos, Vec3 carDirVec, flt steerAction);
	void activateNNSteering();
	void backPropagateNNSteering();
	flt applyNNSteering();
	void normalizePointsToCarPosAndDir();
	//NN Learning Part Methods Ends

	//STEP 7: get auxiliary devices data (headlights, siren, wiper, turninglights)
	/** retrieve data for auxiliary devices
	 * @return a auxiliary device data */
	AuxDevicesData controlAuxDevices(Plan_ptr plan);



	void engageForwardOrBackwardDrive(int gearPosition);
	void disengageGear();

	flt getControlOutput(flt wantedSpeed, flt currentSpeed);
	flt steerPIDcontroller(flt wantedSteerAngle, flt currentSteerAngle);

	void normalizeAngle(flt & angle);

	bool mInForwardOrBackwardDriveEngaging;

	aa::modules::nav::controller::data::TimedControllerData curControllerData;

	std::vector< std::pair < flt, flt > > mTestScenario;

	flt mIntegral;
	flt mOldError;
	flt mNewError;
	flt mOutput;

	flt mSteerIntegral;
	flt mOldSteerError;
	flt mNewSteerError;
	flt mSteerOutput;

	flt mLastSteerAngle;
	flt mPreLastSteerAngle;
	flt mGearDorRSelectedTime;
	flt mDistanceToTrajectory;
	flt mDifferenceAngleBearingHeading;
	flt mBrakingDownCurrently;

	flt mKpSteerInterpolated;
	flt mKiSteerInterpolated;
	flt mKdSteerInterpolated;

	flt mLowSpeedValue;
	flt mHighSpeedValue;
	flt mVelocitySmoothedThrottle;
	flt mVelocitySmoothedBrake;

	bool mShiftBackward;

	FILE * out;
	std::ofstream os;
	long mPackethistory[100];
	int mPackethistoryIdx;
	TimeStamp mZeroTime;
	long ts;
	long logts;
	std::string mFileName;
	flt mFreq;
	flt mLogDataCounter;

	flt mCyclesFlashLightsOn;
	flt mLateralErrorSign;
	flt mSignedLateralError;
	flt mIMUCyclesOnlineCalibration;
	flt mIMUErrorIntegral;

	//Learning Simple Variables Start
	bool mInitializedLearningParams;
	long mThrottleHistogram[100];
	flt mLearnedMaxThrottle;
	long mStartRollingThrottleHistogram[100];
	flt mLearnedStartRollingThrottle;
	long mBrakeHistogram[100];
	flt mLearnedMinBrake;
	long mCentrifugalForceHistogram[100];
	flt mLearnedCentrifugalForce;
	long mVelocityHistogram[100];
	flt mLearnedVelocity;
	//Learning Simple Variables End


	//NN Learning Part Variables Starts
	flt perceptronLayer1[1];
	flt weights[3][1]; //input: planPoints, normalized to front of vehicle, egocentric position
	flt mInitializeNN;
	//NN Learning Part Variables Ends

};

}

}

}

}

