#include "ControllerPassat.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <math/AutoMath.h>
#include <math/PathSpline.h>
#include <math/DiffGeom.h>
#include <data/VehicleData.h>

#include <fstream>

#include <aa/modules/nav/statemachine/StateMachine.h>
#include <modules/models/egostate/EgoStateInterpolator.h>

#define VERBOSE
#if defined(VERBOSE)
#define COUT(X)		std::cout << X << std::endl
#define DEBUG(X)	RTT::Logger::log() << RTT::Logger::Debug << X << RTT::Logger::endl
#define INFO(X)		RTT::Logger::log() << RTT::Logger::Info << X << RTT::Logger::endl
#define WARNING(X)	RTT::Logger::log() << RTT::Logger::Warning << X << RTT::Logger::endl
#define ERROR(X)	RTT::Logger::log() << RTT::Logger::Error << X << RTT::Logger::endl
#define CRITICAL(X)	RTT::Logger::log() << RTT::Logger::Critical << X << RTT::Logger::endl
#define FATAL(X)	RTT::Logger::log() << RTT::Logger::Fatal << X << RTT::Logger::endl
#else
#define COUT(X)		{}
#define DEBUG(X)	{}
#define INFO(X)		{}
#define WARNING(X)	{}
#define ERROR(X)	{}
#define CRITICAL(X)	{}
#define FATAL(X)	{}
#endif

using ::modules::models::carstate::CarState;

namespace aa
{
namespace modules
{
namespace nav
{
namespace controller
{

REGISTERTASKCONTEXT(ControllerPassat);


using namespace std;
using namespace RTT;
using namespace ::math;
using namespace modules::models::carstate;
using RTT::Logger;

ControllerPassat::ControllerPassat(string const & name)
	: AbstractController(name)

	, mPhaseDur("PhaseDuration", "time in seconds for complete steering cycle", 16.0)
	, mWheelAmplitude("WheelAmplitude", "max Angle for the front wheels to steer", 5.0)
	, mKpa("Kpa", "P-Value for Accel PID Controller", 0.08)	// 0.1
	, mKia("Kia", "I-Value for Accel PID Controller", 0.005) // 0.006  //clipping at 50
	, mKda("Kda", "D-Value for Accel PID Controller", 0.0)
	, mKps("Kps", "P-Value for Brake PID Controller", 0.05)	 // 0.15
	, mKis("Kis", "I-Value for Brake PID Controller", 0.001) // 0.002	//clipping at 50
	, mKds("Kds", "D-Value for Brake PID Controller", 0.0)
	, mDynamicHandbrake(0.0)
	, mThrottleOutputScale("ThrottleOutputScale", "Scales the positive controller output", 4.0)
	, mIntegral(0.0)
	, mLimitThrottleBrakeIntegral("LimitThrottleBrakeIntegral", "Limits the Throttle or Brake Integral", 50.0)									//used to be at 50

	, mKpSteer("KpSteer", "P-Value for Steer PID Controller", 1.0) // 1.0/(33.9*D2R) // frontwheelavus 2.0
	, mKiSteer("KiSteer", "I-Value for Steer PID Controller", 0.00)// 0.0 // frontwheelavus 0.0
	, mKdSteer("KdSteer", "D-Value for Steer PID Controller", 1.0) // // frontwheelavus  3.0
	//Use FrontWheelAngle = false, UseSimpleParameterSet with KpSteer = 1 (2), KiSteer = 0 (0), KdSteer = 0 (3);
	, mKpSteerLowSpeedValue("KpSteerLowSpeedValue", "P-Value for Steer PID Controller Low Speed", 1.0)
	, mKpSteerHighSpeedValue("KpSteerHighSpeedValue", "P-Value for Steer PID Controller High Speed", 0.6)
	, mKpSteerInterpolated(0.0)
	, mKiSteerLowSpeedValue("KiSteerLowSpeedValue", "I-Value for Steer PID Controller Low Speed", 0.0)
	, mKiSteerHighSpeedValue("KiSteerHighSpeedValue", "I-Value for Steer PID Controller High Speed", 0.0)
	, mKiSteerInterpolated(0.0)
	, mKdSteerLowSpeedValue("KdSteerLowSpeedValue", "D-Value for Steer PID Controller Low Speed", 1.0)
	, mKdSteerHighSpeedValue("KdSteerHighSpeedValue", "D-Value for Steer PID Controller High Speed", 1.0)
	, mKdSteerInterpolated(0.0)
	, mUseSimplePidSetForSteer("UseSimplePidSetForSteer", "SwitchBack to Simple Model", false) //for Bibendum
	, mSteerIntegral(0.0)
	, mLimitSteerIntegral("LimitSteerIntegral", "Maximum Value of the Steer Integral", 10.0) //4
	, mTolerance("Tolerance", "Tolerance for Steer Angle limiter", 10.0)
	, mLogDataToFile("LogDataToFile", "Write Logged Data to File", false)
	, mMaxLateralAcceleration("MaxLateralAcceleration", "Max lateral acceleration for steer limiter", 3.0)
	, mLowSpeedValue(10.0)
	, mHighSpeedValue(20.0)
	, mVelocitySmoothedThrottle(0.0)
	, mVelocitySmoothedBrake(0.0)
	, mShiftBackward(false)

	, mSoftBrake("SoftBrake", "Value for SoftBrake", -0.12) //-0.15
	, mMinBrake(-0.35) // --> changeComfortSettings!!!-0.2
	, mMaxThrottle(0.5)	// --> changeComfortSettings!!! soft: 0.5, sporty: 0.7
	, mMinBrakeComfort("MinBrakeComfort", "Value for Min Wanted Brake", -0.35)
	, mMaxThrottleComfort("MaxThrottleComfort", "Value for Max Wanted Throttle", 0.5)	//soft: 0.5, sporty: 0.7
	, mMinBrakeSporty("MinBrakeSporty", "Value for Min Wanted Brake", -0.35)
	, mMaxThrottleSporty("MaxThrottleSporty", "Value for Max Wanted Throttle", 0.66)	// tested also but high tire usage: 1.0
	, mMinBrakeUltimate("MinBrakeUltimate", "Value for Min Wanted Brake", -0.35)
	, mMaxThrottleUltimate("MaxThrottleUltimate", "Value for Max Wanted Throttle", 1.0)	// tested also but high tire usage: 1.0
	, mStartRollingThrottleValue("StartRollingThrottleValue", "Throttle when starting", 0.3) //0.4
	, mFullThrottleWheelSpeed("FullThrottleWheelSpeed", "Velocity at which max throttle is released", 20.0)
	, mFullBrakeWheelSpeed("FullBrakeWheelSpeed", "Velocity at which full brake is possible", 0.6)
	, mMaxAbsSteer("MaxAbsSteer", "Value for Max Absolute Steering Value", 0.8)
	, mTargetSpeed("TargetSpeed", "TargetSpeed after Braking", 0.0)
	, mSteerAmplifyer("SteerAmplifyer", "Value for SteeringAcceleration", 3.5)    //in Simulation should be 3.0

	, mObstacleDistToLeft("ObstacleDistToLeft", "Current dist to left vehicles in m", 10.0)
	, mObstacleDistToRight("ObstacleDistToRight", "Current min dist to right vehicles in m", 10.0)
	, mMinObstacleDist("MinObstacleDist", "Desired min dist to side vehicles in m", 0.5)
	, mMaxObstacleAvoidanceDistance("MaxObstacleAvoidanceDistance", "Correction of controller from planned trajectory in m", 0.0) //for safety until tested,used to be 0.4
	, mAvoidSideObstacles("AvoidSideObstacles", "Flag to switch side obstacle avoidance", false)

	, mOverrideFlag("OverrideFlag", "Flag for overriding Controller", false)
	, mLimitSteer("LimitSteer", "limits steer angle depending on velocity", false)
	, mOverrideValue("OverrideValue", "Value for Overriding Controller", 0.0)
	, mDout("Dout", "Comments In", false)
	, mPreviewDistance("PreviewDistance", "Preview Distance for trajectory following in m", 2.0) // 4.0
	, mStaticPreviewSpeed("StaticPreviewSpeed", "Maximum Speed Threshold for Static Preview Distance in m/s", 5.0) // 5.0
	, mDynamicPreviewSpeedScale("DynamicPreviewSpeedScale", "Preview Distance Scale, used when Speed is larger than StaticPreviewSpeed", 1.0) //THF 0.5;

	, mPredictionTimeCarModelForSteer("PredictionTimeCarModelForSteer", "Time by Which the CarModel is Predicted for PathFollowController", 0.0) //0.0
	, mCarDirAngleOffsetDeg("CarDirAngleOffset", "offset for the yaw angle off the car", -0.6) //see rangecut
	, mUseFrontWheelsForSteerController("UseFrontWheelsForSteerController", "Use Front Wheel Angle instead of Body Angle", false) //for bibendum

	, mAvoidEgoStateJumps("AvoidEgoStateJumps", "Avoids Jumps in Egostate", true)
	, mAvoidEgoStateJumpsLowerThreshold("AvoidEgoStateJumpsLowerThreshold", "Lower Threshold for Egostate Jumps", 0.1)
	, mAvoidEgoStateJumpsUpperThreshold("AvoidEgoStateJumpsUpperThreshold", "Upper Threshold for Egostate Jumps", 1.0)
	, mAvoidEgoStateJumpsTimeToInterpolate("AvoidEgoStateTimeToInterpolate", "Time to interpolate between two egostates in seconds", 2.0)


	, mSpeed(0.0)
	, mSteer(0.0)
    , mGear(0) // initialization added by shuiying 06.11 2013. otherwise, it will be used uninitialized
	, mGearSet(0)
	, mGearWish(8) //should be set from controller

	, mLastSteerAngle(0.0)
	, mPreLastSteerAngle(0.0)
	, mGearDorRSelectedTime(0.0)
	, mDistanceToTrajectory(0.0)
	, mDifferenceAngleBearingHeading(0.0)
	, mBrakingDownCurrently(0.0) //if distance to trajectory gets too big
	, mPackethistoryIdx(1)
	, mFreq(0.0)
	, mLogDataCounter(0.0)
	, ts(0.0)
	, logts(0.0)
	, mFileName("")
	, mCyclesFlashLightsOn(0.0)
	, mSignedLateralError(0.0)
	, mLateralErrorSign(0.0)
	, mIMUCyclesOnlineCalibration(0.0)
	, mIMUErrorIntegral(0.0)
	, mIMUOnlineCalibrate("IMUOnlineCalibrate", "Flag for Online Calibration of IMU", false)
	, mLearnHumanControllerParameters("LearnHumanControllerParameters", "To enable learning", false)
	, mApplyHumanControllerParameters("ApplyHumanControllerParameters", "To apply learned params", false)
	, mInitializedLearningParams(false)
	, mLearnNNSteering("LearnNNSteering", "Enables Learning with NN", false)
	, mApplyNNSteering("ApplyNNSteering", "Applys Learned NN", false)
	, mInitializeNN(false)
    , mAccumulatedOffset(0.0, 0.0, 0.0)
	, mCurrentTimeToInterpolate(0.0)

{
	addProperty(mPhaseDur);
	addProperty(mWheelAmplitude);
	addProperty(mKpa);
	addProperty(mKia);
	addProperty(mKda);
	addProperty(mKps);
	addProperty(mKis);
	addProperty(mKds);
	addProperty(mThrottleOutputScale);
	addProperty(mLimitThrottleBrakeIntegral);
	addProperty(mKpSteer);
	addProperty(mKiSteer);
	addProperty(mKdSteer);
	addProperty(mKpSteerLowSpeedValue);
	addProperty(mKpSteerHighSpeedValue);
	addProperty(mKiSteerLowSpeedValue);
	addProperty(mKiSteerHighSpeedValue);
	addProperty(mKdSteerLowSpeedValue);
	addProperty(mKdSteerHighSpeedValue);
	addProperty(mUseSimplePidSetForSteer);
	addProperty(mLimitSteerIntegral);
	addProperty(mTolerance);
	addProperty(mLogDataToFile);
	addProperty(mMaxLateralAcceleration);
	addProperty(mSoftBrake);
	addProperty(mMinBrakeComfort);
	addProperty(mMaxThrottleComfort);
	addProperty(mMinBrakeSporty);
	addProperty(mMaxThrottleSporty);
	addProperty(mMinBrakeUltimate);
	addProperty(mMaxThrottleUltimate);
	addProperty(mStartRollingThrottleValue);
	addProperty(mFullThrottleWheelSpeed);
	addProperty(mFullBrakeWheelSpeed);
	addProperty(mMaxAbsSteer);
	addProperty(mTargetSpeed);
	addProperty(mSteerAmplifyer);

	addProperty(mObstacleDistToLeft);
	addProperty(mObstacleDistToRight);
	addProperty(mMinObstacleDist);
	addProperty(mMaxObstacleAvoidanceDistance);
	addProperty(mAvoidSideObstacles);

	addProperty(mOverrideFlag);
	addProperty(mOverrideValue);
	addProperty(mDout);
	addProperty(mPreviewDistance);
	addProperty(mLimitSteer);
	addProperty(mStaticPreviewSpeed);
	addProperty(mDynamicPreviewSpeedScale);
	addProperty(mPredictionTimeCarModelForSteer);
	addProperty(mCarDirAngleOffsetDeg);
	addProperty(mUseFrontWheelsForSteerController);
	addProperty(mAvoidEgoStateJumps);
	addProperty(mAvoidEgoStateJumpsLowerThreshold);
	addProperty(mAvoidEgoStateJumpsUpperThreshold);
	addProperty(mAvoidEgoStateJumpsTimeToInterpolate);

	addProperty(mIMUOnlineCalibrate);
	addProperty(mLearnHumanControllerParameters);
	addProperty(mApplyHumanControllerParameters);
	addProperty(mLearnNNSteering);
	addProperty(mApplyNNSteering);
}

ControllerPassat::~ControllerPassat()
{
}


bool ControllerPassat::startHook()
{
	bool result = AbstractController::startHook();

	Logger::In in("ControllerPassat");

	count = 0;
	mInForwardOrBackwardDriveEngaging = false;
	mIntegral = -mLimitThrottleBrakeIntegral;
	mOldError = 0;
	mNewError = 0;
	mOutput = 0;

	mSteerIntegral = 0;
	mOldSteerError = 0;
	mNewSteerError = 0;
	mSteerOutput = 0;

	mZeroTime.stamp();

	if (mSimulationMode.rvalue()) {
		mCarDirAngleOffsetDeg = 0.0;
	}

	return result;
}


flt ControllerPassat::getControlOutput(flt wantedSpeed, flt currentSpeed)
{

	if ((!mSimulationMode.rvalue()) && (mGearDorRSelectedTime < 0.5) && (currentSpeed <= 0.5))	{
//		COUT("setting wanted speed to zero");
		wantedSpeed = 0.0;
	}


	mNewError = wantedSpeed - currentSpeed;
//	25hz controller
	flt help;

	mIntegral = rangeCut(-mLimitThrottleBrakeIntegral.rvalue(), mIntegral += mNewError * getPeriod() / 0.04, mLimitThrottleBrakeIntegral.rvalue());


	mOutput = mKps * mNewError + mKis * mIntegral + mKds * (mNewError - mOldError) * 0.04 / getPeriod();

	//COUT("Error " << mNewError << " OutPut " << mOutput << " Integral " << mIntegral);

	if (mOutput > 0.0) {
		mOutput *= mThrottleOutputScale;
	}

	//COUT("Error " << mNewError<< "WantS " << wantedSpeed << " CurSp " << currentSpeed << " | OutPut " << mOutput << " Integral " << mIntegral);


	mOldError = mNewError;

	mDebug1 = wantedSpeed;
	mDebug2 = currentSpeed;

	//Dynamic Handbrake
	if ((wantedSpeed == 0.0) && (currentSpeed <= 4.0) && (currentSpeed > 0.01)) {
		mDynamicHandbrake++;
		mOutput -= mDynamicHandbrake * 0.0002;

		//	COUT("DynamicHandbrake " << mDynamicHandbrake * 0.0002 * 127.5);
	}
	else {
		if ((wantedSpeed == 0) && (currentSpeed <= 0.01)) { //0.5
			mOutput = mSoftBrake - mDynamicHandbrake * 0.0002; // 2 bar sper second at 100 Hz;
			//COUT(mOutput);
		}
		else {
			mDynamicHandbrake = 0.0;
		}
	}

	return rangeCut(-1.0, mOutput, 1.0);
}



flt ControllerPassat::steerPIDcontroller(flt wantedSteerAngle, flt currentSteerAngle)
{
	mNewSteerError = wantedSteerAngle - currentSteerAngle;
	normalizeAngle(mNewSteerError);

	mSteerIntegral = rangeCut(-mLimitSteerIntegral.rvalue(),	mSteerIntegral += mNewSteerError * getPeriod() / 0.04, mLimitSteerIntegral.rvalue());

	if (mUseSimplePidSetForSteer.rvalue() == true) {
		mSteerOutput = mKpSteer * mNewSteerError + mKiSteer * mSteerIntegral + mKdSteer * (mNewSteerError - mOldSteerError) * 0.04 / getPeriod();
		//  COUT("Static");
	}
	else {
		mSteerOutput = ::math::boundedLinearInterpolation(mCurPassatCarState.wheelSpeeds.speedAvg(), mLowSpeedValue, mHighSpeedValue, mKpSteerLowSpeedValue, mKpSteerHighSpeedValue)
					   * mNewSteerError +
					   ::math::boundedLinearInterpolation(mCurPassatCarState.wheelSpeeds.speedAvg(), mLowSpeedValue, mHighSpeedValue, mKiSteerLowSpeedValue, mKiSteerHighSpeedValue)
					   * mSteerIntegral +
					   ::math::boundedLinearInterpolation(mCurPassatCarState.wheelSpeeds.speedAvg(), mLowSpeedValue, mHighSpeedValue, mKdSteerLowSpeedValue, mKdSteerHighSpeedValue)
					   * (mNewSteerError - mOldSteerError) * 0.04 / getPeriod();
		// COUT("Dynamic");
	}

	mOldSteerError = mNewSteerError;
	return (mSteerOutput);
}



int ControllerPassat::controlGear()
{

	//TODO simple gear decision
	Plan const & curPlan = *mCurPlan;
	std::pair<flt, flt> const dom(curPlan.domain());

	boost::optional<Plan::action_descr> reverseAction = curPlan.findFirstAction(Plan::REVERSE, dom.first, dom.second);


	mShiftBackward = false;

	if (!reverseAction) {
		mShiftBackward = false;
	}
	else {
		//where does reverse begin?
		Plan::action_descr const action = reverseAction.get();
		flt const reverseStart = action.get<1>();
		flt const reverseEnd = action.get<2>();

		if (abs(reverseStart - dom.first) < 0.1f) {
			mShiftBackward = true;
		}
		else {
			mShiftBackward = false;
		}
	}


//    if (count%100 == 0) {
//      COUT("Gear: " << mCurPassatCarState.gearStatus.chosenPos());
//    }
	if ((mCurPassatCarState.gearStatus.chosenPos() != aa::modules::io::passat::GearStatus::Pos_D_Automatic &&
			mCurPassatCarState.gearStatus.chosenPos() != aa::modules::io::passat::GearStatus::Pos_S_Automatic_Sport) &&
			(mCurPassatCarState.gearStatus.chosenPos() != aa::modules::io::passat::GearStatus::Pos_R))	{

		mGearSet = 0;
		mGearDorRSelectedTime = 0;
	}
	else {

		if (mCurPassatCarState.gearStatus.chosenPos() == aa::modules::io::passat::GearStatus::Pos_D_Automatic ||
				mCurPassatCarState.gearStatus.chosenPos() == aa::modules::io::passat::GearStatus::Pos_S_Automatic_Sport) {
			mGearSet = CarState::GEAR_DRIVE;

			if (mSportySet.rvalue()) {
				mGearSet = CarState::GEAR_SPORT;
			}



			mGearDorRSelectedTime += getPeriod();
		}

		if (mCurPassatCarState.gearStatus.chosenPos() == aa::modules::io::passat::GearStatus::Pos_R) {
			mGearSet = CarState::GEAR_REVERSE;
			mGearDorRSelectedTime += getPeriod();
		}
	}

	//Status  requested

	//EngagingDrive
	if (mGearWish != 0 && mGearWish != CarState::GEAR_PARK) {			//Start Engaging
		engageForwardOrBackwardDrive(mGearWish);
	}
	else

		//DisengagingReverse
	{
		//Interrupt Engaging
		disengageGear();
	}

	return mGear;
}


flt ControllerPassat::controlThrottleBrake(flt curSpeed, flt wantedSpeed)
{
	//COUT("Activation Request Set");
	//SAFE
	if (!mAllowDriverIntervention) {
		mSpeed = mSoftBrake.rvalue();  //Default
	}

	if (mInForwardOrBackwardDriveEngaging) {
		mSpeed = rangeCut(mMinBrake, mSpeed, 0.0);
		return mSpeed; //set by shifting module
	}

	if (mOverrideFlag.value()) {
		flt throttleOut = getControlOutput(mOverrideValue.rvalue(), curSpeed);

		mSpeed = rangeCut(mMinBrake, throttleOut, mMaxThrottle);
		return mSpeed;
	}


	//tp approach trajectory more than 4 meters or more than 60 degrees
	if ((mDistanceToTrajectory > 4.0 || fabs(mDifferenceAngleBearingHeading) > PI * 0.35)) {

		// mBrakingDownCurrently = max(mBrakingDownCurrently - getPeriod() * 1, 4.0);
		wantedSpeed = min(4.0, wantedSpeed); //1 m/s/s

		if (mDistanceToTrajectory < 10) {
			wantedSpeed = min(2.0, wantedSpeed);
		}

		if (!(count % 100)) {
			WARNING("Left Trajectory too big: " << mDistanceToTrajectory << " | Angle: " << mDifferenceAngleBearingHeading << " | Wanted " << wantedSpeed);
		}
	}


	mSpeed = getControlOutput(wantedSpeed, curSpeed);   //where comes curSpeed From?

	mVelocitySmoothedThrottle = mStartRollingThrottleValue + (mCurPassatCarState.wheelSpeeds.speedAvg() /
								mFullThrottleWheelSpeed) * (mMaxThrottle - mStartRollingThrottleValue);

	//mVelocitySmoothedBrake = mSoftBrake + (mCurPassatCarState.wheelSpeeds.speedAvg() /
	//									   mFullBrakeWheelSpeed) * (mMinBrake - mSoftBrake);

	if (mSportySet.rvalue() == false) {		// go smooth if Settings are Comfort and not Sport
//		mSpeed = rangeCut(mVelocitySmoothedBrake, mSpeed, mVelocitySmoothedThrottle);
		mSpeed = rangeCut(mMinBrake, mSpeed, mVelocitySmoothedThrottle);

	}

	mSpeed = rangeCut(mMinBrake, mSpeed, mMaxThrottle);

	mDebug3 = mSpeed;		//throttle

	return mSpeed;
}

flt ControllerPassat::controlSteering(Plan_ptr plan)
{
	mCarDirAngleOffsetDeg = rangeCut(-1.0, mCarDirAngleOffsetDeg.rvalue(), 1.0);  //for safety reasons

	now.stamp();
	mSteer = 0.0;

	flt const frontShaftDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontShaftDistance")->value();


    Vec3 const imuPos = mCurEgoState.position();
    Vec3 const carDirVec = normalized(mCurEgoState.forwardDirection());
    Vec3 const carPos = imuPos + frontShaftDistance * carDirVec;
    Vec3 const carPosReverse = imuPos - 2.709 * carDirVec;

	Plan const & curPlan = *mCurPlan;
	std::pair<flt, flt> const dom(curPlan.domain());

    Vec3 currentCarDirVec;
    Vec3 currentCarPos;

    Vec3 carDirPred;
    Vec3 carDirPred_2;

	carDirPred[0] = carDirVec[0];
    carDirPred[1] = carDirVec[1];
    carDirPred[2] = carDirVec[2];

	currentCarDirVec = carDirVec;

	//recognize jumps in egostate
	mPredictedEgoState = extrapolate(mLastEgoState, mCurEgoState);

    const Vec3 vehicleDir = mCurEgoState.forwardDirection();
    const Vec3 sidewayDir = normalized(Vec3(vehicleDir[1], -vehicleDir[0], 0.f));

    Vec3 offset = mCurEgoState.position() - mPredictedEgoState.position();
    flt lateralDist = dot_product(offset, sidewayDir);

	if (fabs(lateralDist) > mAvoidEgoStateJumpsLowerThreshold) {
		COUT(" !!!!!!!!!!! EgoStateJump detected: " << lateralDist << " m ");
		mAccumulatedOffset += offset;
		rangeCut(-mAvoidEgoStateJumpsUpperThreshold.rvalue(), mAccumulatedOffset[0], mAvoidEgoStateJumpsUpperThreshold.rvalue());
		rangeCut(-mAvoidEgoStateJumpsUpperThreshold.rvalue(), mAccumulatedOffset[1], mAvoidEgoStateJumpsUpperThreshold.rvalue());
		mDegradeSubtrahendOfAccumulatedOffset = mAccumulatedOffset / (mAvoidEgoStateJumpsTimeToInterpolate / getPeriod());
		mCurrentTimeToInterpolate = 0.0;
	}

	mCurrentTimeToInterpolate += getPeriod();
	mAccumulatedOffset -= mDegradeSubtrahendOfAccumulatedOffset;

	if (mCurrentTimeToInterpolate >= mAvoidEgoStateJumpsTimeToInterpolate) {
        mAccumulatedOffset = Vec3(0.0, 0.0, 0.0);
	}

	//now handle jumps
	if (mAvoidEgoStateJumps) {
		currentCarPos = carPos - mAccumulatedOffset;
	}
	else {
		currentCarPos = carPos;
	}

	//until here

	// ----> NN Learning starts

	if (mApplyNNSteering.rvalue()) {
		mLearnNNSteering.set(false);
		//mApplyNNSteering.set(false);
		mSteer = applyNNSteering();
		return mSteer;
		mInitializeNN = false;
	}

	// ----> NN Learning ends

	///////////////20110415

	flt assumedDelayInSeconds = mPredictionTimeCarModelForSteer;//
	flt predictedGlobalFrontWheelAngle = mCurPassatCarState.steerAssist3Status.steerAngle() / 530.0 * 33.9 * D2R + atan2(carDirVec[1], carDirVec[0]);
	predictedGlobalFrontWheelAngle += mCarDirAngleOffsetDeg * D2R;
	//COUT("NonPredictedWheelAngle " << predictedGlobalFrontWheelAngle);

	mDistanceToTrajectory = sqrt(mClosestSqrDistToTrajectory);


	if (assumedDelayInSeconds > 0.0) {

		////Daniel COUT("Before, ClosestParam: " << mClosestParamOnTrajectory << " | Curr. Carpos X: " << currentCarPos[0] << " | Curr. Carpos Y: " << currentCarPos[1]);

		/** Predict Carposition,
		* Using Wheel Speed, SteeringWheelAngle, old position and old angle, generating new Position and new angle
		*/

		flt frontWheelAngle = (mCurPassatCarState.steerAssist3Status.steerAngle()  +
							   assumedDelayInSeconds * (mCurPassatCarState.steerAssist3Status.steerAngle() - mPreLastSteerAngle) * 50) / 530.0 * 33.9 * D2R;

		// omega = (v * sin alpha) / L
		flt omega = mCurPassatCarState.wheelSpeeds.speedAvg() * sin(frontWheelAngle) / 2.709; //angle speed in rad/s = carSpeed * sin(frontwheelAngle) / shaftDist
		flt angleDelta = assumedDelayInSeconds * omega;
		flt angleDelta_2 = assumedDelayInSeconds * omega * 0.5;



		//Vec2 carDirPred;
		carDirPred[0] =  currentCarDirVec[0] * cos(angleDelta) - currentCarDirVec[1] * sin(angleDelta);
		carDirPred[1] =  currentCarDirVec[0] * sin(angleDelta) + currentCarDirVec[1] * cos(angleDelta);

		//Vec2 carDirPred_2;
		carDirPred_2[0] =  currentCarDirVec[0] * cos(angleDelta_2) - currentCarDirVec[1] * sin(angleDelta_2);
		carDirPred_2[1] =  currentCarDirVec[0] * sin(angleDelta_2) + currentCarDirVec[1] * cos(angleDelta_2);

        Vec3 carPosPred = currentCarPos + carDirPred_2 * mCurPassatCarState.wheelSpeeds.speedAvg() * assumedDelayInSeconds; //improve by circular rotation

		flt angleHead = atan2(carDirPred[1], carDirPred[0]);
		currentCarPos = carPosPred;

		predictedGlobalFrontWheelAngle = frontWheelAngle + angleHead + mCarDirAngleOffsetDeg * D2R; //predicted

		//COUT("...PredictedWheelAngle " << predictedGlobalFrontWheelAngle);


		boost::tie(mClosestSqrDistToTrajectory, mClosestParamOnTrajectory) = findClosestPoint(curPlan, dom.first, dom.second, (dom.first + dom.second) / 2.0, currentCarPos);
		mDistanceToTrajectory = sqrt(mClosestSqrDistToTrajectory);

		////Daniel COUT("After, ClosestParam: " << mClosestParamOnTrajectory  << " | Curr. Carpos X: " << currentCarPos[0] << " | Curr. Carpos Y: " << currentCarPos[1] << " | WheelSpeeds: " << mCurPassatCarState.wheelSpeeds.speedAvg());

		//check if closestparamOnTrajectory changed or not
		//steeringWheelModel needed
		////// prediction ends
	}

	/////////////20110415



	flt followParam = 0.f;

	/*	flt tP1 = curPlan(rangeCut(dom.first, mClosestParamOnTrajectory, dom.second));
	        flt tP2 = curPlan(rangeCut(dom.first, mClosestParamOnTrajectory + 0.1, dom.second));
	        flt tangentVector = tP2 - tP1;*/

	flt scale = 0;			//for dynamic previewdistances for larger speeds

	if (mCurPassatCarState.wheelSpeeds.speedAvg() > mStaticPreviewSpeed) {
		scale = mDynamicPreviewSpeedScale;
	}
	else {
		scale = 0;
	}

	if (mGearWish == 2) { //Drive Backwards
		followParam = rangeCut((flt)dom.first, mClosestParamOnTrajectory + (mPreviewDistance.value() +
							   scale * (mCurPassatCarState.wheelSpeeds.speedAvg() - mStaticPreviewSpeed) + 2.709), (flt)dom.second);

	}
	else {
		followParam = rangeCut((flt)dom.first, mClosestParamOnTrajectory + mPreviewDistance.value() +
							   scale * (mCurPassatCarState.wheelSpeeds.speedAvg() - mStaticPreviewSpeed) , (flt)dom.second);
	}


    Vec3 diffPosTrajCarVec(0.f, 0.f, 0.f);
	flt angleHead = atan2(carDirVec[1], carDirVec[0]);



    Vec3 sideObstcaleAvoidanceVector(0.f, calculateSideObstaclesAvoidance(), 0.f);
	sideObstcaleAvoidanceVector[0] = cos(angleHead) * sideObstcaleAvoidanceVector[0] + sin(angleHead) * sideObstcaleAvoidanceVector[1];
	sideObstcaleAvoidanceVector[1] = -sin(angleHead) * sideObstcaleAvoidanceVector[0] + cos(angleHead) * sideObstcaleAvoidanceVector[1];



	if (mGearWish == 2) { //Drive Backwards
		diffPosTrajCarVec = curPlan(followParam) - carPosReverse;
	}
	else {
		if (mAvoidSideObstacles) {
			diffPosTrajCarVec = curPlan(followParam) - currentCarPos + sideObstcaleAvoidanceVector;

			if (count % 100 == 0) {
				flt mleft, mright;
				mSideObstacleDistLeftIn.read(mleft);
				mSideObstacleDistRightIn.read(mright);
				std::cout << " Avoidance: " << calculateSideObstaclesAvoidance() << " X: " << diffPosTrajCarVec[0] << " Y: " << diffPosTrajCarVec[1]  << " DistL " << mleft << " DistR " << mright << std::endl;
			}



		}
		else {
			diffPosTrajCarVec = curPlan(followParam) - currentCarPos;
		}

	}

	flt angleBear = atan2(diffPosTrajCarVec[1], diffPosTrajCarVec[0]);


	if (mUseFrontWheelsForSteerController) { //Use Front Wheel Angle (global) or Body Angle
		mDifferenceAngleBearingHeading = angleBear - predictedGlobalFrontWheelAngle;
	}
	else {
		mDifferenceAngleBearingHeading = angleBear - (angleHead + mCarDirAngleOffsetDeg * D2R);

		//COUT("AngleBear " << angleBear << " | angleHead " << (angleHead + mCarDirAngleOffsetDeg * D2R));
		////COUT("Difference Bear-Head " << mDifferenceAngleBearingHeading);

	}

	normalizeAngle(mDifferenceAngleBearingHeading);
	flt steerAngle;


	//if ((count % 100) == 0) {
	//COUT("Angle Bear " << angleBear * ::math::R2D << " | PredFrontWheel: " << predictedGlobalFrontWheelAngle * ::math::R2D << "| diff: " << angleDiffBearHead * ::math::R2D << " | SteerIntegr. " << mSteerIntegral);
	//}

	//	if (curPlan.findFirstAction(Plan::REVERSE, ))
	//
	if (mGearWish == 2) { //Drive Backwards
//		angleDiffBearHead += ::math::PI;
//		angleDiffBearHead = -angleDiffBearHead;
		//ToDo
		angleBear += ::math::PI;
		angleBear = -angleBear;
	}

//	normalizeAngle(angleDiffBearHead);

	//steerAngle = (-angleDiffBearHead / ::math::PI * mSteerAmplifyer);

	//steerAngle = steerPIDcontroller(-angleDiffBearHead, 0.0);

	if (mUseFrontWheelsForSteerController) {
		steerAngle = -steerPIDcontroller(angleBear, predictedGlobalFrontWheelAngle);
	}
	else {
		steerAngle = -steerPIDcontroller(angleBear, (angleHead + mCarDirAngleOffsetDeg * D2R));
		////COUT("SteerAngle " << steerAngle);
	}

	if (mGearWish == 2) { //Drive Backwards
		steerAngle = steerAngle * 3.0;
	}


	mSteer = rangeCut(-1.0, steerAngle, 1.0);


	mPreLastSteerAngle = mLastSteerAngle;
	mLastSteerAngle = mCurPassatCarState.steerAssist3Status.steerAngle();


	//////////////// STEER ANGLE LIMITER CLONE
	float WHEELBASE = 2.709;
	float MAXWHEELANGLE = 33.894;
	float MAXSTEERINGWHEELANGLE = 530;

	float maxWheelAngleAtGivenSpeed;
	float mLimitValue;

	if ((mCurPassatCarState.wheelSpeeds.speedFrontAvg() != 0.0) &&
			(fabs(WHEELBASE * mMaxLateralAcceleration  / (mCurPassatCarState.wheelSpeeds.speedFrontAvg() * mCurPassatCarState.wheelSpeeds.speedFrontAvg())) < 1)) {

		maxWheelAngleAtGivenSpeed = asin(WHEELBASE * mMaxLateralAcceleration  / (mCurPassatCarState.wheelSpeeds.speedFrontAvg() * mCurPassatCarState.wheelSpeeds.speedFrontAvg()));
		maxWheelAngleAtGivenSpeed = maxWheelAngleAtGivenSpeed * (180.0 / 3.14159265358979323846);



		if (MAXSTEERINGWHEELANGLE < fabs((maxWheelAngleAtGivenSpeed / MAXWHEELANGLE * MAXSTEERINGWHEELANGLE) + mTolerance)) {
			mLimitValue = MAXSTEERINGWHEELANGLE;
		}
		else {
			mLimitValue = fabs((maxWheelAngleAtGivenSpeed / MAXWHEELANGLE * MAXSTEERINGWHEELANGLE) + mTolerance);
		}

		//return mLimitValue;

	}
	else {
		//return MAXSTEERINGWHEELANGLE;
		mLimitValue = MAXSTEERINGWHEELANGLE;
	}


//		std::cout << " mLimitValue " << mLimitValue << std::endl;
	//////////////// STEER ANGLE LIMITER CLONE START

    Vec3 mCarToTrajectoryVector;
	mCarToTrajectoryVector = curPlan(mClosestParamOnTrajectory) - currentCarPos;
//	std::cout << " lateral error " << sqrt(mCarToTrajectoryVector[0]*mCarToTrajectoryVector[0]+mCarToTrajectoryVector[1]*mCarToTrajectoryVector[1]) + << std::endl;

	if (mLimitSteer && (fabs(mLimitValue / 530.0) < fabs(mSteer))) {
		std::cout << "STEERANGLELIMITTING - WANTED: " << mSteer * -530.0 << " | LIMIT: " << mLimitValue << " | Velocity: " << mCurPassatCarState.wheelSpeeds.speedFrontAvg() << std::endl;

		if (mSteer < -mLimitValue / 530.0) {
			mSteer = -mLimitValue / 530.0;
		}
		else if (mSteer > mLimitValue / 530.0) {
			mSteer = mLimitValue / 530.0;
		}
	}

	//////////////// STEER ANGLE LIMITER CLONE FINISH
	flt mLaterErrorAngleDiff = atan2(mCarToTrajectoryVector[1], mCarToTrajectoryVector[0]) - atan2(carDirVec[1], carDirVec[0]);
	normalizeAngle(mLaterErrorAngleDiff);
	mLateralErrorSign = -1.0 * mLaterErrorAngleDiff;
	mLateralErrorSign = mLateralErrorSign / fabs(mLateralErrorSign);
	mSignedLateralError = mLateralErrorSign * sqrt(mCarToTrajectoryVector[0] * mCarToTrajectoryVector[0] + mCarToTrajectoryVector[1] * mCarToTrajectoryVector[1]);




	//Logging Starts:
	if (mLogDataToFile) {
		if (mLogDataCounter == 0) {	//OPEN FILE
			mLogDataCounter++;

			std::stringstream strstr;
			//strstr << "MeasureLateralError_" << (int)((1E-6f * RTT::TimeService::ticks2nsecs(mStartTime - mZeroTime))) << ".txt";
			strstr << "CanLogFile_" << ts << ".txt";
			os.open(strstr.str().c_str());
			mFileName = strstr.str().c_str();
			COUT("LogFile " << mFileName << " created");
			os.precision(6);
			logts = ts;

			os << "#Time," << "GPS-PX," << "PY," <<  "DirX," << "DirY," <<  "TX," << "TY," << "Odo-FL," << "FR," <<
			   "RL," << "RR," << "Speed-FL,"  << "FR," <<  "RL," << "RR,"  <<  "CurStAng," <<
			   "DesStAng," << "CurStMom," << "BrPr,"  << "ThrVlt," << " A/H," << "LatErr"   <<  std::endl;
		}
		else {	//WRITE INPUT


			os << 0.001 * (ts - logts) << "," << currentCarPos[0] << "," << currentCarPos[1] <<
			   "," << carDirVec[0] << "," << carDirVec[1] <<
			   "," << curPlan(mClosestParamOnTrajectory)[0] << "," << curPlan(mClosestParamOnTrajectory)[1] <<
			   "," << mCurPassatCarState.pathPulse.pulseFrontL() << "," << mCurPassatCarState.pathPulse.pulseFrontR() <<
			   "," << mCurPassatCarState.pathPulse.pulseRearL() << "," << mCurPassatCarState.pathPulse.pulseRearR() <<
			   "," <<  mCurPassatCarState.wheelSpeeds.speedFrontL() << "," <<  mCurPassatCarState.wheelSpeeds.speedFrontR() <<
			   "," <<  mCurPassatCarState.wheelSpeeds.speedRearL() << "," <<  mCurPassatCarState.wheelSpeeds.speedRearR() <<
			   "," << mCurPassatCarState.steerAssist3Status.steerAngle() << "," << -530.0 * mSteer <<
			   "," << mCurPassatCarState.steerAssist3Status.steerMomentum() <<
			   "," << mCurPassatCarState.brakeStatus1.actualPressure() << "," << mCurPassatCarState.throttleStatus.voltageMotor1() <<
			   "," << (mCurPassatCarState.watchdogStatus.powerOffReason() == 0 ? "1" : "0") <<
			   ","  << mSignedLateralError  << std::endl;

			if ((ts % 500) == 0) {
				COUT("Lateral Error in m: " <<  mSignedLateralError);
			}
		}
	}
	else {
		if (mLogDataCounter == 0) {
			// do nothing
		}
		else {   //FINISH FILE
			mLogDataCounter = 0;
			os.close();
			COUT("LogFile " << mFileName << " written");
		}
	}

	//Logging Finishs
	//std::cout << " UPDATE " << std::endl;

	/* Online calibration of imu starts */
	if (mIMUOnlineCalibrate.rvalue()) {
		if ((mCurPassatCarState.watchdogStatus.powerOffReason() == 0)  &&
				(fabs(-530 * mSteer) <= 2.5) && (mCurPassatCarState.wheelSpeeds.speedFrontAvg() >= 15) &&
				(fabs(mSignedLateralError) <= 0.25)) {
			flt mIMUChangeOffset = onlineCalibrateIMU();
			mCarDirAngleOffsetDeg = mCarDirAngleOffsetDeg + mIMUChangeOffset;

			if (mIMUChangeOffset != 0) {
				COUT("Changed IMU Calibration Offset by " << mIMUChangeOffset << " to value " << mCarDirAngleOffsetDeg);
			}

			//safetyClipping
			mCarDirAngleOffsetDeg = rangeCut(-1.0, mCarDirAngleOffsetDeg.rvalue(), 0.1);
		}
		else {
			mIMUErrorIntegral = 0;
			mIMUCyclesOnlineCalibration = 0;
		}
	}



	// ----> NN Learning starts 2
	if (mLearnNNSteering.rvalue()) {
		if (!mInitializeNN) {
			initializeNNSteering();
			mInitializeNN = true;
		}

        /**
         * @brief learnNNSteering ARND function body empty
		learnNNSteering((Vec2)curPlan(mClosestParamOnTrajectory),
						(Vec2)rangeCut((flt)dom.first, mClosestParamOnTrajectory + 10.0, (flt)dom.second),
						(Vec2)rangeCut((flt)dom.first, mClosestParamOnTrajectory + 20.0, (flt)dom.second),
						currentCarPos, carDirVec, mSteer);
         */
    }

	// ----> NN Learning ends 2




	/* Online calibration of imu ends */


	//Predict EgoState with 100 Hz;
	//mEgoStatePrediction = carPos + carDirVec * mCurPassatCarState.wheelSpeeds.speedFrontAvg() * getPeriod();
	//COUT("CPX: " << carPos[0] << " CPY: " << carPos[1] << " CPX-P: " << mEgoStatePrediction[0] << " CPY-P: " << mEgoStatePrediction[1]);

	mLastEgoState = mCurEgoState;

	return mSteer;
}



flt ControllerPassat::calculateSideObstaclesAvoidance()
{
	flt correctionValue = 0.0;

	if (!(mAvoidSideObstacles && mSideObstacleDistLeftIn.connected() && mSideObstacleDistRightIn.connected())) {
		return 0.0;
	}
	else {
		flt mLeft;
		flt mRight;
		mSideObstacleDistLeftIn.read(mLeft);
		mSideObstacleDistRightIn.read(mRight);
		mObstacleDistToLeft = mLeft;
		mObstacleDistToRight = mRight;

		if ((mObstacleDistToLeft + mObstacleDistToRight) > mMinObstacleDist * 2.0) { // Tunnel big enough
			if (mObstacleDistToLeft < mMinObstacleDist) {
				correctionValue = -(mMinObstacleDist - mObstacleDistToLeft); // negative = dodge to right
			}
			else if (mObstacleDistToRight < mMinObstacleDist) {
				correctionValue = (mMinObstacleDist - mObstacleDistToRight); // positive = dodge to left
			}
			else {
				correctionValue = 0.0;
			}
		}
		else {      // tunnel
			correctionValue = 0.5 * (mObstacleDistToLeft - mObstacleDistToRight);
		}
	}

	correctionValue = rangeCut(-mMaxObstacleAvoidanceDistance.rvalue(), correctionValue, mMaxObstacleAvoidanceDistance.rvalue());
	return correctionValue;
}



AuxDevicesData ControllerPassat::controlAuxDevices(Plan_ptr plan)
{
	AuxDevicesData auxDevicesCorrection;

	auxDevicesCorrection.headlightState = AuxDevicesData::HEADLIGHT_OFF;
	auxDevicesCorrection.wiperState = AuxDevicesData::WIPER_OFF;
	auxDevicesCorrection.sirenState = AuxDevicesData::SIREN_OFF;
	auxDevicesCorrection.turnsignalState = AuxDevicesData::TURNSIGNAL_OFF;

	if (!plan) {
		return auxDevicesCorrection;
	}

	Plan const & curPlan = *plan;
	std::pair<flt, flt> const dom(curPlan.domain());

	boost::optional<Plan::action_descr> laneChangeAction = curPlan.findFirstAction(Plan::LANE_CHANGE, dom.first, dom.second);

	if (laneChangeAction) {
		Plan::action_descr const action = laneChangeAction.get();
		flt const laneChangeStart = action.get<1>();
		flt const laneChangeEnd = action.get<2>();
		int const laneChangeIndex = boost::any_cast<int>(action.get<3>());

		if (laneChangeStart - 5.f <= mClosestParamOnTrajectory && mClosestParamOnTrajectory <= laneChangeEnd) {
			if (laneChangeIndex > 0) {
				auxDevicesCorrection.turnsignalState = AuxDevicesData::TURNSIGNAL_RIGHT;
			}
			else {
				auxDevicesCorrection.turnsignalState = AuxDevicesData::TURNSIGNAL_LEFT;
			}
		}
	}

	return auxDevicesCorrection;
}





void ControllerPassat::engageForwardOrBackwardDrive(int gearPosition)
{
	Logger::In in("ControllerPassat");

	//Do NOTHING IF wanted and selected gear are categorical equal
	if (((mCurPassatCarState.gearStatus.chosenPos() == aa::modules::io::passat::GearStatus::Pos_D_Automatic ||
			mCurPassatCarState.gearStatus.chosenPos() == aa::modules::io::passat::GearStatus::Pos_S_Automatic_Sport) &&
			(gearPosition == CarState::GEAR_DRIVE || gearPosition == CarState::GEAR_SPORT)) ||
			((mCurPassatCarState.gearStatus.chosenPos() == aa::modules::io::passat::GearStatus::Pos_R) &&
			 (gearPosition == CarState::GEAR_REVERSE))) {
		mGear = gearPosition;
		mInForwardOrBackwardDriveEngaging = false;
		//COUT("GearCorrect");
		return;
	}

	//Else
	mInForwardOrBackwardDriveEngaging = true;


	//if ((mCurPassatCarState.wheelSpeeds.speedAvg() <= 3) && () {

	mSpeed = mSoftBrake;  //will be overridden, if mAllowDriverIntervention is set true, because after this, MiG lands in neutral -> fix this to be able to do back and forward maneuvers

	//}

	//measureBraking Pressure && measureWheelSpeed
	if ((mCurPassatCarState.wheelSpeeds.speedAvg() == 0) && (mCurPassatCarState.brakeStatus1.actualPressure() > (0.1 * 127.5)))	{
		if ((gearPosition == CarState::GEAR_REVERSE) || (gearPosition == CarState::GEAR_DRIVE || gearPosition == CarState::GEAR_SPORT)) {
			mGear = gearPosition;		//Shift to Drive
		}
		else {
			COUT("ERROR: INVALID GEAR SET, ONLY R OR D ARE VALID");
		}
	}

	//if Driver Override...
}


void ControllerPassat::disengageGear()
{

	mSpeed = mSoftBrake;

	if ((mCurPassatCarState.wheelSpeeds.allWheelsStopped()) && (mCurPassatCarState.brakeStatus1.actualPressure() > (mSoftBrake * 0.25 * 127.5))) {
		if ((count % 200) == 0) {
			COUT("PassatController::disengageGear(): Braking-Pressure reached and Wheels still");
		}

		mGear = CarState::GEAR_PARK;		//Park

		if (mCurPassatCarState.gearStatus.chosenPos() == aa::modules::io::passat::GearStatus::Pos_P_Key_Lock_Release) {
			//COUT("PassatController::disengageGear():  Gear Disengaged");
		}
	}
}


void ControllerPassat::updateHook()
{
	AbstractController::updateHook();

	Logger::In in("ControllerPassat");

	if (!mSportySet.rvalue()) {
		mMaxThrottle = mMaxThrottleComfort.rvalue();
		mMinBrake = mMinBrakeComfort.rvalue();
		mGearWish = 8;
	}
	else {
		if (!mUltimateSet.rvalue()) {
			mMaxThrottle = mMaxThrottleSporty.rvalue();
			mMinBrake = mMinBrakeSporty.rvalue();
		}
		else {
			mMaxThrottle = mMaxThrottleUltimate.rvalue();
			mMinBrake = mMinBrakeUltimate.rvalue();
		}

		mGearWish = 16;
	}

	if (mShiftBackward) {
		mGearWish = 2; //Backward
	}


	// ----> Engaging with Flashlight, disengaging with DriverIntervention (e.g. throttle Brake Pressure)
	if (mCurPassatCarState.signalWipersStatus.valid() && mCurPassatCarState.signalWipersStatus.flashLight()) {
		mCyclesFlashLightsOn++;
	}
	else {
		if (mCyclesFlashLightsOn >= 1) {  //1/100 second
			mEngage = !mEngage;
		}

		mCyclesFlashLightsOn = 0;
	}

//	if (mCurPassatCarState.watchdogStatus.powerOffReason() != 0) {
//		mCyclesFlashLightsOn = 0;
//		mEngage = false;
//	}
	// <---- Engaging ends

	// ----> Learning starts
	if (mLearnHumanControllerParameters.rvalue()) {
		if (!mInitializedLearningParams) {
			initializeHumanControllerLearning();
			mInitializedLearningParams = true;
		}

		learnHumanControllerParameters();
	}

	if (mApplyHumanControllerParameters.rvalue()) {
		mLearnHumanControllerParameters.set(false);
		mApplyHumanControllerParameters.set(false);
		applyHumanControllerParameters();
		mInitializedLearningParams = false;
	}

	// ----> Learning ends


	now.stamp();
	ts = 1E-6f * RTT::os::TimeService::ticks2nsecs(now - mZeroTime);


	mPackethistory[mPackethistoryIdx % 100] = ts;

	if (mPackethistoryIdx % 100 == 0) {
		mFreq = 100000.0 / (mPackethistory[0] - mPackethistory[1]);

		if (mFreq < 95.) {
			ERROR("==============================");
			ERROR("ControllerPassat Freq < 95hz: " << mFreq);
			ERROR("==============================");
		}

	}

	mPackethistoryIdx = (mPackethistoryIdx + 1) % 100;

	count++;
}

void ControllerPassat::stopHook()
{
	os.close();
	AbstractController::stopHook();
}


void ControllerPassat::normalizeAngle(flt & angle)
{
	if (angle <= (-M_PI)) {
		angle += (2 * M_PI);
	}
	else if (angle > M_PI) {
		angle -= (2 * M_PI);
	}
}

flt ControllerPassat::onlineCalibrateIMU()
{

	if (mIMUCyclesOnlineCalibration >= 400) {
		mIMUCyclesOnlineCalibration = 0;

		if (mIMUErrorIntegral / 400 >= 0.01) {
			COUT("Current Error in mm: " << mIMUErrorIntegral / 0.4);
			mIMUErrorIntegral = 0;
			return 0.01;
		}
		else if (mIMUErrorIntegral / 400 <= -0.01) {
			COUT("Current Error in mm: " << mIMUErrorIntegral / 0.4);
			mIMUErrorIntegral = 0;
			return -0.01;
		}
		else {
			mIMUOnlineCalibrate = false;
			mIMUErrorIntegral = 0;
			COUT("Finishing Online calibration, Lateral Error < 1 cm");
			return 0;
		}
	}

	mIMUErrorIntegral += mSignedLateralError;
	mIMUCyclesOnlineCalibration++;
	return 0;
}


/**********************************************************************************************
* Learning Methods
*
***********************************************************************************************/
void ControllerPassat::initializeHumanControllerLearning()
{
	for (int i = 0; i < 100; i++) {
		mThrottleHistogram[i] = 0.0;
		mStartRollingThrottleHistogram[i] = 0;
		mBrakeHistogram[i] = 0.0;
		mCentrifugalForceHistogram[i] = 0;

	}
}

void ControllerPassat::learnHumanControllerParameters()
{
	flt index;
	//Throttle and StartRollingThrottle
	index = ((rangeCut(0.7, mCurPassatCarState.throttleStatus.voltageMotor1(), 4.0) - 0.75) / (4.0 - 0.75)) * 99.9;

	if ((index < 0) || (index >= 100)) {
		COUT("Index throttle out of bounds " << mCurPassatCarState.throttleStatus.voltageMotor1());
	}
	else {
		mThrottleHistogram[(int)index]++;

		if (fabs(mCurPassatCarState.wheelSpeeds.speedFrontAvg() <= 3.0)) {
			mStartRollingThrottleHistogram[(int)index]++;
		}
	}

	//brake
	index = ((rangeCut(0.0, mCurPassatCarState.brakeStatus1.actualPressure(), 127.5) / 128)) * 100;

	if ((index < 0) || (index >= 100)) {
		COUT("Index brake out of bounds " << mCurPassatCarState.brakeStatus1.actualPressure());
	}
	else {
		mBrakeHistogram[(int)index]++;
	}

	//centrifugalforce
	index = (rangeCut(0.0, mCurPassatCarState.wheelSpeeds.speedAvg() * mCurPassatCarState.wheelSpeeds.speedAvg() * fabs(sin(mCurPassatCarState.steerAssist3Status.steerAngle() / 530.0 * 33.9 * D2R) / 2.709), 10.0)) * 10.0;

	if ((index < 0) || (index >= 100)) {
		COUT("Index centrifugalforce out of bounds, Vel: " << mCurPassatCarState.wheelSpeeds.speedAvg() << " SWA Deg: " << mCurPassatCarState.steerAssist3Status.steerAngle());
	}
	else {
		mCentrifugalForceHistogram[(int)index]++;
	}

//	mVelocityHistogram[100];
}

void ControllerPassat::applyHumanControllerParameters()
{
	long sumOfThrottleHistogram = 0.0;
	long sumOfStartRollingThrottleHistogram = 0.0;
	long sumOfBrakeHistogram = 0.0;
	long sumOfCentrifugalForceHistogram = 0.0;

	long findThrottlePercentilValue = 0.0;
	long findStartRollingThrottlePercentilValue = 0.0;
	long findBrakePercentilValue = 0.0;
	long findCentrifugalForcePercentilValue = 0.0;

	for (int i = 0; i < 100; i++) {
		sumOfThrottleHistogram += mThrottleHistogram[i];
		sumOfStartRollingThrottleHistogram += mStartRollingThrottleHistogram[i];
		sumOfBrakeHistogram += mBrakeHistogram[i];
		sumOfCentrifugalForceHistogram += mCentrifugalForceHistogram[i];
	}

	//****************************** find index
	//1: Throttle
	for (int i = 0; i < 100; i++) {
		findThrottlePercentilValue += mThrottleHistogram[i];

		if (findThrottlePercentilValue >= sumOfThrottleHistogram/** 0.8 */) {
			mLearnedMaxThrottle = 0.01 * i;
			break;
		}
	}

	//2: StartRollingThrottle
	for (int i = 0; i < 100; i++) {
		findStartRollingThrottlePercentilValue += mStartRollingThrottleHistogram[i];

		if (findStartRollingThrottlePercentilValue >= sumOfStartRollingThrottleHistogram/** 0.8 */) {
			mLearnedStartRollingThrottle = 0.01 * i;
			break;
		}
	}

	//3: Brake
	for (int i = 0; i < 100; i++) {
		findBrakePercentilValue += mBrakeHistogram[i];

		if (findBrakePercentilValue >= sumOfBrakeHistogram/** 0.8 */) {
			mLearnedMinBrake = -0.01 * i;
			break;
		}
	}

	//4: Centrifugal Force
	for (int i = 0; i < 100; i++) {
		findCentrifugalForcePercentilValue += mCentrifugalForceHistogram[i];

		if (findCentrifugalForcePercentilValue >= sumOfCentrifugalForceHistogram/** 0.8 */) {
			mLearnedCentrifugalForce = 0.1 * i;
			break;
		}
	}


	COUT("Learned MaxThrottle: " << mLearnedMaxThrottle << " applying ... ");
	COUT("Learned StartRollingThrottle: " << mLearnedStartRollingThrottle << " applying ... ");
	COUT("Learned MinBrake: " << mLearnedMinBrake << " applying ... ");
	COUT("Learned MaxCentrifualForce: " << mLearnedCentrifugalForce << " applying ... ");


	mMaxThrottle = mLearnedMaxThrottle;
	mStartRollingThrottleValue = mLearnedStartRollingThrottle;
	mMinBrake = mLearnedMinBrake;
	mComfortSettings.mCentrifugalAccelerationComfort = mLearnedCentrifugalForce;
}


//NN Learning Part Starts
void ControllerPassat::initializeNNSteering()
{
	weights[0][0] = 0;
	weights[1][0] = 0;
	weights[2][0] = 0;
}

void ControllerPassat::learnNNSteering(Vec2 p0, Vec2 p1, Vec2 p2, Vec3 carPos, Vec3 carDirVec, flt steerAction)
{


}



void ControllerPassat::activateNNSteering() {}
void ControllerPassat::backPropagateNNSteering() {}
flt ControllerPassat::applyNNSteering()
{
	return 0;
}
void ControllerPassat::normalizePointsToCarPosAndDir() {}
//NN Learning Part Ends

}
}
}
}
