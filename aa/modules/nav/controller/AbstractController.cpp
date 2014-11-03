#include "AbstractController.h"

#include <patterns/Singleton.h>
#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <util/PrettyPrint.h>
#include <math/AutoMath.h>
#include <math/PathSpline.h>
#include <math/DiffGeom.h>
#include <data/VehicleData.h>

#include <aa/modules/nav/obstacles/ObstaclePrediction.h>
#include <aa/modules/nav/statemachine/StateMachine.h>
#include <aa/data/obstacle/util/Location.h>

using ::modules::models::carstate::CarState;
using ::modules::models::carstate::TimedCarState;

namespace aa
{
namespace modules
{
namespace nav
{
namespace controller
{


using namespace std;
using namespace RTT;
using namespace boost;
using namespace ::math;
using namespace ::util;
using namespace aa::data::obstacle;
using namespace ::aa::data::obstacle::util;
using namespace modules::models::carstate;
using namespace aa::modules::models::rndf;
using namespace aa::modules::nav::obstacles;
using RTT::Logger;



AbstractController::AbstractController(string const & name)
	: ::util::RtTaskContext(name)

	// read ports
	, mPlanIn("PlanIn")
	, mEgoStateIn("EgoStateIn")
	, mCarStateIn("CarStateIn")
	, mPassatCarStateIn("PassatCarStateIn")
	, mObstaclesIn("ObstaclesIn")
	, mResumeDriveIn("ResumeDriveIn")
	, mSideObstacleDistLeftIn("SideObstacleDistLeftIn")
	, mSideObstacleDistRightIn("SideObstacleDistRightIn")

	// write ports
	, mSteerOut("SteerOut")
	, mSpeedOut("SpeedOut")
	, mGearOut("GearOut")
	, mAuxDevicesOut("AuxDevicesOut")
	, mActivationRequestOut("ActivationRequestOut")

	, mWantedSpeedOut("WantedSpeedOut")
	, mWantedSpeedByObstaclesOut("WantedSpeedByObstaclesOut")
	, mDistToCrashOut("DistToCrashOut")
	, mControllerDataOut("ControllerDataOut")
	, mInterferingObstaclesOut("InterferingObstaclesOut")

	, mBrakingObstacleIdOut("BrakingObstacleIdOut")

	, mResetRequestOut("ResetRequestOut")	//for learning


	// general properties
	, mEngage("Engage", "Activate Control ", false)
	, mMaxBrake("MaxBrake", "maximum allowed brake value (throttle position)", -1.0)
	, mMaxAccel("MaxAccel", "maximum allowed acceleration value (throttle position)", 1.0)
	, mMaxLeftSteer("MaxLeftSteer", "maximum allowed left steer value", -1.0)
	, mMaxRightSteer("MaxRightSteer", "maximum allowed right steer value", 1.0)
	, mMinSpeed("MinSpeed", "minimum allowed speed in m/s", 0 * KMH_2_MS)
	, mMaxSpeed("MaxSpeed", "maximum allowed speed in m/s", 160 * KMH_2_MS)
	, mMinZoneSpeed("MinZoneSpeed", "minimum allowed speed in zones in m/s", 0 * KMH_2_MS)
	, mMaxZoneSpeed("MaxZoneSpeed", "maximum allowed speed in zones in m/s", 20 * KMH_2_MS)
	, mWalkingSpeed("WalkingSpeed", "walking speed", 4.0 * KMH_2_MS)
	, mStopSpeed("StopSpeed", "speed at which full stop can be engaged", 0.25 * KMH_2_MS)
	, mFullStopSpeed("FullStopSpeed", "speed that indicates full stop", 0)
	, mUseMissionSpeedLimits("UseMissionSpeedLimits", "enable or disable mission to retrieve speed limits", true)
	, mSimulationMode("SimulationMode", "flag to check whether the controller is acting in simulation environment or not", false)

	// look ahead function: min(mLookAheadMax, mLookAheadSlope*speed)
	, mLookAheadSlope("LookAheadSlope", "slope of look ahead function proportional to speed in m/s", 0.7)
	, mLookAheadMax("LookAheadMax", "max of look ahead function", 6.0)

	// speed proposal properties
	// curvature function: mCurvatureSlope * radius + mCurvatureMin
	, mCurvatureSlope("CurvatureSlope", "slope of curvature function proportional to radius", 1.0 / 6.0)
	, mCurvatureMin("CurvatureMin", "wanted speed weight for curvature radius", 3 * KMH_2_MS)
	, mSampleCurvatureDistance(2.0)


	// free track width: max(mMinFreeTrackWidth, carWidth + 2*mFreeMargin)
	, mMinFreeTrackWidth("MinFreeTrackWidth", "minimal width of the track in m, which must be free of obstacles", 2.40)
	, mFreeMargin("FreeMargin", "free margin around car", 0.25)

	// throttle by speed window
	, mUseThrottleBySpeedWindow("UseThrottleBySpeedWindow", "enable or disable throttle by speed window", true)
	, mThrottleBySpeedWindowStart("ThrottleBySpeedWindowStart", "speed at which throttle by speed window starts", 10 * KMH_2_MS)
	, mThrottleBySpeedWindowEnd("ThrottleBySpeedWindowEnd", "speed at which throttle by speed window ends", 80 * KMH_2_MS)
	, mMaxBrakeBeforeThrottleBySpeedWindow("MaxBrakeBeforeThrottleBySpeedWindow", "max allowed brake position before throttle by speed window begins", -0.55)
	, mMaxAccelBeforeThrottleBySpeedWindow("MaxAccelBeforeThrottleBySpeedWindow", "max allowed acceleration position before throttle by speed window begins", 0.9)
	, mMaxBrakeAfterThrottleBySpeedWindow("MaxBrakeAfterThrottleBySpeedWindow", "max allowed brake position after throttle by speed window", -0.7)
	, mMaxAccelAfterThrottleBySpeedWindow("MaxAccelAfterThrottleBySpeedWindow", "max allowed acceleration position after throttle by speed window", 0.7)
	, mMaxAccelStartThrottleBySpeed("MaxAccelStartThrottleBySpeed", "max allowed acceleration value for starting car", 0.65)

	// steering by speed window
	, mUseSteerBySpeedWindow("UseSteerBySpeedWindow", "enable or disable steering by speed window", true)
	, mSteerBySpeedWindowStart("SteerBySpeedWindowStart", "speed at which steering by speed window starts", 10 * KMH_2_MS)
	, mSteerBySpeedWindowEnd("SteerBySpeedWindowEnd", "speed at which steering by speed window ends", 80 * KMH_2_MS)
	, mMaxLeftSteerBeforeSteerBySpeedWindow("MaxLeftSteerBeforeSteerBySpeedWindow", "max allowed left steer value before steering by speed window begins", -1.0)
	, mMaxRightSteerBeforeSteerBySpeedWindow("MaxRightSteerBeforeSteerBySpeedWindow", "max allowed right steer value before steering by speed window begins", 1.0)
	, mMaxLeftSteerAfterSteerBySpeedWindow("MaxLeftSteerAfterSteerBySpeedWindow", "max allowed left steer value after steering by speed window", -0.1)
	, mMaxRightSteerAfterSteerBySpeedWindow("MaxRightSteerAfterSteerBySpeedWindow", "max allowed right steer value after steering by speed window", 0.1)

	// throttle by steer window (positive only)
	, mUseThrottleBySteerWindow("UseThrottleBySteerWindow", "enable or disable throttle by steer window", true)
	, mThrottleBySteerWindowStart("ThrottleBySpeedWindowStart", "steering at which throttle by steer window starts", 0)
	, mThrottleBySteerWindowEnd("ThrottleBySpeedWindowEnd", "speed at which throttle by steer window ends", 0.85)
	, mMaxBrakeBeforeThrottleBySteerWindow("MaxBrakeBeforeThrottleBySpeedWindow", "max allowed brake position before throttle by steer window begins", -1.0)
	, mMaxAccelBeforeThrottleBySteerWindow("MaxAccelBeforeThrottleBySpeedWindow", "max allowed acceleration position before throttle by steer window begins", 1.0)
	, mMaxBrakeAfterThrottleBySteerWindow("MaxBrakeAfterThrottleBySpeedWindow", "max allowed brake position after throttle by steer window", -1.0)
	, mMaxAccelAfterThrottleBySteerWindow("MaxAccelAfterThrottleBySpeedWindow", "max allowed acceleration position after throttle by steer window", 0.59)


	//goal handling
	, mStopAtGoal("StopAtGoal", "enable or disable stops at goal", true)
	, mGoalSafetyDist("GoalSafetyDist", "safety distance before goal in m", 2.0)
	, mGoalRollingDist("GoalRollingDist", "distance for rolling speed before goal in m", 1.0)
	, mGoalRollingSpeed("GoalRollingSpeed", "speed during rolling phase to goal in m/s", 1.51  * KMH_2_MS)
	, mGoalMinLookaheadDist("GoalMinLookaheadDist", "minimum lookahead distance for goalin m", 10.00)

	//end spline handling
	, mStopAtEndSpline("StopAtEndSpline", "enable or disable stops at end of spline", true)
	, mEndSplineSafetyDist("EndSplineSafetyDist", "safety distance before end of spline in m", 3.0)
	, mEndSplineRollingDist("EndSplineRollingDist", "distance for rolling speed before end of spline in m", 3.0)
	, mEndSplineRollingSpeed("EndSplineRollingSpeed", "speed during rolling phase to end of spline in m/s", 2.05  * KMH_2_MS)
	, mEndSplineMinLookaheadDist("EndSplineMinLookaheadDist", "minimum lookahead distance for end of spline in m", 10.00)       //20.00

	//traffic light handling
	, mStopAtTrafficLight("StopAtTrafficLight", "enable or disable stops at traffic lights", true)
	, mTrafficLightSafetyDist("TrafficLightSafetyDist", "safety distance before traffic lights in m", 6.0) //shall be below the clearing radius
	, mTrafficLightRollingDist("TrafficLightRollingDist", "distance for rolling speed before traffic lights in m", 0.5)
	, mTrafficLightRollingSpeed("TrafficLightRollingSpeed", "speed during rolling phase to traffic lights in m/s", 3.14 * KMH_2_MS)
	, mTrafficLightMinLookaheadDist("TrafficLightMinLookaheadDist", "minimum lookahead distance for traffic lights in m", 20.0)
	, mTrafficLightPointOfNoReturnRatio("TrafficLightPointOfNoReturnRatio", "ratio of point of no return in front of traffic light and brake distance", 0.42) //0.5 - Daniel 2011-08-11
	, mTrafficLightBrakeInFrontPointOfNoReturn(false)

	//stop sign handling
	, mStopAtStopSign("StopAtStopSign", "enable or disable stops at stop signs", true)
	, mStopSignSafetyDist("StopSignSafetyDist", "safety distance before stop signs in m", 1.50) //shall be below the clearing radius
	, mStopSignRollingDist("StopSignRollingDist", "distance for rolling speed before stop signs in m", 2)	//5
	, mStopSignRollingSpeed("StopSignRollingSpeed", "speed during rolling phase to stop sign in m/s", 2.01 * KMH_2_MS) //1 km/h
	, mStopSignMinLookaheadDist("StopSignMinLookaheadDist", "minimum lookahead distance for stop signs in m", 10.00)

	//give way handling
	, mStopAtGiveWay("StopAtGiveWay", "enable or disable stops before give way", true)
	, mGiveWaySafetyDist("GiveWaySafetyDist", "safety distance before give way in m", 2.00)
	, mGiveWayRollingDist("GiveWayRollingDist", "distance for rolling speed before give way in m", 0.0)
	, mGiveWayRollingSpeed("GiveWayRollingSpeed", "speed during rolling phase to give way in m/s", 1.01 * KMH_2_MS)
	, mGiveWayMinLookaheadDist("GiveWayMinLookaheadDist", "minimum lookahead distance for give ways in m", 10.00)

	//reverse point handling
	, mStopAtReversePoint("StopAtReversePoint", "enable or disable stops at reverse points", true)
	, mReversePointSafetyDist("ReversePointSafetyDist", "safety distance before reverse points in m", 0.05)
	, mReversePointRollingDist("ReversePointRollingDist", "distance for rolling speed before reverse points in m", 3.00)	//5
	, mReversePointRollingSpeed("ReversePointRollingSpeed", "speed during rolling phase to reverse point in m/s", 1 * KMH_2_MS)
	, mReversePointMinLookaheadDist("ReversePointMinLookaheadDist", "minimum lookahead distance for reverse points in m", 4.00)

	//decision point handling
	, mStopAtDecisionPoint("StopAtDecisionPoint", "enable or disable stops at decision points", true)
	, mDecisionPointSafetyDist("DecisionPointSafetyDist", "safety distance before decision points in m", 1.50) //shall be below the clearing radius
	, mDecisionPointRollingDist("DecisionPointRollingDist", "distance for rolling speed before decision points in m", 2.00)	//5
	, mDecisionPointRollingSpeed("DecisionPointRollingSpeed", "speed during rolling phase to decision point in m/s", 2.01 * KMH_2_MS) //1 km/h
	, mDecisionPointMinLookaheadDist("DecisionPointMinLookaheadDist", "minimum lookahead distance for decision points in m", 10.00)

	//obstacle handling
	, mUseObstacles("UseObstacles", "enable or disable obstacle processing", true)
	, mCheckForObstacleMalfunction("CheckForObstacleMalfunction", "enable or disable check for malfunction in obstacle detection", false)
	, mObstacleSafetyDist("ObstacleSafetyDist", "safety distance before obstacles in m", 12.0)		//shall be above the swerve radius of Behaviour
	, mObstacleRollingDist("ObstacleRollingDist", "distance for rolling speed before obstacles in m", 6.0)
	, mObstacleRollingSpeed("ObstacleRollingSpeed", "speed during rolling phase to obstacles in m/s", 1.03)
	, mFollowSafetyDist("FollowSafetyDist", "Minimum Distance to follow in m", 8.0)	//used to be 12.0			//shall be above the swerve radius of Behaviour
	, mUseObstacleSpeed("UseObstacleSpeed", "use obstacle speed for follow velocity", true)
	, mSportySet("SportySet", "parameter set for sporty ", false)
	, mUltimateSet("UltimateSet", "parameter set for ultimate, requires sporty set ", false)
	, mSoftBrakeTrajectory("SoftBrakeTrajectory", "Flag to use new braking trajectory", true)
	, mSoftBrakeTrajectoryExp(0.57)  //0.6666 - Daniel 2011-08-11
	, mSoftBrakeTrajectoryLin(-0.65)   //-0.3 - Daniel 2011-08-11

	// constant throttle controller properties
	, mConstantThrottle("ConstantThrottle", "wanted constant throttle to be maintained", 0.25)

	// constant speed controller properties
	, mConstantSpeed("ConstantSpeed", "wanted constant speed to be maintained in m/s", 30 * KMH_2_MS)

	// constant steer controller properties
	, mConstantSteer("ConstantSteer", "wanted constant steer value", 0.0)

	// driver intenvention handling (Passat Only)
	, mAllowDriverIntervention("AllowDriverIntervention", "Allows Driver Intervention, shifts back without braking", true)

	// turn signals
	, mTurnSignalTime("TurnSignalTime", "time before turn to activate turn signal in s", 4.0)
	, mMinTurnSignalDistance("MinTurnSignalDistance", "minimal distance to activate turn signal before turn in m", 5.0)


	, mCounter("Counter", 0)
    , mARNDGraph(patterns::Singleton<RNDFGraph>::instance())
	, mComfortSettings(patterns::Singleton<ComfortSettings>::instance())
	, lastVelocityFromCurvatureAndRoadSignPlanning(0.0)


	//Log
	, mLogWantedSpeed("LogWantedSpeed", "enable or disable logging of wanted speed into file", false)


	// Debug
	, mDebugPrintWantedSpeedIntermediateSteps("DebugPrintWantedSpeedIntermediateSteps", false)
	, mDebug1("Debug1", "Plot Variable 1", 0.0)
	, mDebug2("Debug2", "Plot Variable 2", 0.0)
	, mDebug3("Debug3", "Plot Variable 3", 0.0)
	, mPrintObstacleInfo("PrintObstacleInfo", "Prints Info about Obstacles to Follow", false)

	// Predict movement
	, mPredictUnknownObstacles("PredictUnknownObstacles", "Predict movement of obstacles if there is no new data", false)
	, mPredictUnknownObstaclesThreshold("PredictUnknownObstaclesThreshold", "How old must the scanner data be before it will be predicted (Ticks)", 1e9 / 25.0) //not older than two egostates
	, mNumberOfCarStatePorts(0)
{
	ports()->addPort(mPlanIn);
	ports()->addPort(mEgoStateIn);
	ports()->addPort(mCarStateIn);
	ports()->addPort(mPassatCarStateIn);
	ports()->addPort(mObstaclesIn);
	ports()->addPort(mResumeDriveIn);
	ports()->addPort(mSideObstacleDistLeftIn);
	ports()->addPort(mSideObstacleDistRightIn);


	ports()->addPort(mSteerOut);
	ports()->addPort(mSpeedOut);
	ports()->addPort(mGearOut);
	ports()->addPort(mAuxDevicesOut);
	ports()->addPort(mActivationRequestOut);

	ports()->addPort(mWantedSpeedOut);
	ports()->addPort(mWantedSpeedByObstaclesOut);
	ports()->addPort(mDistToCrashOut);
	ports()->addPort(mControllerDataOut);
	ports()->addPort(mInterferingObstaclesOut);

	ports()->addPort(mBrakingObstacleIdOut);

	ports()->addPort(mResetRequestOut);


	addAttribute(mCounter);
	addAttribute(mDebugPrintWantedSpeedIntermediateSteps);


	addProperty(mEngage);
	addProperty(mMaxBrake);
	addProperty(mMaxAccel);
	addProperty(mMaxLeftSteer);
	addProperty(mMaxRightSteer);
	addProperty(mMinSpeed);
	addProperty(mMaxSpeed);
	addProperty(mMinZoneSpeed);
	addProperty(mMaxZoneSpeed);
	addProperty(mWalkingSpeed);
	addProperty(mStopSpeed);
	addProperty(mFullStopSpeed);
	addProperty(mUseMissionSpeedLimits);
	addProperty(mSimulationMode);


	addProperty(mLookAheadSlope);
	addProperty(mLookAheadMax);

	addProperty(mMinFreeTrackWidth);
	addProperty(mFreeMargin);

	addProperty(mCurvatureSlope);
	addProperty(mCurvatureMin);

	addProperty(mUseThrottleBySpeedWindow);
	addProperty(mMaxBrakeBeforeThrottleBySpeedWindow);
	addProperty(mMaxAccelBeforeThrottleBySpeedWindow);
	addProperty(mThrottleBySpeedWindowStart);
	addProperty(mThrottleBySpeedWindowEnd);
	addProperty(mMaxBrakeAfterThrottleBySpeedWindow);
	addProperty(mMaxAccelAfterThrottleBySpeedWindow);
	addProperty(mMaxAccelStartThrottleBySpeed);

	addProperty(mUseSteerBySpeedWindow);
	addProperty(mMaxLeftSteerBeforeSteerBySpeedWindow);
	addProperty(mMaxRightSteerBeforeSteerBySpeedWindow);
	addProperty(mSteerBySpeedWindowStart);
	addProperty(mSteerBySpeedWindowEnd);
	addProperty(mMaxLeftSteerAfterSteerBySpeedWindow);
	addProperty(mMaxRightSteerAfterSteerBySpeedWindow);


	addProperty(mUseThrottleBySteerWindow);
	addProperty(mThrottleBySteerWindowStart);
	addProperty(mThrottleBySteerWindowEnd);
	addProperty(mMaxBrakeBeforeThrottleBySteerWindow);
	addProperty(mMaxAccelBeforeThrottleBySteerWindow);
	addProperty(mMaxBrakeAfterThrottleBySteerWindow);
	addProperty(mMaxAccelAfterThrottleBySteerWindow);



	addProperty(mStopAtGoal);
	addProperty(mGoalSafetyDist);
	addProperty(mGoalRollingDist);
	addProperty(mGoalRollingSpeed);
	addProperty(mGoalMinLookaheadDist);

	addProperty(mStopAtEndSpline);
	addProperty(mEndSplineSafetyDist);
	addProperty(mEndSplineRollingDist);
	addProperty(mEndSplineRollingSpeed);
	addProperty(mEndSplineMinLookaheadDist);

	addProperty(mStopAtTrafficLight);
	addProperty(mTrafficLightSafetyDist);
	addProperty(mTrafficLightRollingDist);
	addProperty(mTrafficLightRollingSpeed);
	addProperty(mTrafficLightMinLookaheadDist);
	addProperty(mTrafficLightPointOfNoReturnRatio);

	addProperty(mStopAtStopSign);
	addProperty(mStopSignSafetyDist);
	addProperty(mStopSignRollingDist);
	addProperty(mStopSignRollingSpeed);
	addProperty(mStopSignMinLookaheadDist);

	addProperty(mStopAtGiveWay);
	addProperty(mGiveWaySafetyDist);
	addProperty(mGiveWayRollingDist);
	addProperty(mGiveWayRollingSpeed);
	addProperty(mGiveWayMinLookaheadDist);

	addProperty(mStopAtReversePoint);
	addProperty(mReversePointSafetyDist);
	addProperty(mReversePointRollingDist);
	addProperty(mReversePointRollingSpeed);
	addProperty(mReversePointMinLookaheadDist);

	addProperty(mStopAtDecisionPoint);
	addProperty(mDecisionPointSafetyDist);
	addProperty(mDecisionPointRollingDist);
	addProperty(mDecisionPointRollingSpeed);
	addProperty(mDecisionPointMinLookaheadDist);


	addProperty(mUseObstacles);
	addProperty(mCheckForObstacleMalfunction);
	addProperty(mObstacleSafetyDist);
	addProperty(mObstacleRollingDist);
	addProperty(mObstacleRollingSpeed);
	addProperty(mFollowSafetyDist);
	addProperty(mUseObstacleSpeed);
	addProperty(mSportySet);
	addProperty(mUltimateSet);
	addProperty(mSoftBrakeTrajectory);

	addProperty(mConstantThrottle);
	addProperty(mConstantSpeed);
	addProperty(mConstantSteer);

	addProperty(mAllowDriverIntervention);

	addProperty(mTurnSignalTime);
	addProperty(mMinTurnSignalDistance);


	addProperty(mLogWantedSpeed);

	addProperty(mDebug1);
	addProperty(mDebug2);
	addProperty(mDebug3);

	addProperty(mPredictUnknownObstacles);
	addProperty(mPredictUnknownObstaclesThreshold);

	addProperty(mPrintObstacleInfo);


	for (PropertyBag::const_iterator it = mComfortSettings.begin(); it != mComfortSettings.end(); ++it) {
		addProperty(**it);
	}

#if 0
	/* must be copied in updatehook, if we want to avoid this redundancy we would have to use the mComfortSettings-object (use prefix mComfortSettings. ...) */
	mCentrifugalAccelerationComfort = mComfortSettings.mCentrifugalAccelerationComfort.get();
	mThrottleAccelerationComfort = mComfortSettings.mThrottleAccelerationComfort.get();
	mBrakingAccelerationComfort = mComfortSettings.mBrakingAccelerationComfort.get();
	mBrakingAccelerationForObstacles = mComfortSettings.mBrakingAccelerationForObstacles.get();
	mBrakingAccelerationForCurveEntries = mComfortSettings.mBrakingAccelerationForCurveEntries.get();
	mSampleCurvatureDistance = mComfortSettings.mSampleCurvatureDistanceComfort.get();
#endif

	addOperation("ResumeDrive", &AbstractController::resumeDrive, this, RTT::ClientThread).doc("go back to drive state");
	addOperation("PauseDrive", &AbstractController::pauseDrive, this, RTT::ClientThread).doc("go into stop state");
	addOperation("ResetProcessedActions", &AbstractController::resetProcessedActions, this, RTT::ClientThread).doc("reset processed actions");

	addOperation("ForceTrafficLightRed", &AbstractController::forceTrafficLightRed, this, RTT::ClientThread).doc("Sets Red Light in Controller.");
	addOperation("ForceTrafficLightGreen", &AbstractController::forceTrafficLightGreen, this, RTT::ClientThread).doc("Sets Green Light in Controller.");


	mSpeedControlMode = VARIABLE_SPEED;

	addOperation("UseFullStopMode", &AbstractController::useFullStopMode, this, RTT::ClientThread).doc("constant full stop");
	addOperation("UseConstantThrottleMode", &AbstractController::useConstantThrottleMode, this, RTT::ClientThread).doc("constant throttle");
	addOperation("UseConstantSpeedMode", &AbstractController::useConstantSpeedMode, this, RTT::ClientThread).doc("controlled constant speed");
	addOperation("UseVariableSpeedMode", &AbstractController::useVariableSpeedMode, this, RTT::ClientThread).doc("controlled variable speed");


	mSteerControlMode = VARIABLE_STEER;

	addOperation("UseConstantSteerMode", &AbstractController::useConstantSteerMode, this, RTT::ClientThread).doc("controlled constant steering");
	addOperation("UseVariableSteerMode", &AbstractController::useVariableSteerMode, this, RTT::ClientThread).doc("controlled variable steering");

	addOperation("printOutput", &AbstractController::printOutput, this, RTT::ClientThread).doc("print current output variables");

	now.stamp();
}


AbstractController::~AbstractController()
{}


bool AbstractController::startHook()
{
	Logger::In in("Controller");

	mStateMachine = findPeer(this, "StateMachine");

	if (!mStateMachine) {
		logError() << "missing StateMachine peer";
		return false;
	}

	REQUIRED_PORT(mPlanIn);
	REQUIRED_PORT(mEgoStateIn);
	REQUIRED_PORT(mCarStateIn);
	OPTIONAL_PORT(mPassatCarStateIn);
	OPTIONAL_PORT(mObstaclesIn);
	OPTIONAL_PORT(mResumeDriveIn);

	REQUIRED_PORT(mSteerOut);
	REQUIRED_PORT(mSpeedOut);
	OPTIONAL_PORT(mGearOut);
	OPTIONAL_PORT(mAuxDevicesOut);
	OPTIONAL_PORT(mActivationRequestOut);

	OPTIONAL_PORT(mWantedSpeedOut);
	OPTIONAL_PORT(mControllerDataOut);
// 	OPTIONAL_PORT(mSpeedRampOut);
	OPTIONAL_PORT(mInterferingObstaclesOut);

	OPTIONAL_PORT(mBrakingObstacleIdOut);

	OPTIONAL_PORT(mResetRequestOut);
	OPTIONAL_PORT(mSideObstacleDistLeftIn);
	OPTIONAL_PORT(mSideObstacleDistRightIn);

	return true;
}

void AbstractController::updateHook()
{
	Logger::In in("Controller");

	mCounter.set(mCounter.get() + 1);

	now.stamp();

	TimedFlt speedCorrection;
	speedCorrection.stamp();
	flt steerCorrection;
	AuxDevicesData auxDevicesCorrection;

	// read egostate port
	mEgoStateIn.read(mCurEgoState);

	if (mCurEgoState == TimeStamp()) {
		if (mCounter.get() % 100 == 0) {
			logWarning() << "Invalid or no EgoState";
		}

		return;
	}


	flt curEgoStateAge = 1E-9f * RTT::os::TimeService::ticks2nsecs(now - mCurEgoState);
/**
 * not usable when playing a log file... "now" is compared to the time the logfile was recorded
	if (curEgoStateAge > 5 * getPeriod()) {
		if (mCounter.get() % 50 == 0) {
			logWarning() << "Data from EgoStatePort is too old (" << curEgoStateAge << " s > " << 5 * getPeriod() << " s)";
		}
	}
*/

	mNumberOfCarStatePorts = 0;

	//read carstate ports
	if (mCarStateIn.connected()) {
		mCarStateIn.read(mCurCarState);
	}

	if (mPassatCarStateIn.connected()) {
		mPassatCarStateIn.read(mCurPassatCarState);
		mNumberOfCarStatePorts++;
	}


	if ((mNumberOfCarStatePorts > 1) && (!mSimulationMode.rvalue())) {
//		if (mCounter.get() % 100 == 0) {
//			logError()<<"Multiple Carstates Connected: " << mNumberOfCarStatePorts << " - Terminating Controller";
//		}

//		return;
	}


	// read trajectory port
	mPlanIn.read(mCurPlan);

	if (!mCurPlan) {
		if (mCounter.get() % 100 == 0) {
			logWarning() << "Got no plan";
		}

		return;
	}

	Plan const & curPlan = *mCurPlan;
	std::pair<flt, flt> const dom(curPlan.domain());

	if (dom.second == dom.first) {
		std::cout << "Got invalid plan";

		return;
	}

//	std::cout<<"plan: " << dom.first << " " << dom.second;




	// read obstacles port
	if (mUseObstacles.get() && mObstaclesIn.connected()) {
		mObstaclesIn.read(mCurObstacles);

		if (mCheckForObstacleMalfunction.get() && mCurObstacles) {
			flt const waitSecs = 1.0;
			flt const diffTime = 1E-9f * RTT::os::TimeService::ticks2nsecs(now - *mCurObstacles);

			if (diffTime > waitSecs) {
				if (mCounter.get() % 50 == 0) {
					logWarning() << "Got obsolete obstacles";
				}

				handleMalfunction("got obsolete obstacles");
			}
		}
	}






	//get statemachine functions
	OperationCaller<std::string(void)> getCurrentStateName(mStateMachine->getOperation("getCurrentStateName"));



	flt const frontShaftDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontShaftDistance")->value();
	flt const rearShaftDistance = ::data::theVehicleData::instance().getPropertyType<flt>("rearShaftDistance")->value();
	flt const frontDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontDistance")->value();
	flt const rearDistance = ::data::theVehicleData::instance().getPropertyType<flt>("rearDistance")->value();
	flt const carLength = ::data::theVehicleData::instance().getPropertyType<flt>("realCarLength")->value();

    Vec3 const imuPos = mCurEgoState.position();
    Vec3 const carDir = normalized(mCurEgoState.forwardDirection());
    Vec3 const carPos = imuPos + frontShaftDistance * carDir;
    Vec3 const carRearPos = imuPos + rearShaftDistance * carDir;
    Vec3 const carFrontPos = imuPos + (frontShaftDistance + frontDistance) * carDir;
    Vec3 const carBackPos = imuPos + (rearShaftDistance - rearDistance) * carDir;

	/* is a clone of the block in start hook */
	mCentrifugalAccelerationComfort = mComfortSettings.mCentrifugalAccelerationComfort.get();
	mThrottleAccelerationComfort = mComfortSettings.mThrottleAccelerationComfort.get();
	mBrakingAccelerationComfort = mComfortSettings.mBrakingAccelerationComfort.get();
	mBrakingAccelerationForObstacles = mComfortSettings.mBrakingAccelerationForObstacles.get();
	mBrakingAccelerationForCurveEntries = mComfortSettings.mBrakingAccelerationForCurveEntries.get();
	mSampleCurvatureDistance = mComfortSettings.mSampleCurvatureDistanceComfort.get();

	if (!mSportySet.rvalue()) {
		mCentrifugalAccelerationComfort = mComfortSettings.mCentrifugalAccelerationComfort.get();
		mThrottleAccelerationComfort = mComfortSettings.mThrottleAccelerationComfort.get();
		mBrakingAccelerationComfort = mComfortSettings.mBrakingAccelerationComfort.get();
		mBrakingAccelerationForObstacles = mComfortSettings.mBrakingAccelerationForObstacles.get();
		mBrakingAccelerationForCurveEntries = mComfortSettings.mBrakingAccelerationForCurveEntries.get();
		mSampleCurvatureDistance = mComfortSettings.mSampleCurvatureDistanceComfort.get();
	}
	else {
		if (!mUltimateSet.rvalue()) {
			mCentrifugalAccelerationComfort = mComfortSettings.mCentrifugalAccelerationSporty.get();
			mThrottleAccelerationComfort = mComfortSettings.mThrottleAccelerationSporty.get();
			mBrakingAccelerationComfort = mComfortSettings.mBrakingAccelerationSporty.get();
			mBrakingAccelerationForObstacles = mComfortSettings.mBrakingAccelerationForObstaclesSporty.get();
			mBrakingAccelerationForCurveEntries = mComfortSettings.mBrakingAccelerationForCurveEntriesSporty.get();
			mSampleCurvatureDistance = mComfortSettings.mSampleCurvatureDistanceSporty.get();
		}
		else {
			mCentrifugalAccelerationComfort = mComfortSettings.mCentrifugalAccelerationUltimate.get();
			mThrottleAccelerationComfort = mComfortSettings.mThrottleAccelerationUltimate.get();
			mBrakingAccelerationComfort = mComfortSettings.mBrakingAccelerationUltimate.get();
			mBrakingAccelerationForObstacles = mComfortSettings.mBrakingAccelerationForObstaclesUltimate.get();
			mBrakingAccelerationForCurveEntries = mComfortSettings.mBrakingAccelerationForCurveEntriesUltimate.get();
			mSampleCurvatureDistance = mComfortSettings.mSampleCurvatureDistanceUltimate.get();
		}
	}



	/// Step 0: create controller data
	if (!mRecentControllerData.empty()) {
		mLastControllerData = mRecentControllerData.front();
	}

	if (mRecentControllerData.empty() || dom.first == 0) {
		flt const range = 2 * carLength;

		tie(mClosestSqrDistToTrajectory, mClosestParamOnTrajectory) = findClosestPoint(curPlan, dom.first, dom.second, (dom.first + dom.second) / 2.0, carPos);
		mClosestPosOnTrajectory = curPlan(mClosestParamOnTrajectory);


		tie(mClosestSqrDistToTrajectory_Rear, mClosestParamOnTrajectory_Rear) = findClosestPoint(curPlan, rangeCut(dom, mClosestParamOnTrajectory - range), rangeCut(dom, mClosestParamOnTrajectory + range), mClosestParamOnTrajectory, carRearPos);
		mClosestPosOnTrajectory_Rear = curPlan(mClosestParamOnTrajectory_Rear);

		tie(mClosestSqrDistToTrajectory_Front, mClosestParamOnTrajectory_Front) = findClosestPoint(curPlan, rangeCut(dom, mClosestParamOnTrajectory - range), rangeCut(dom, mClosestParamOnTrajectory + range), mClosestParamOnTrajectory, carFrontPos);
		mClosestPosOnTrajectory_Front = curPlan(mClosestParamOnTrajectory_Front);

		tie(mClosestSqrDistToTrajectory_Back, mClosestParamOnTrajectory_Back) = findClosestPoint(curPlan, rangeCut(dom, mClosestParamOnTrajectory - range), rangeCut(dom, mClosestParamOnTrajectory + range), mClosestParamOnTrajectory, carBackPos);
		mClosestPosOnTrajectory_Back = curPlan(mClosestParamOnTrajectory_Back);

	}
	else {
		flt range = 5.0f;

		//get current forward or reverse segment as relDom
		std::pair<flt, flt> const relDom = getRelativeForwardReverseSegment(mCurPlan, mClosestParamOnTrajectory_Tip);


		flt const relPos = mClosestParamOnTrajectory + (std::isnan(mLastControllerData.curSpeed) ? 0 : (mLastControllerData.curSpeed * getPeriod()));

//		std::cout<<"closest: " << mClosestParamOnTrajectory << " relPos: " << relPos << " rangeCut: " << rangeCut(relDom, relPos - range) << " " << rangeCut(relDom, relPos + range) << " relDom: " << relDom.first << " " << relDom.second << " dom: " << dom.first << " " << dom.second;
		tie(mClosestSqrDistToTrajectory, mClosestParamOnTrajectory) = findClosestPoint(curPlan, rangeCut(relDom, relPos - range), rangeCut(relDom, relPos + range), relPos, carPos);
		mClosestPosOnTrajectory = curPlan(mClosestParamOnTrajectory);


		flt const relRearPos = mClosestParamOnTrajectory_Rear + (std::isnan(mLastControllerData.curSpeed) ? 0 : (mLastControllerData.curSpeed * getPeriod()));

		tie(mClosestSqrDistToTrajectory_Rear, mClosestParamOnTrajectory_Rear) = findClosestPoint(curPlan, rangeCut(relDom, relRearPos - range), rangeCut(relDom, relRearPos + range), relRearPos, carRearPos);
		mClosestPosOnTrajectory_Rear = curPlan(mClosestParamOnTrajectory_Rear);


//		range = 1.0f;

		flt const relFrontPos = mClosestParamOnTrajectory_Front + (std::isnan(mLastControllerData.curSpeed) ? 0 : (mLastControllerData.curSpeed * getPeriod()));

		tie(mClosestSqrDistToTrajectory_Front, mClosestParamOnTrajectory_Front) = findClosestPoint(curPlan, rangeCut(relDom, relFrontPos - range), rangeCut(relDom, relFrontPos + range), relFrontPos, carFrontPos);
		mClosestPosOnTrajectory_Front = curPlan(mClosestParamOnTrajectory_Front);

		flt const relBackPos = mClosestParamOnTrajectory_Back + (std::isnan(mLastControllerData.curSpeed) ? 0 : (mLastControllerData.curSpeed * getPeriod()));

		tie(mClosestSqrDistToTrajectory_Back, mClosestParamOnTrajectory_Back) = findClosestPoint(curPlan, rangeCut(relDom, relBackPos - range), rangeCut(relDom, relBackPos + range), relBackPos, carBackPos);
		mClosestPosOnTrajectory_Back = curPlan(mClosestParamOnTrajectory_Back);


//		std::cout<<"dom: " << dom.first << " " << dom.second;
//		std::cout<<"relDom: " << relDom.first << " " << relDom.second;
	}

	//update tip
	if (mCurCarState.gearPosition == CarState::GEAR_REVERSE) {
		mClosestParamOnTrajectory_Tip = mClosestParamOnTrajectory_Back;
		mClosestSqrDistToTrajectory_Tip = mClosestSqrDistToTrajectory_Back;
		mClosestPosOnTrajectory_Tip = mClosestPosOnTrajectory_Back;

//		std::cout<<"b: " << mClosestParamOnTrajectory << " " << mClosestParamOnTrajectory_Back << " " << mClosestParamOnTrajectory_Back-mClosestParamOnTrajectory;

	}
	else {
		mClosestParamOnTrajectory_Tip = mClosestParamOnTrajectory_Front;
		mClosestSqrDistToTrajectory_Tip = mClosestSqrDistToTrajectory_Front;
		mClosestPosOnTrajectory_Tip = mClosestPosOnTrajectory_Front;

//		std::cout<<"f: " << mClosestParamOnTrajectory << " " << mClosestParamOnTrajectory_Front << " " << mClosestParamOnTrajectory_Front-mClosestParamOnTrajectory;
	}

//	std::cout<<"p: " << mClosestParamOnTrajectory << " " << mClosestPosOnTrajectory.transpose();

//	std::cout<<"CurrentSpeed: " << curControllerData.curSpeed;

	flt lookAheadDist = min(mLookAheadMax.get(), mCurControllerData.curSpeed * mLookAheadSlope.get());
	std::string curStateName = getCurrentStateName();
	mCurControllerData = aa::modules::nav::controller::data::TimedControllerData(now, aa::modules::nav::controller::data::ControllerData(mCurEgoState, mCurCarState, mCurPlan, mClosestParamOnTrajectory, mClosestSqrDistToTrajectory, lookAheadDist, NAN, NAN, NAN, NAN, curStateName));

	mCurControllerData.projectedFrontAxlePos = mClosestPosOnTrajectory;
	mCurControllerData.projectedRearAxlePos = mClosestPosOnTrajectory_Rear;
	mCurControllerData.projectedFrontTipPos = mClosestPosOnTrajectory_Front;
	mCurControllerData.projectedBackTipPos = mClosestPosOnTrajectory_Back;

	if (mPassatCarStateIn.connected()) {
		mCurControllerData.curSpeed = mCurPassatCarState.wheelSpeeds.speedAvg();
	}

	/// Step 1: process state machine
	processStateMachine();

	/// Step 2: process obstacles
	processObstacles(mCurControllerData.curSpeed);


	/// Step 3: calculate wanted speed
	flt wantedSpeed = getWantedSpeed(mCurControllerData.curSpeed);
	mCurControllerData.wantedSpeed = wantedSpeed;
	tie(mCurControllerData.minSpeedLimit, mCurControllerData.maxSpeedLimit) = getSpeedLimitAtPlanStart(mCurPlan);


	/// Step 4: gear position
	int gearCorrection = getGearPosition();
	mCurControllerData.gearCorrection = gearCorrection;


	/// Step 5: speed correction
	speedCorrection.data = getThrottleBrakePosition(mCurControllerData.curSpeed, wantedSpeed);
	mCurControllerData.speedCorrection = speedCorrection.data;

	/// Step 6: steer correction
	steerCorrection = getSteeringPosition(mCurPlan);
	mCurControllerData.steerCorrection = steerCorrection;


	/// Step 7: aux devices data
	auxDevicesCorrection = getAuxDevicesData(mCurControllerData.curSpeed, mCurPlan);


	//fill in remaining values
	mCurControllerData.throttleWindow = getThrottleBySpeedWindow(mCurControllerData.curSpeed);
	mCurControllerData.steerWindow = getSteerBySpeedWindow(mCurControllerData.curSpeed);
	mCurControllerData.curStateName = getCurrentStateName();
	mCurControllerData.stamp();

	mRecentControllerData.push_front(mCurControllerData);


	/// write out activation
	mActivationRequestOut.write(mEngage);				// set the activation Request

	/// write out speed correction

	if (!mSpeedOut.connected()) {

	}

	if (!std::isnan(speedCorrection.data) && mSpeedOut.connected()) {
		speedCorrection.stamp();
		mSpeedOut.write(speedCorrection);
	}

	/// write out steer correction
	if (!std::isnan(steerCorrection)) {// && mSteerOut.connected()) {
		mSteerOut.write(steerCorrection);
	}

	/// write out gear correction
	mGearOut.write(gearCorrection);

	/// write out auxiliary devices data
	if (AuxDevicesData::isValidAuxDevicesData(auxDevicesCorrection)) {//&& mAuxDevicesOut.connected()) {
		mAuxDevicesOut.write(auxDevicesCorrection);
	}

	/// write out wanted speed
	mWantedSpeedOut.write(mCurControllerData.wantedSpeed);

	/// write out controller data
	mControllerDataOut.write(mCurControllerData);

	/// write out interfering obstacles
	mInterferingObstaclesOut.write(mCurObstacles);
//		mInterferingObstaclesOut.write( mCurObstaclesOnSpline );


	/// Log
	if (mLogWantedSpeed.get()) {
		now.stamp();
		std::ofstream os;

		string filename = "abstractcontroller.log";
		os.open(filename.c_str(), ios::out | ios::app);
		os << "ts/ws: " << RTT::os::TimeService::ticks2nsecs(now - TimeStamp()) << " " << mCurControllerData.wantedSpeed << std::endl;
		os.close();
	}

	mResetRequestOut.write(false);

	//HACK

	// read resumeDriveIn
	if (mResumeDriveIn.connected()) {
		bool resDrive = false;
		mResumeDriveIn.read(resDrive);

		if (resDrive) {
			resumeDrive();
		}
		else {
			pauseDrive();
		}
	}


}

void AbstractController::stopHook()
{
}

void AbstractController::errorHook()
{
}




std::pair<flt, flt> AbstractController::getRelativeForwardReverseSegment(Plan_ptr plan, flt param_tip) const
{
	Plan const & curPlan = *plan;

	std::pair<flt, flt> const dom(curPlan.domain());
	std::pair<flt, flt> relDom = dom;

	boost::optional<Plan::action_descr> lastReversePoint;
	boost::optional<Plan::action_descr> nextReversePoint = curPlan.findFirstAction(Plan::REVERSE, dom.first, dom.second);

	while (nextReversePoint && nextReversePoint.get().get<1>() <= param_tip && nextReversePoint.get().get<2>() <= param_tip) {
		lastReversePoint = nextReversePoint;
		Plan::action_descr action = nextReversePoint.get();
		nextReversePoint = curPlan.findFirstAction(Plan::REVERSE, action.get<2>(), dom.second);
	}

	float const tresh = 0.25;

	if (nextReversePoint) {
		Plan::action_descr const action = nextReversePoint.get();
		flt const nextReversePointStart = action.get<1>();
		flt const nextReversePointEnd = action.get<2>();

		if (nextReversePointStart > param_tip + tresh) {	//we are in forward segment
			if (lastReversePoint) {
				relDom.first = rangeCut(dom, lastReversePoint.get().get<2>());
				relDom.second = rangeCut(dom, nextReversePointStart);
			}
			else {
				relDom.first = dom.first;
				relDom.second = rangeCut(dom, nextReversePointStart);
			}
		}
		else {	//we are in reverse segment
			relDom.first = rangeCut(dom, nextReversePointStart);
			relDom.second = rangeCut(dom, nextReversePointEnd);
		}
	}
	else {

		relDom.first = rangeCut(dom, param_tip);
		relDom.second = dom.second;

	}

	return relDom;
}


std::pair<flt, flt> AbstractController::getSpeedLimitAtPlanStart(Plan_ptr plan) const
{
    return make_pair(NAN, NAN);
}


std::pair<flt, flt> AbstractController::getAvgSpeedLimitForConnectingEdge(edge_descr const & conEdge) const
{
    return make_pair(NAN, NAN);
}




std::pair<flt, flt> AbstractController::getThrottleBySpeedWindow(flt curSpeed) const
{
	pair<flt, flt> result = getLinearWindowValue(curSpeed, mThrottleBySpeedWindowStart.get(), mThrottleBySpeedWindowEnd.get(),
							mMaxBrakeBeforeThrottleBySpeedWindow.get(), mMaxAccelBeforeThrottleBySpeedWindow.get(),
							mMaxBrakeAfterThrottleBySpeedWindow.get(), mMaxAccelAfterThrottleBySpeedWindow.get());

	//limit at start
	if (curSpeed < mThrottleBySpeedWindowStart.get()) {
		pair<flt, flt> temp = getLinearWindowValue(curSpeed, 0.0, mThrottleBySpeedWindowStart.get(),
							  0.0, mMaxAccelStartThrottleBySpeed.get(),
							  0.0, mMaxAccelBeforeThrottleBySpeedWindow.get());

		result.second = temp.second;
	}

	return result;
}

std::pair<flt, flt> AbstractController::getSteerBySpeedWindow(flt curSpeed) const
{
	return getLinearWindowValue(curSpeed, mSteerBySpeedWindowStart.get(), mSteerBySpeedWindowEnd.get(),
								mMaxLeftSteerBeforeSteerBySpeedWindow.get(), mMaxRightSteerBeforeSteerBySpeedWindow.get(),
								mMaxLeftSteerAfterSteerBySpeedWindow.get(), mMaxRightSteerAfterSteerBySpeedWindow.get());
}

std::pair<flt, flt> AbstractController::getThrottleBySteerWindow(flt steer) const
{
	return getLinearWindowValue(abs(steer), mThrottleBySteerWindowStart.get(), mThrottleBySteerWindowEnd.get(),
								mMaxBrakeBeforeThrottleBySteerWindow.get(), mMaxAccelBeforeThrottleBySteerWindow.get(),
								mMaxBrakeAfterThrottleBySteerWindow.get(), mMaxAccelAfterThrottleBySteerWindow.get());
}

std::vector< std::pair<flt, flt> > AbstractController::slowDownLinear(flt curSpeed, flt stopSpeed, flt curParam, flt stopParam) const
{
	vector< pair<flt, flt> >  result;

	if (stopParam > curParam && curSpeed > mStopSpeed.get()) {
		assert(curSpeed >= stopSpeed);

		flt tick = getPeriod();
		flt distPerTick = curSpeed * tick;
		flt distToStop = stopParam - curParam;

		//use function f(x) = mx+b
		flt m = stopSpeed - curSpeed / distToStop;
		flt b = curSpeed;

		for (flt i = curParam, x = 0.0; i < stopParam; i += distPerTick, x += distPerTick) {
			flt speed = m * x + b;

			if (speed < mStopSpeed.get()) {
				speed = mFullStopSpeed.get();
			}

			result.push_back(pair<flt, flt>(i, speed));
		}
	}

	if (stopSpeed < mStopSpeed.get()) {
		result.push_back(pair<flt, flt>(max(stopParam, curParam), mFullStopSpeed.get()));
	}
	else {
		result.push_back(pair<flt, flt>(max(stopParam, curParam), stopSpeed));
	}

	return result;
}

std::vector< std::pair<flt, flt> > AbstractController::slowDownProgressive(flt curSpeed, flt stopSpeed, flt curParam, flt stopParam) const
{
	vector< pair<flt, flt> >  result;

	if (stopParam > curParam && curSpeed > mStopSpeed.get()) {
		assert(curSpeed >= stopSpeed);

		flt tick = getPeriod();
		flt distPerTick = curSpeed * tick;
		flt distToStop = stopParam - curParam;

		//use function f(x) = c/(x-a)+b with a farther away from 0
		flt c = 500;
		flt a1 = distToStop / 2 + sqrt((distToStop / 2) * (distToStop / 2) + c * distToStop / (curSpeed - stopSpeed));
		flt a2 = distToStop / 2 - sqrt((distToStop / 2) * (distToStop / 2) + c * distToStop / (curSpeed - stopSpeed));
		flt a = abs(a1) > abs(a2) ? a1 : a2;
		flt b = curSpeed + c / a;

		for (flt i = curParam, x = 0.0; i < stopParam; i += distPerTick, x += distPerTick) {
			flt speed = c / (x - a) + b;

			if (speed < mStopSpeed.get()) {
				speed = mFullStopSpeed.get();
			}

			result.push_back(pair<flt, flt>(i, speed));
		}
	}

	if (stopSpeed < mStopSpeed.get()) {
		result.push_back(pair<flt, flt>(max(stopParam, curParam), mFullStopSpeed.get()));
	}
	else {
		result.push_back(pair<flt, flt>(max(stopParam, curParam), stopSpeed));
	}

	return result;
}

std::vector< std::pair<flt, flt> > AbstractController::slowDownDegressive(flt curSpeed, flt stopSpeed, flt curParam, flt stopParam) const
{
	vector< pair<flt, flt> >  result;

	if (stopParam > curParam && curSpeed > mStopSpeed.get()) {
		assert(curSpeed >= stopSpeed);

		flt tick = getPeriod();
		flt distPerTick = curSpeed * tick;
		flt distToStop = stopParam - curParam;

		//use function f(x) = c/(x-a)+b with a closer to 0
		flt c = 500;
		flt a1 = distToStop / 2 + sqrt((distToStop / 2) * (distToStop / 2) + c * distToStop / (curSpeed - stopSpeed));
		flt a2 = distToStop / 2 - sqrt((distToStop / 2) * (distToStop / 2) + c * distToStop / (curSpeed - stopSpeed));
		flt a = abs(a1) < abs(a2) ? a1 : a2;
		flt b = curSpeed + c / a;

		for (flt i = curParam, x = 0.0; i < stopParam; i += distPerTick, x += distPerTick) {
			flt speed = c / (x - a) + b;

			if (speed < mStopSpeed.get()) {
				speed = mFullStopSpeed.get();
			}

			result.push_back(pair<flt, flt>(i, speed));
		}
	}

	if (stopSpeed < mStopSpeed.get()) {
		result.push_back(pair<flt, flt>(max(stopParam, curParam), mFullStopSpeed.get()));
	}
	else {
		result.push_back(pair<flt, flt>(max(stopParam, curParam), stopSpeed));
	}

	return result;
}


flt AbstractController::getBrakingDistanceForConstantBrakeAcceleration(flt acceleration, flt startVelocity) const
{
	if (mSoftBrakeTrajectory) {
		return -0.5 * pow(fabs(startVelocity), 1 / mSoftBrakeTrajectoryExp) / mSoftBrakeTrajectoryLin;
	}
	else {
		return -0.5 * startVelocity * startVelocity / acceleration;
	}


}


flt AbstractController::getStartVelocityForConstantBrakeAcceleration(flt acceleration, flt stopSpeed, flt curParam, flt stopParam, bool includeSafety) const
{
	flt returnSquared = 0.0;

	//std::cout << " accel " << acceleration << " stopSpeed " << stopSpeed << " curParam " << curParam << " stopP " << stopParam << std::endl;

	if (mSoftBrakeTrajectory) {
		if (includeSafety) {
			returnSquared = -2 * mSoftBrakeTrajectoryLin * (stopParam - curParam - stopSpeed) + pow(stopSpeed, 1.0 / mSoftBrakeTrajectoryExp);
		}
		else {
			returnSquared = -2 * mSoftBrakeTrajectoryLin * (stopParam - curParam) + pow(stopSpeed, 1.0 / mSoftBrakeTrajectoryExp);
		}

		if (returnSquared > 0.0) {
			//    std::cout << " soft " << pow(returnSquared, mExponent) << std::endl;
			return pow(returnSquared, mSoftBrakeTrajectoryExp);
		}
		else {
			//          std::cout << " soft nil " << std::endl;
			//std::cout<<"Error in getStartVelocityForConstantBrakeAcc., acc.:" << acceleration << ", stopSpeed: " << stopSpeed << " curP: " << curParam << ", stopParam: " << stopParam ;
			return 0.0;
		}
	}
	else {
		if (includeSafety) {
			returnSquared = -2 * acceleration * (stopParam - curParam - /*2 * */ stopSpeed) + stopSpeed * stopSpeed;
		}
		else {
			returnSquared = -2 * acceleration * (stopParam - curParam) + stopSpeed * stopSpeed;
		}

		if (returnSquared > 0.0) {
			std::cout << " not soft " << sqrt(returnSquared) << std::endl;
			return sqrt(returnSquared);
		}
		else {
			//std::cout<<"Error in getStartVelocityForConstantBrakeAcc., acc.:" << acceleration << ", stopSpeed: " << stopSpeed << " curP: " << curParam << ", stopParam: " << stopParam ;
			return 0.0;
		}
	}
}

bool AbstractController::isActionProcessed(Plan::action_descr action) const
{
	flt tolerance = 7.5f;

	for (vector<Plan::action_descr>::const_iterator it = processedActions.begin(); it != processedActions.end(); it++) {
		Plan::action_descr a = *it;

		if (action.get<0>() == Plan::STOP_SIGN || action.get<0>() == Plan::DECISION_POINT) {
			try {
				VertexData aV = any_cast<VertexData>(a.get<3>());
				VertexData actionV = any_cast<VertexData>(action.get<3>());

				if (a.get<0>() == action.get<0>() && aV.id == actionV.id && aV.name == actionV.name) {		//equal action
					return abs(action.get<1>() - mClosestParamOnTrajectory) <= tolerance;					//at equal position
				}
			}
			catch (const boost::bad_any_cast &) {
				return false;
			}
		}
	}

	return false;
}

#if 0
bool AbstractController::setRamp(std::vector< std::pair<flt, flt> > ramp, int successingState, bool force)
{
	if (!force && !speedRamp.ramp.empty()) {
		//check if we can overwrite
		if (ramp.front().second > speedRamp.ramp.front().second) {		//faster than before
			return false;
		}

		flt rampSlope = (ramp.back().second - ramp.front().second) / (ramp.back().first - ramp.front().first);
		flt speedRampSlope = (speedRamp.ramp.back().second - speedRamp.ramp.front().second) / (speedRamp.ramp.back().first - speedRamp.ramp.front().first);

		if (rampSlope > speedRampSlope) {								//less steep than before
			return false;
		}
	}

	speedRamp.ramp = ramp;
	speedRamp.successingState = successingState;

	return true;
}
#endif




aa::modules::nav::controller::data::ControllerData AbstractController::simulateStep(Plan_ptr plan, TimedEgoState egoState, TimedCarState carState, TimedBaseObstacleBundle_ptr obstacles)
{
	std::cout << "not implemented yet";
	assert(false && "not implemented yet");
	return aa::modules::nav::controller::data::ControllerData();
}



void AbstractController::processStateMachine()
{
	Logger::In in("Controller");

	Plan const & curPlan = *mCurPlan;
	std::pair<flt, flt> const dom(curPlan.domain());

	//get statemachine functions
	OperationCaller<void(void)> updateStates(mStateMachine->getOperation("updateStates"));
	OperationCaller<bool(int)> isInState(mStateMachine->getOperation("isInState"));
	OperationCaller<void(int, std::string, std::string)> enterState(mStateMachine->getOperation("enterState"));



	//generic update
	updateStates();

	//specific updates
	if (isInState(CONTEXT_CONTROLLEDBRAKE)) {

		if (mCurControllerData.curSpeed < 0.8 * KMH_2_MS) {

			if (isInState(STATE_CONTROLLEDBRAKE_MALFUNCTION)) {
				enterState(STATE_PERMANENTSTOP, rtti::typeName(typeid(*this)), "goto permanent stop after malfunction has been encountered");

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_PAUSE)) {
				enterState(STATE_PERMANENTSTOP, rtti::typeName(typeid(*this)), "goto permanent stop after pause");

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_END)) {
				enterState(STATE_PERMANENTSTOP, rtti::typeName(typeid(*this)), "goto permanent stop after controlled brake");

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_TRAFFICLIGHT)) {
				//NOP

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_STOPSIGN)) {
				enterState(STATE_TIMEDSTOP, rtti::typeName(typeid(*this)), "goto temporary stop after controlled brake");

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_GIVEWAY)) {
				//NOP

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_REVERSE)) {
				//NOP

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_DECISIONPOINT)) {
				enterState(STATE_WAITFOROPERATOR, rtti::typeName(typeid(*this)), "wait for decision by operator");

			}
			else if (isInState(STATE_CONTROLLEDBRAKE_OBSTACLE)) {
				//NOP
			}

			//process all stop signs and decision points in vicinity
			boost::optional<Plan::action_descr> nextStopOrDecision = curPlan.findFirstAction(Plan::STOP_SIGN, Plan::DECISION_POINT, dom.first, dom.second);

			while (nextStopOrDecision && isActionProcessed(nextStopOrDecision.get())) {
				nextStopOrDecision = curPlan.findFirstAction(Plan::STOP_SIGN, Plan::DECISION_POINT, nextStopOrDecision.get().get<2>(), dom.second);
			}

			flt distToStopOrDecision = numeric_limits<flt>::infinity();

			if (nextStopOrDecision) {
				flt const stopStart = nextStopOrDecision.get().get<1>();
				distToStopOrDecision = stopStart - mClosestParamOnTrajectory;
			}

			if (distToStopOrDecision < 5) {
				processedActions.push_back(nextStopOrDecision.get());
			}

		}
		else {

		}

	}
	else if (isInState(STATE_PERMANENTSTOP)) {

	}
	else if (isInState(STATE_WAITFORROADTOCLEAR)) {	//check for clear road
		if (!mCurObstacles) {	//no obstacles -> go ahead
			enterState(STATE_DRIVE, rtti::typeName(typeid(*this)), "resume drive, road ahead is clear of obstacles");

		}
		else {

// 			ObstacleData const & curObstacles = *mCurObstacles;
			BaseObstacleBundle const & curObstacles = *mCurObstacles;
			std::vector< int > mOnOrNearLaneObstacles;

			for (uint o = 0; o < curObstacles.size(); o++) {
				if (curObstacles[o].onLane() || curObstacles[o].location() == NEAR_LANE) {
					mOnOrNearLaneObstacles.push_back(o);
				}
			}

			bool clearRoad = true;

			flt const frontDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontDistance")->value();
            Vec3 const carFrontPos = mCurControllerData.carPos + frontDistance * mCurControllerData.carDir;

			flt const realCarWidth = ::data::theVehicleData::instance().getPropertyType<flt>("realCarWidth")->value();
			flt mFreeTrackWidth = max(mMinFreeTrackWidth.get(), realCarWidth + 2 * mFreeMargin.get());

			//filter nearby obstacles
			for (uint i = 0; i < mOnOrNearLaneObstacles.size(); i++) {
				int o = mOnOrNearLaneObstacles[i];
				::aa::data::obstacle::util::Contour const & cPoints = curObstacles[o].contour();

				for (uint p = 0; p < cPoints.size(); p++) {
                    Vec3 point = zeroExtend( cPoints[p] );
					flt dist = (point - carFrontPos).norm();
					flt angle = ::math::angle(mCurControllerData.carDir, point - carFrontPos);
					flt latDist = abs(sin(angle)) * dist;

					//is in front of car, in free track width and closer than 2 meters?
					if (abs(angle) < PI / 2.0 && latDist < mFreeTrackWidth && dist < 2.0) {
						clearRoad = false;
					}
				}
			}

			//road is clear
			if (clearRoad) {
				enterState(STATE_DRIVE, rtti::typeName(typeid(*this)), "resume drive, road ahead is clear of obstacles");
			}
		}
	}



//	if (mCurMission.mRemainingCheckpoints.empty() && mCurControllerData.curSpeed < 0.8 * KMH_2_MS) {
//		enterState(STATE_WAITFORMISSION, rtti::typeName(typeid(*this)), "no mission");
//	}

}




void AbstractController::processObstacles(flt curSpeed)
{
	Logger::In in("Controller");

	if (!mUseObstacles.get()) {
		return;
	}

	if (!mCurObstacles) {
		return;
	}

	if (!mCurPlan) {
		return;
	}

	Plan const & curPlan = *mCurPlan;
	std::pair<flt, flt> const dom(curPlan.domain());

	BaseObstacleBundle & curObstacles = *mCurObstacles;

	if (mPredictUnknownObstacles && RTT::os::TimeService::ticks2nsecs(static_cast<TimeStamp>(mCurEgoState) - *mCurObstacles) > mPredictUnknownObstaclesThreshold) {
		aa::modules::nav::obstacles::predictMovement(curObstacles, mCurEgoState);
		mCurObstacles->adoptTimeStamp(mCurEgoState);
	}

	flt const realCarWidth = ::data::theVehicleData::instance().getPropertyType<flt>("realCarWidth")->value();
	flt const mFreeTrackWidth = max(mMinFreeTrackWidth.get(), realCarWidth + 2 * mFreeMargin.get());

	mCurObstaclesOnSpline.clear();
	//mCurObstaclesOnSpline = ObstacleMath::findAllObstaclesOnSpline(curPlan, curObstacles, mFreeTrackWidth, dom.first, dom.second);
	mCurObstaclesOnSpline = ObstacleMath::findAllObstaclesOnSpline(curPlan, curObstacles, mFreeTrackWidth * 0.5, mFreeTrackWidth * 0.5, max(dom.first, mClosestParamOnTrajectory), dom.second);

	unsigned int firstId = 0;
	unsigned int i = 0;

	for (ObstacleMath::InterferingObstacleList::iterator it = mCurObstaclesOnSpline.begin(); it != mCurObstaclesOnSpline.end(); it++, i++) {
//		ObstacleMath::ObstTriple obstacleTriple = *it;
		InterferingObstacle const iobst = *it;

		BaseObstacle * const obstacle = iobst.baseObstacle;

		if (i == 0) {
			obstacle->setLocation(FIRST_ON_PLAN);
// 			std::cout<<"first: " << abs(obstacleTriple.get<1>() - mClosestParamOnTrajectory_Tip) << "  " << obstacle.location() << "  " << obstacle.id();
			firstId = obstacle->id();
		}
		else {
			obstacle->setLocation(ON_PLAN);
// 			std::cout<<"other: " << abs(obstacleTriple.get<1>() - mClosestParamOnTrajectory_Tip) << "  " << obstacle.location() << "  " << obstacle.id();
		}
	}

// 	std::cout<<"first id: " << id;


	mBrakingObstacleIdOut.write(firstId);


	/// Log
	if (mLogWantedSpeed.get()) {
		now.stamp();
		std::ofstream os;

		string filename = "abstractcontroller.log";
		os.open(filename.c_str(), ios::out | ios::app);
		os << "ts/id: " << RTT::os::TimeService::ticks2nsecs(now - TimeStamp()) << " " << firstId << std::endl;
		os.close();
	}
}




flt AbstractController::getWantedSpeed(flt curSpeed)
{
	Logger::In in("Controller");

// 	cout << "processed:\t";
// 	for(vector<Plan::action_descr>::iterator it=processedActions.begin(); it != processedActions.end(); it++) {
// 		cout << " " << (*it).get<0>() << " (" << (*it).get<1>() << ")";
// 	}
// 	cout << endl;


	//get statemachine functions
	OperationCaller<bool(int)> isInState(mStateMachine->getOperation("isInState"));


	if (isInState(CONTEXT_STOP)) {
		return mFullStopSpeed.get();
	}

	if (isInState(STATE_CONTROLLEDBRAKE_PAUSE) || isInState(STATE_CONTROLLEDBRAKE_MALFUNCTION)) {
		if (curSpeed < 1 * KMH_2_MS) {
			return mFullStopSpeed;
		}

        //std::cout << "m: " << min(curSpeed, mLastControllerData.wantedSpeed) * 0.99;
		return min(curSpeed, mLastControllerData.wantedSpeed) * 0.99;
	}

	flt wantedSpeed = mMaxSpeed.get();
	//TODO calculate wantedSpeed based on surface quality
	//TODO calculate wantedSpeed based on other factors (health of car, visibility range, traffic density)

	//calculate speed proposal by trajectory: mission, length, actions
	wantedSpeed = min(wantedSpeed, getSpeedProposalByTrajectory(curSpeed, wantedSpeed));


	if (mDebugPrintWantedSpeedIntermediateSteps.get()) {
		std::cout << "wantedSpeed after trajectory: " << wantedSpeed * MS_2_KMH;
	}

	if (wantedSpeed == mFullStopSpeed.get()) {
		return wantedSpeed;
	}

	//calculate speed proposal by centrifugal force
	wantedSpeed = min(wantedSpeed, getSpeedProposalByCentrifugalForce(curSpeed, wantedSpeed));

	if (mDebugPrintWantedSpeedIntermediateSteps.get()) {
		std::cout << "wantedSpeed after centrifugal force: " << wantedSpeed * MS_2_KMH;
	}

	if (wantedSpeed == mFullStopSpeed.get()) {
		return wantedSpeed;
	}

	//calculate speed proposal by curvature
// 	wantedSpeed = min(wantedSpeed, getSpeedProposalByCurvature(curSpeed, wantedSpeed));
// 	if (wantedSpeed == mFullStopSpeed.get()) {
//		return wantedSpeed;
//	}

	flt wantedSpeedByObstacles = getSpeedProposalByObstacles(curSpeed, wantedSpeed);

	mWantedSpeedByObstaclesOut.write(wantedSpeedByObstacles);

	//calculate speed proposal by obstacles
	wantedSpeed = min(wantedSpeed, wantedSpeedByObstacles);

	if (mDebugPrintWantedSpeedIntermediateSteps.get()) {
		std::cout << "wantedSpeed after obstacles: " << wantedSpeed * MS_2_KMH;
	}

	if (wantedSpeed == mFullStopSpeed.get()) {
		return wantedSpeed;
	}

	//calculate speed proposal by surface quality
// 	wantedSpeed = min(wantedSpeed, getSpeedProposalBySurface(curSpeed, wantedSpeed));
// 	if (wantedSpeed == mFullStopSpeed.get()) {
//		return wantedSpeed;
//	}

	wantedSpeed = rangeCut(mMinSpeed.get(), wantedSpeed, mMaxSpeed.get());

	if (mDebugPrintWantedSpeedIntermediateSteps.get()) {
		std::cout << "wantedSpeed after rangecut: " << wantedSpeed * MS_2_KMH;
	}

// 	if(mUseActiveSmoothing.get() && !theStateMachine::instance().isInState(CONTEXT_CONTROLLEDBRAKE)) {
// 		flt accWantedSpeed = wantedSpeed;
// 		int numSamples = min(mASWantedSpeedWindowSize.get(), (int)recentControllerData.size());
// 		for(int i=0; i < numSamples; i++) {
// 			if(recentControllerData[i].wantedSpeed > 0) {
// 				accWantedSpeed = min(accWantedSpeed, recentControllerData[i].wantedSpeed);
// 			}
// 		}
// 		wantedSpeed = (wantedSpeed + accWantedSpeed)/2.0;
// 	}

	if (mDebugPrintWantedSpeedIntermediateSteps.get()) {
		std::cout << "";
	}

	return wantedSpeed;
}


flt AbstractController::getSpeedProposalByTrajectory(flt curSpeed, flt wantedSpeedSoFar)
{
	if (wantedSpeedSoFar == mFullStopSpeed.get()) {
		return mFullStopSpeed.get();
	}

	flt wantedSpeed = wantedSpeedSoFar;

	Plan const & curPlan = *mCurPlan;
	std::pair<flt, flt> const dom(curPlan.domain());

	// access graph
    aGraph const & rGraph = mARNDGraph.getBoostGraph();
	property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rGraph);

	pair<flt, flt> moderateSpeedLimit(mMinSpeed.get(), min(mMaxSpeed.get(), mConstantSpeed.get()));

	pair<flt, flt> speedLimit(mMinSpeed.get(), mMaxSpeed.get());

    speedLimit.second = min(speedLimit.second, moderateSpeedLimit.second);
    speedLimit.first = min(min(speedLimit.first, moderateSpeedLimit.first), speedLimit.second);

	//std::cout<<"speed limit: " << speedLimit.first << " / " << speedLimit.second;


	wantedSpeed = min(wantedSpeed, speedLimit.second);

	//rule of thumb: braking distance = v/10 * v/10
	//rule of thumb: reaction distance = v/10 * 3
	//rule of thumb: overall stopping distance = reaction distance + braking distance

	//Commented out by Daniel
	//flt brake_dist = (curSpeed * MS_2_KMH / 10.0) * (curSpeed * MS_2_KMH / 10.0);
	//flt reaction_dist = (curSpeed * MS_2_KMH / 10.0) * 3.0;
	//flt stopping_dist = /*2.0*/ 1.5 * (reaction_dist + brake_dist);

//	flt testa = getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, 0.0, 0.0, 10.31415, false);
//	std::cout<<"Test A ... stopping dist to velocity: " << testa << "  km/h ";

//	flt testb = getBrakingDistanceForConstantBrakeAcceleration(mBrakingAccelerationComfort, testa);
//	std::cout<<"Test ... velocity to stopping dist: " << testb << " meters ";
//	std::cout<<"*************";

	//1.2 is for 20% safety zone, 5meter constant reserve
	flt stopping_dist = 1.2 * getBrakingDistanceForConstantBrakeAcceleration(mBrakingAccelerationComfort, curSpeed) + 5;
//	if(mCounter.get() % 100 == 0) std::cout<<"stopping_dist: " << stopping_dist;

	// stop at goal
	flt distToGoal = numeric_limits<flt>::infinity();
	bool stopAtGoal = false;

	boost::optional<Plan::action_descr> nextGoal = curPlan.findFirstAction(Plan::GOAL, dom.first, dom.second);

	if (nextGoal) {
		Plan::action_descr action = nextGoal.get();
		flt const goalStart = action.get<1>();
		flt const goalEnd = action.get<2>();
		distToGoal = max(flt(0), goalStart - mClosestParamOnTrajectory_Tip);
		stopAtGoal = mStopAtGoal.get() && distToGoal <= max((flt)stopping_dist + mGoalSafetyDist.get(), mGoalMinLookaheadDist.get());
	}



	// stop at end of spline?
	flt distToEndSpline = max(flt(0), dom.second - mClosestParamOnTrajectory_Tip);
	bool stopAtEndSpline = distToEndSpline <= max((flt)stopping_dist + mEndSplineSafetyDist.get(), mEndSplineMinLookaheadDist.get());



	//stop at traffic light?
	flt distToTrafficLight = numeric_limits<flt>::infinity();
	bool stopAtTrafficLight = false;

	boost::optional<Plan::action_descr> nextTrafficLight = curPlan.findFirstAction(Plan::TRAFFIC_LIGHT, dom.first, dom.second);

	if (nextTrafficLight) {
		distToTrafficLight = nextTrafficLight.get().get<1>() - mClosestParamOnTrajectory_Tip;
		//std::cout<<"Acceration " << 0.5 * curSpeed * curSpeed / distToTrafficLight << " Dist " << distToTrafficLight << " Speed " << curSpeed;
		//Uses Point of no return
		stopAtTrafficLight = mStopAtTrafficLight.get() && distToTrafficLight <= max((flt)stopping_dist + mTrafficLightSafetyDist.get() , mTrafficLightMinLookaheadDist.get())
							 && ((fabs(distToTrafficLight) > mTrafficLightPointOfNoReturnRatio * getBrakingDistanceForConstantBrakeAcceleration(mBrakingAccelerationComfort, curSpeed)) || mTrafficLightBrakeInFrontPointOfNoReturn);
		mTrafficLightBrakeInFrontPointOfNoReturn = stopAtTrafficLight;

		Plan::action_descr const action = nextTrafficLight.get();
		boost::any const trafficLightMeta = action.get<3>();

		try {
			VertexData const trafficLightVertex = any_cast<VertexData>(trafficLightMeta);
			mTrafficLightVertexName = trafficLightVertex.name;
			boost::any const trafficLightStatusMeta = trafficLightVertex.metaData;

			if (!trafficLightStatusMeta.empty()) {
				VertexData::TrafficSignalState const trafficLightStatus = any_cast<VertexData::TrafficSignalState>(trafficLightStatusMeta);

				if ((trafficLightStatus == VertexData::RED || trafficLightStatus == VertexData::RED_YELLOW || trafficLightStatus == VertexData::YELLOW || trafficLightStatus == VertexData::UNKNOWN
						|| trafficLightStatus == VertexData::FORCE_RED               || trafficLightStatus == VertexData::FORCE_RED_YELLOW               || trafficLightStatus == VertexData::FORCE_YELLOW
						|| trafficLightStatus == VertexData::FORCE_RED_CV_RED        || trafficLightStatus == VertexData::FORCE_RED_YELLOW_CV_RED        || trafficLightStatus == VertexData::FORCE_YELLOW_CV_RED
						|| trafficLightStatus == VertexData::FORCE_RED_CV_RED_YELLOW || trafficLightStatus == VertexData::FORCE_RED_YELLOW_CV_RED_YELLOW || trafficLightStatus == VertexData::FORCE_YELLOW_CV_RED_YELLOW
						|| trafficLightStatus == VertexData::FORCE_RED_CV_YELLOW     || trafficLightStatus == VertexData::FORCE_RED_YELLOW_CV_YELLOW     || trafficLightStatus == VertexData::FORCE_YELLOW_CV_YELLOW
						|| trafficLightStatus == VertexData::FORCE_RED_CV_GREEN      || trafficLightStatus == VertexData::FORCE_RED_YELLOW_CV_GREEN      || trafficLightStatus == VertexData::FORCE_YELLOW_CV_GREEN)
				   ) {

					// std::cout << " drin " << std::endl;

					Plan::action_descr action = nextTrafficLight.get();
					flt const trafficLightStart = action.get<1>();
					//flt const trafficLightEnd = action.get<2>();
					distToTrafficLight = max(flt(0), trafficLightStart - mClosestParamOnTrajectory_Tip);
					//Uses Point of no return
					stopAtTrafficLight = mStopAtTrafficLight.get() && distToTrafficLight <= max((flt)stopping_dist + mTrafficLightSafetyDist.get() , mTrafficLightMinLookaheadDist.get())
										 && ((fabs(distToTrafficLight) > mTrafficLightPointOfNoReturnRatio * getBrakingDistanceForConstantBrakeAcceleration(mBrakingAccelerationComfort, curSpeed)) || mTrafficLightBrakeInFrontPointOfNoReturn);
					mTrafficLightBrakeInFrontPointOfNoReturn = stopAtTrafficLight;
				}
				else if (trafficLightStatus != VertexData::UNKNOWN) {
					stopAtTrafficLight = false;
					mTrafficLightBrakeInFrontPointOfNoReturn = false;
				}

// 				std::cout<<"got traffic light in " << distToTrafficLight <<  "  "  << trafficLightStatus;
			}
			else {
				//   std::cout << "empty" << std::endl;
// 			 // no camera traffic light detected

				Plan::action_descr action = nextTrafficLight.get();
				flt const trafficLightStart = action.get<1>();

				//flt const trafficLightEnd = action.get<2>();

				distToTrafficLight = max(flt(0), trafficLightStart - mClosestParamOnTrajectory_Tip);
				//Uses Point of no return
				stopAtTrafficLight = mStopAtTrafficLight.get() && distToTrafficLight <= max((flt)stopping_dist + mTrafficLightSafetyDist.get() , mTrafficLightMinLookaheadDist.get())
									 && ((fabs(distToTrafficLight) > mTrafficLightPointOfNoReturnRatio * getBrakingDistanceForConstantBrakeAcceleration(mBrakingAccelerationComfort, curSpeed)) || mTrafficLightBrakeInFrontPointOfNoReturn);

				//std::cout << "foo " << stopAtTrafficLight << std:: endl;

				mTrafficLightBrakeInFrontPointOfNoReturn = stopAtTrafficLight;

//                  logWarning()<<"found traffic light at " << trafficLightVertex.name << " with no signal state";
			}

		}
		catch (const boost::bad_any_cast &) {
// 			std::cout<<"got exception";
		}
	}
	else {
		mTrafficLightVertexName = std::string("");
	}



	// stop at stop sign?
	flt distToStopSign = numeric_limits<flt>::infinity();
	bool stopAtStopSign = false;

	boost::optional<Plan::action_descr> nextStopSign = curPlan.findFirstAction(Plan::STOP_SIGN, dom.first, dom.second);

	while (nextStopSign && isActionProcessed(nextStopSign.get())) {
		Plan::action_descr action = nextStopSign.get();
		nextStopSign = curPlan.findFirstAction(Plan::STOP_SIGN, action.get<2>(), dom.second);
	}

	if (nextStopSign) {
		Plan::action_descr const action = nextStopSign.get();
		flt const stopStart = action.get<1>();
		flt const stopEnd = action.get<2>();
		distToStopSign = max(flt(0), stopStart - mClosestParamOnTrajectory_Tip);
		stopAtStopSign = mStopAtStopSign.get() && distToStopSign <= max((flt)stopping_dist + mStopSignSafetyDist.get(), mStopSignMinLookaheadDist.get());
	}


	//stop because of give way?
	flt distToGiveWay = numeric_limits<flt>::infinity();
	bool stopAtGiveWay = false;

	boost::optional<Plan::action_descr> nextGiveWay = curPlan.findFirstAction(Plan::GIVE_WAY, dom.first, dom.second);

	if (nextGiveWay) {
		Plan::action_descr const action = nextGiveWay.get();
		flt const giveWayStart = action.get<1>();
		flt const giveWayEnd = action.get<2>();
		distToGiveWay = max(flt(0), giveWayStart - mClosestParamOnTrajectory_Tip);
		stopAtGiveWay = mStopAtGiveWay.get() && distToGiveWay <= max((flt)stopping_dist + mGiveWaySafetyDist.get() , mGiveWayMinLookaheadDist.get());
	}

	// stop at reverse point?
	flt distToReversePoint = numeric_limits<flt>::infinity();
	bool stopAtReversePoint = false;

	boost::optional<Plan::action_descr> nextReversePoint = curPlan.findFirstAction(Plan::REVERSE, dom.first, dom.second);

	while (nextReversePoint && nextReversePoint.get().get<1>() <= mClosestParamOnTrajectory_Tip && nextReversePoint.get().get<2>() <= mClosestParamOnTrajectory_Tip) {
		Plan::action_descr action = nextReversePoint.get();
		nextReversePoint = curPlan.findFirstAction(Plan::REVERSE, action.get<2>(), dom.second);
	}

	if (nextReversePoint) {
		Plan::action_descr const action = nextReversePoint.get();
		flt const reversePointStart = action.get<1>();
		flt const reversePointEnd = action.get<2>();

		bool needSwitch = false;

		if (reversePointStart > mClosestParamOnTrajectory_Tip) {	//we are in forward segment
			distToReversePoint = max(flt(0), reversePointStart - mClosestParamOnTrajectory_Tip);

			if (mCurCarState.gearPosition != CarState::GEAR_REVERSE) {
				needSwitch = true;
			}

//			std::cout<<"forward: " << distToReversePoint << "   " << mClosestParamOnTrajectory_Tip << "\t" << reversePointStart << "\t" << reversePointEnd;
		}
		else {	//we are in reverse segment
			distToReversePoint = max(flt(0), reversePointEnd - mClosestParamOnTrajectory_Tip);

			if (mCurCarState.gearPosition == CarState::GEAR_REVERSE) {
				needSwitch = true;
			}

//			std::cout<<"reverse: " << distToReversePoint << "   " << mClosestParamOnTrajectory_Tip << "\t" << reversePointStart << "\t" << reversePointEnd;
		}

		stopAtReversePoint = needSwitch && mStopAtReversePoint.get() && distToReversePoint <= max((flt)stopping_dist + mReversePointSafetyDist.get() , mReversePointMinLookaheadDist.get());

//		mCurControllerData.projectedRearAxlePos = curPlan(reversePointStart);
//		mCurControllerData.projectedBackTipPos = curPlan(reversePointEnd);

	}
	else {
//		std::cout<<"no reverse";

	}



	// stop at decision point?
	flt distToDecisionPoint = numeric_limits<flt>::infinity();
	bool stopAtDecisionPoint = false;

	boost::optional<Plan::action_descr> nextDecisionPoint = curPlan.findFirstAction(Plan::DECISION_POINT, dom.first, dom.second);

	while (nextDecisionPoint && isActionProcessed(nextDecisionPoint.get())) {
		Plan::action_descr action = nextDecisionPoint.get();
		nextDecisionPoint = curPlan.findFirstAction(Plan::DECISION_POINT, action.get<2>(), dom.second);
	}

	if (nextDecisionPoint) {
		Plan::action_descr const action = nextDecisionPoint.get();
		flt const decisionStart = action.get<1>();
		flt const decisionEnd = action.get<2>();
		distToDecisionPoint = max(flt(0), decisionStart - mClosestParamOnTrajectory_Tip);
		stopAtDecisionPoint = mStopAtDecisionPoint.get() && distToDecisionPoint <= max((flt)stopping_dist + mDecisionPointSafetyDist.get(), mDecisionPointMinLookaheadDist.get());
	}


	//get statemachine functions
	OperationCaller<bool(int)> isInState(mStateMachine->getOperation("isInState"));
	OperationCaller<void(int, std::string, std::string)> enterState(mStateMachine->getOperation("enterState"));


//        if (stopAtTrafficLight) {
//        std::cout << " stopTrafficLight true " << std::endl;
//        } else
//        { std::cout << " stopTrafficLight false " << std::endl;}
	if (stopAtGoal || stopAtEndSpline || stopAtTrafficLight || stopAtStopSign || stopAtGiveWay || stopAtReversePoint || stopAtDecisionPoint) {
		flt minStopDist = min(distToGoal, distToEndSpline);
		minStopDist = min(minStopDist, distToTrafficLight);
		minStopDist = min(minStopDist, distToStopSign);
		minStopDist = min(minStopDist, distToGiveWay);
		minStopDist = min(minStopDist, distToReversePoint);
		minStopDist = min(minStopDist, distToDecisionPoint);


		if (stopAtGoal && minStopDist == distToGoal) {
			wantedSpeed = min(wantedSpeed, getSpeedProposalByGoal(distToGoal));
			enterState(STATE_CONTROLLEDBRAKE_GOAL, rtti::typeName(typeid(*this)), (format("goto brake because goal is in %1% m") % distToGoal).str());

		}
		else if (stopAtEndSpline && minStopDist == distToEndSpline) {
			wantedSpeed = min(wantedSpeed, getSpeedProposalByEndSpline(distToEndSpline));
			enterState(STATE_CONTROLLEDBRAKE_END, rtti::typeName(typeid(*this)), (format("goto brake because spline is ending in %1% m") % distToEndSpline).str());

		}
		else if (stopAtTrafficLight && minStopDist == distToTrafficLight) {
			wantedSpeed = min(wantedSpeed, getSpeedProposalByTrafficLight(distToTrafficLight));
			enterState(STATE_CONTROLLEDBRAKE_TRAFFICLIGHT, rtti::typeName(typeid(*this)), (format("goto brake because traffic light encountered in %1% m") % distToTrafficLight).str());
		}
		else if (stopAtStopSign && minStopDist == distToStopSign) {
			wantedSpeed = min(wantedSpeed, getSpeedProposalByStopSign(distToStopSign));
			enterState(STATE_CONTROLLEDBRAKE_STOPSIGN, rtti::typeName(typeid(*this)), (format("goto brake because stop sign encountered in %1% m") % distToStopSign).str());

		}
		else if (stopAtGiveWay && minStopDist == distToGiveWay) {
			wantedSpeed = min(wantedSpeed, getSpeedProposalByGiveWay(distToGiveWay));
			enterState(STATE_CONTROLLEDBRAKE_GIVEWAY, rtti::typeName(typeid(*this)), (format("goto brake because we need to give way in %1% m") % distToGiveWay).str());

		}
		else if (stopAtReversePoint && minStopDist == distToReversePoint) {
			wantedSpeed = min(wantedSpeed, getSpeedProposalByReversePoint(distToReversePoint));
			enterState(STATE_CONTROLLEDBRAKE_REVERSE, rtti::typeName(typeid(*this)), (format("goto brake because we need to switch gear in %1% m") % distToReversePoint).str());

		}
		else if (stopAtDecisionPoint && minStopDist == distToDecisionPoint) {
			wantedSpeed = min(wantedSpeed, getSpeedProposalByDecisionPoint(distToDecisionPoint));
			enterState(STATE_CONTROLLEDBRAKE_DECISIONPOINT, rtti::typeName(typeid(*this)), (format("goto brake because decision point encountered in %1% m") % distToDecisionPoint).str());
		}

	}
	else {
		int followState = STATE_DRIVE;

		if (mCurControllerData.curSpeed < mWalkingSpeed.get()) {
			followState = STATE_WAITFORROADTOCLEAR;
		}

		if (isInState(STATE_CONTROLLEDBRAKE_GOAL)) {
            enterState(followState, rtti::typeName(typeid(*this)), "go back to drive, goal has vanished");
		}
		else if (isInState(STATE_CONTROLLEDBRAKE_END)) {
			enterState(followState, rtti::typeName(typeid(*this)), "go back to drive, spline is not ending anymore");
		}
		else if (isInState(STATE_CONTROLLEDBRAKE_TRAFFICLIGHT)) {
			enterState(followState, rtti::typeName(typeid(*this)), "go back to drive after traffic light");
		}
		else if (isInState(STATE_CONTROLLEDBRAKE_STOPSIGN)) {
			enterState(followState, rtti::typeName(typeid(*this)), "go back to drive, stop sign has vanished");
		}
		else if (isInState(STATE_CONTROLLEDBRAKE_GIVEWAY)) {
			enterState(followState, rtti::typeName(typeid(*this)), "go back to drive after give way");
		}
		else if (isInState(STATE_CONTROLLEDBRAKE_REVERSE)) {
			enterState(followState, rtti::typeName(typeid(*this)), "go back to drive, gear switch not needed anymore");
		}
		else if (isInState(STATE_CONTROLLEDBRAKE_DECISIONPOINT)) {
			enterState(followState, rtti::typeName(typeid(*this)), "go back to drive, decision point has vanished");
		}
	}

	// std::cout<<curSpeed << "\t" << wantedSpeed << "\t" << dom.first << "\t" << closestParamOnTrajectory << "\t" << dom.second;
	if (wantedSpeed == mFullStopSpeed.get()) {
		return wantedSpeed;
	}

	return rangeCut(mMinSpeed.get(), wantedSpeed, mMaxSpeed.get());
}


flt AbstractController::getSpeedProposalByEndSpline(flt distToEndSpline)
{
	//function depends on wanted target speed, acceleration, distance to point
	if (distToEndSpline <= mEndSplineSafetyDist.get()) {
		return mFullStopSpeed.get();
	}
	else if (distToEndSpline <= (mEndSplineSafetyDist.get() + mEndSplineRollingDist.get())) {
		return mEndSplineRollingSpeed.get();
	}
	else {
		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mEndSplineRollingSpeed.get(), 0.0, distToEndSpline - mEndSplineSafetyDist.get() - mEndSplineRollingDist.get());
	}
}

flt AbstractController::getSpeedProposalByGoal(flt distToGoal)
{
	//function depends on wanted target speed, acceleration, distance to point
	if (distToGoal <= mGoalSafetyDist.get()) {
		return mFullStopSpeed.get();
	}
	else if (distToGoal <= (mGoalSafetyDist.get() + mGoalRollingDist.get())) {
		return mGoalRollingSpeed.get();
	}
	else {
		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mGoalRollingSpeed.get(), 0.0, distToGoal - mGoalSafetyDist.get() - mGoalRollingDist.get());
	}
}

flt AbstractController::getSpeedProposalByTrafficLight(flt distToTrafficLight)
{
	//function depends on wanted target speed, acceleration, distance to point
	if (distToTrafficLight <= mTrafficLightSafetyDist.get()) {
		logDebug() << "traffic light: full stop";
		return mFullStopSpeed.get();
	}
	else if (distToTrafficLight <= (mTrafficLightSafetyDist.get() + mTrafficLightRollingDist.get())) {
		logDebug() << "traffic light: approaching";
		//return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mTrafficLightRollingSpeed.get(), 0.0, distToTrafficLight - mTrafficLightSafetyDist.get() - mTrafficLightRollingDist.get());

		return mTrafficLightRollingSpeed.get();
	}
	else {
		logDebug() << "traffic light: braking";
		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mTrafficLightRollingSpeed.get(), 0.0, distToTrafficLight - mTrafficLightSafetyDist.get() - mTrafficLightRollingDist.get());
	}
}

flt AbstractController::getSpeedProposalByStopSign(flt distToStopSign)
{
	//function depends on wanted target speed, acceleration, distance to point
	if (distToStopSign <= mStopSignSafetyDist.get()) {
		return mFullStopSpeed.get();
	}
	else if (distToStopSign <= (mStopSignSafetyDist.get() + mStopSignRollingDist.get())) {
		return mStopSignRollingSpeed.get();
	}
	else {
		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mStopSignRollingSpeed.get(), 0.0, distToStopSign - mStopSignSafetyDist.get() - mStopSignRollingDist.get());
	}
}

flt AbstractController::getSpeedProposalByGiveWay(flt distToGiveWay)
{
	//function depends on wanted target speed, acceleration, distance to point
	if (distToGiveWay <= mGiveWaySafetyDist.get()) {
		return mFullStopSpeed.get();
	}
	else if (distToGiveWay <= (mGiveWaySafetyDist.get() + mGiveWayRollingDist.get())) {
		return mGiveWayRollingSpeed.get();
	}
	else {
		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mGiveWayRollingSpeed.get(), 0.0, distToGiveWay - mGiveWaySafetyDist.get() - mGiveWayRollingDist.get());
	}
}

flt AbstractController::getSpeedProposalByReversePoint(flt distToReversePoint)
{
	//function depends on wanted target speed, acceleration, distance to point
	if (distToReversePoint <= mReversePointSafetyDist.get()) {
		return mFullStopSpeed.get();
	}
	else if (distToReversePoint <= (mReversePointSafetyDist.get() + mReversePointRollingDist.get())) {
		return mReversePointRollingSpeed.get();
	}
	else {
		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mReversePointRollingSpeed.get(), 0.0, distToReversePoint - mReversePointSafetyDist.get() - mReversePointRollingDist.get());
	}
}

flt AbstractController::getSpeedProposalByDecisionPoint(flt distToDecisionPoint)
{
	//function depends on wanted target speed, acceleration, distance to point
	if (distToDecisionPoint <= mDecisionPointSafetyDist.get()) {
		return mFullStopSpeed.get();
	}
	else if (distToDecisionPoint <= (mDecisionPointSafetyDist.get() + mDecisionPointRollingDist.get())) {
		return mDecisionPointRollingSpeed.get();
	}
	else {
		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, mDecisionPointRollingSpeed.get(), 0.0, distToDecisionPoint - mDecisionPointSafetyDist.get() - mDecisionPointRollingDist.get());
	}
}

/*
flt AbstractController::getSpeedProposalByObstacle(flt distToObstacle)			//new Method, not used
{

	//function depends on wanted target speed, acceleration, distance to point
//	if (distToObstacle <= (mObstacleSafetyDist.get())) {
//		return mFullStopSpeed.get();
//	}
//	else if (distToObstacle <= (mObstacleSafetyDist.get() + mObstacleRollingDist.get())) {
//		return mObstacleRollingSpeed.get();
//	}
//	else {
//		return getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort.get(), mObstacleRollingSpeed.get(), 0.0, distToObstacle - mObstacleSafetyDist.get() - mObstacleRollingDist.get());
//	}

	return 0.0;
}
*/


flt AbstractController::getSpeedProposalByCentrifugalForce(flt curSpeed, flt wantedSpeedSoFar)
{

	//init starts
	int previewDist = 320;
	flt sampleStepSize = mSampleCurvatureDistance; // race 8 //used to be 2

	//300 meter preview distance
	Plan const & curPlan = *mCurPlan;
    Vec3 * samples = new Vec3[previewDist];
	////Vec2 samplesDot[previewDist];
	flt curvature[previewDist];
	flt v[previewDist];
	flt vreg[previewDist];
	flt vmax =  mMaxSpeed.get();
	//int currentSample = 0;
	flt currentSample = 0;
	////flt currentSampleDot = 0;

	flt angle1, angle2;
	//init ends


	//acquire sample points

	//std::cout<<"ClosestParam " << mClosestParamOnTrajectory;
	//std::cout<<"check " << ((int)(mClosestParamOnTrajectory/sampleStepSize) * sampleStepSize);


	//std::cout<<"PreviewDist before: " << previewDist << " | After: " << (int)(min((flt)previewDist, curPlan.domain().second - mClosestParamOnTrajectory));

	//limit Preview distance by plan length
	previewDist = (int)(curPlan.domain().second - mClosestParamOnTrajectory);

	for (int i = 0; i < (previewDist/* ////- 0.01*/) / sampleStepSize; i++) {

		currentSample = rangeCut(curPlan.domain(), mClosestParamOnTrajectory + i * sampleStepSize);
		////currentSampleDot = rangeCut(curPlan.domain(), mClosestParamOnTrajectory + i * sampleStepSize + 0.01);


		samples[i] = curPlan(currentSample);
		////samplesDot[i] = curPlan(currentSampleDot);

		v[i] = vreg[i] = mMaxSpeed.get();
	}

	for (int i = 0; i < ((previewDist/* ////- 0.01*/) / sampleStepSize - 2); i++)  {

		angle2 = atan2(samples[i + 2][1] - samples[i + 1][1], samples[i + 2][0] - samples[i + 1][0]);
		angle1 = atan2(samples[i + 1][1] - samples[i][1], samples[i + 1][0] - samples[i][0]);

		curvature[i] = (normalizeAngle(angle2 - angle1)) / sampleStepSize;

//		std::cout << "Episode: " << i << ". |Classic Curv: " << curvature[i];


//		angle2 = atan2(samplesDot[i+1][1] - samples[i+1][1], samplesDot[i+1][0] - samples[i+1][0]);
//		angle1 = atan2(samplesDot[i][1] - samples[i][1], samplesDot[i][0] - samples[i][0]);


//		curvature[i] = (normalizeAngle(angle2 - angle1)) / sampleStepSize;

//		std::cout << "| New Curv: " << curvature[i] << std::endl;

		//COUT("xxxxxxxx i: " << i << " | samples[i][0]: " << samples[i][0] << " | samples[i][1]: " << samples[i][1]
		//                         << " | samples[i+1][0]: " << samples[i+1][0] << " | samples[i+1][1]: " << samples[i+1][1]
		//                         << " | samples[i+2][0]: " << samples[i+2][0] << " | samples[i+2][1]: " << samples[i+2][1]);

		//std::cout << " angle2 " << angle2 << " angle 1 " << angle1;

		if (curvature[i] != 0.0) {
			v[i] = sqrt(mCentrifugalAccelerationComfort / fabs(curvature[i]));   //F = v^2/r --> v = sqrt(F * r) = sqrt(F / c)
		}
		else {
			v[i] = mMaxSpeed.get();
		}

		if (mSoftBrakeTrajectory) { ///////
			//returnSquared = -2 * mLinear * (stopParam - curParam - stopSpeed) + pow(stopSpeed, 1.0 / mExponent);
			vreg[i] = pow((mSoftBrakeTrajectoryLin * 2.0 * (0.0 - (i) * sampleStepSize) + pow(v[i], 1.0 / mSoftBrakeTrajectoryExp)), mSoftBrakeTrajectoryExp);
			//std::cout << " i " << i << " vreg[i] " << vreg[i] << std::endl;
		}
		else {
			vreg[i] = sqrt(mBrakingAccelerationForCurveEntries * 2.0 * (0.0 - (i) * sampleStepSize) + v[i] * v[i]);  //we use the next curvature speed as the current one
		}


		if (vmax > vreg[i]) {
			vmax = vreg[i];
			//std::cout<<"... Curvature: " << curvature[i] << " | in i: " << i << " | v[i]: " << v[i] << " | vreg[i]: " << vreg[i];
		}

		//std::cout<<"****** " << i << " | " <<  curvature[i] << " angle1 " << angle1 << " angle2 " << angle2;

	}

	delete[] samples;

	//std::cout<<"AbstractController: " << 1 / curvature[0];
	//std::cout << " *** " << std::endl;
	//std::cout<<"VMAX: " << vmax;

	//std::cout<<"VMAX unfiltered: " << vmax;

	//std::cout<<"CentrifugalAcc" << mCentrifugalAccelerationComfort;

	//for smoother acceleration, assuming 25 Hz
	if (getPeriod() > 0.04) {
		logError() << "Behaviour is NOT executed with at least 25 Hz";
	}


	/* //Comment me in for ellipse function
	////////////
	//Elipse formula: x²/a² + y²/b² = 1 -->  y = \sqrt(b² (1 - x²/a²))
	flt a_z_max = mCentrifugalAccelerationComfort;
	flt a_b_max = mBrakingAccelerationForCurveEntries;
	flt v_max = mMaxSpeed.get();
	flt v_prop = v_max;
	flt a_b_current = 0;
	flt a_z_current = 0;


	//start from far to near:
	for (int i = (previewDist / sampleStepSize - 2) - 1; i >= 0; i--)  {        //curvature[0] --> curvature 8 meters in front and assumed until me (worst case)

		//std::cout<<"i: " << i << " | v_prop " << v_prop << " | curvature[i] " << curvature[i] << " | a_z_current " << v_prop * fabs(curvature[i]);

			a_z_current = v_prop * v_prop * fabs(curvature[i]);
			if (a_z_current >= a_z_max) { //too fast
				a_b_current = 0;
				if (fabs(curvature[i]) == 0) {logError()<<"Max Centrifugal Force == ZERO???"; v_prop = 0;} else
				{v_prop = min(v_max, sqrt(a_z_max / fabs(curvature[i])));}    //Drive through curve with max possible velocity
			} else {
				a_b_current = -1.0 * sqrt(a_b_max * a_b_max * (1 - (a_z_current * a_z_current) / (a_z_max * a_z_max)));
			}
			v_prop = min(v_max, sqrt(a_b_current * 2.0 * (-sampleStepSize) + v_prop * v_prop));
	}
	*/

	//std::cout<<">>>>>>>>>>> Speed with ellipse function: " << v_prop << " >>> old: " << vmax << " Diff: " << v_prop - vmax;
	///////////




	// for smoother acceleration after curves...
	lastVelocityFromCurvatureAndRoadSignPlanning = vmax; //min(vmax, lastVelocityFromCurvatureAndRoadSignPlanning + mThrottleAccelerationComfort * getPeriod());


	return lastVelocityFromCurvatureAndRoadSignPlanning;

	//return 10;
}





flt AbstractController::getSpeedProposalByCurvature(flt curSpeed, flt wantedSpeedSoFar)
{
	if (wantedSpeedSoFar == mFullStopSpeed.get()) {
		return mFullStopSpeed.get();
	}

	Plan const & curPlan = *mCurPlan;

	//rule of thumb: braking distance = v/10 * v/10
	//rule of thumb: reaction distance = v/10 * 3
	//rule of thumb: overall stopping distance = reaction distance + braking distance
	flt brake_dist = (curSpeed * MS_2_KMH / 10.0) * (curSpeed * MS_2_KMH / 10.0);
	flt reaction_dist = (curSpeed * MS_2_KMH / 10.0) * 3.0;
	flt lookahead_dist = 2.0 * (reaction_dist + brake_dist);
	lookahead_dist = max(flt(10.0), lookahead_dist);			//minimal 10 meters look ahead

	flt sampleStep = 0.1f;
	vector<flt> futureRadius;
	vector<flt> futureAvgRadius;

	flt distance = 0.0;

	while (distance < lookahead_dist) {
		if (mClosestParamOnTrajectory + distance > curPlan.domain().second) {
			break;
		}

		flt radius = min(flt(1000), ::math::radius(curPlan, mClosestParamOnTrajectory + distance));		//discard infinity
		futureRadius.push_back(radius);

		distance += sampleStep;
	}

	//smoothing with average filter of 8 meters
	flt runningAvg = 0;
	unsigned int averageWindow = ceil(3.0 / sampleStep);

	for (size_t i = 0; i < futureRadius.size(); i++) {
		if (i < averageWindow) {
			runningAvg += futureRadius[i];
		}
		else {
			if (i == averageWindow) {
				runningAvg /= averageWindow;
				futureAvgRadius.push_back(runningAvg);
			}

			runningAvg *= averageWindow;
			runningAvg -= futureRadius[i - averageWindow];
			runningAvg += futureRadius[i];
			runningAvg /= averageWindow;
			futureAvgRadius.push_back(runningAvg);
		}
	}


	//get minimum
	flt minFutureRadius = numeric_limits<flt>::infinity();

	for (size_t i = 0; i < futureAvgRadius.size(); i++) {
		if (futureAvgRadius[i] < minFutureRadius) {
			minFutureRadius = futureAvgRadius[i];
		}
	}

	flt wantedSpeed = mCurvatureMin.get() + mCurvatureSlope.get() * minFutureRadius;
// 	flt wantedSpeed = 1.4 + 0.03 * pow( minFutureRadius, 1.5);

// 	std::cout<<lookahead_dist << "\t" << minFutureRadius << "\t" << rangeCut(mMinSpeed.get(), wantedSpeed, mMaxSpeed.get()) << "\t" << mConstantSpeed.get();

	if (wantedSpeed == mFullStopSpeed.get()) {
		return wantedSpeed;
	}

	return rangeCut(mMinSpeed.get(), wantedSpeed, mMaxSpeed.get());
}


flt AbstractController::getSpeedProposalByObstacles(flt curSpeed, flt wantedSpeedSoFar)		//used
{
	if (wantedSpeedSoFar == mFullStopSpeed.get()) {
		return mFullStopSpeed.get();
	}

	flt wantedSpeed = wantedSpeedSoFar;



	//get statemachine functions
	OperationCaller<bool(int)> isInState(mStateMachine->getOperation("isInState"));
	OperationCaller<void(int, std::string, std::string)> enterState(mStateMachine->getOperation("enterState"));




	if (!mUseObstacles.get() || !mCurObstacles || mCurObstaclesOnSpline.empty()) {
		if (isInState(STATE_CONTROLLEDBRAKE_OBSTACLE) || isInState(STATE_FOLLOW)) {
			enterState(STATE_DRIVE, rtti::typeName(typeid(*this)), "no obstacles ahead");
		}

		return wantedSpeed;
	}

//	BaseObstacleMath::ObstTriple & closest = mCurObstaclesOnSpline.front();
	InterferingObstacle const closest = mCurObstaclesOnSpline.front();

	//FIXME the obstacle can be opposing
//	flt const finalSpeed = max(flt(0), closest.get<2>()->velocity().norm());
//	flt const distToCrash = abs(closest.get<1>() - mClosestParamOnTrajectory_Tip);

	flt const finalSpeed = max(0.0, closest.baseObstacle->velocity().norm());
	flt const distToCrash = abs(closest.obstParam - mClosestParamOnTrajectory_Tip);

	mDistToCrashOut.write(distToCrash);

// 	std::cout<<"dist to crash: " << distToCrash << "\tfinal speed: " << finalSpeed;




	//This method brakes in front of moving obstacles, used for follow behavior, it has its own BrakingAcceleration now (2011-03-24)
	if (mUseObstacleSpeed) { //if not undefined state
		if (closest.baseObstacle->movementState() != BaseObstacle::MOVEMENT_STATE_UNKNOWN) {
			//TODO: Check if obstacles on lane have defined of undefined state, here we used to have the bigger mBrakingAccelerationForObstacles before
			flt followDynamicObstacleSpeed = getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, finalSpeed, 0.0, ((flt)distToCrash - mFollowSafetyDist.get()), true);
			wantedSpeed = min(wantedSpeed, followDynamicObstacleSpeed);

			if (mPrintObstacleInfo) {
				std::cout << "OBST. NOT UNKNOWN, USE OS " << " || Obst Dist: " << distToCrash << " || ObstVel: " << finalSpeed << " || FollowSpeed: " << followDynamicObstacleSpeed;
			}
		}
		else {	// if undefined - use higher brake acceleration
			flt followDynamicObstacleSpeed = getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationForObstacles, finalSpeed, 0.0, ((flt)distToCrash - mFollowSafetyDist.get()), false);
			wantedSpeed = min(wantedSpeed, followDynamicObstacleSpeed);

			if (mPrintObstacleInfo) {
				std::cout << "OBST.UNKNOWN, USE OS " << " || Obst Dist: " << distToCrash << " || ObstVel: " << finalSpeed << " || FollowSpeed: " << followDynamicObstacleSpeed;
			}
		}
	}
	else {
		wantedSpeed = min(wantedSpeed, getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, 0.0 /*Obstacle Speed?*/, 0.0, ((flt)distToCrash - mFollowSafetyDist.get())));

		if (mPrintObstacleInfo) {
			logDebug() << "NOT USE OS " << " || Obst Dist: " << distToCrash << " || FollowSpeed: " <<
					   getStartVelocityForConstantBrakeAcceleration(mBrakingAccelerationComfort, 0.0 /*Obstacle Speed?*/, 0.0, ((flt)distToCrash - mFollowSafetyDist.get()));
		}
	}

	//std::cout<<"WantedSpeed " << wantedSpeed;

	if (isInState(STATE_CONTROLLEDBRAKE_OBSTACLE)) {
		enterState(STATE_CONTROLLEDBRAKE_OBSTACLE, rtti::typeName(typeid(*this)), (format("goto brake because static obstacle has been encountered in %1% m") % distToCrash).str());

	}
	else if (isInState(STATE_FOLLOW)) {
		enterState(STATE_FOLLOW, rtti::typeName(typeid(*this)), (format("follow dynamic obstacle in %1% m with speed: %2% km/h") % distToCrash % (finalSpeed * MS_2_KMH)).str());

	}


	if (wantedSpeed == mFullStopSpeed.get()) {
		return wantedSpeed;
	}

	return rangeCut(mMinSpeed.get(), wantedSpeed, mMaxSpeed.get());
}


flt AbstractController::getSpeedProposalBySurface(flt curSpeed, flt wantedSpeedSoFar)
{
	if (wantedSpeedSoFar == mFullStopSpeed.get()) {
		return mFullStopSpeed.get();
	}

	//TODO not implemented yet
	return 0;
}



int AbstractController::getGearPosition()
{
	Logger::In in("Controller");

	int oldGear = mCurCarState.gearPosition;
	int newGear = controlGear();

	if (newGear == oldGear) {
		return newGear;
	}


	if (mPassatCarStateIn.connected()) {

		if (((mCurPassatCarState.wheelSpeeds.speedAvg() == 0) && (mCurPassatCarState.brakeStatus1.actualPressure() > (0.1 * 127.5)))
				|| (mAllowDriverIntervention)) {
			return newGear;
		}
		else {
			logWarning() << "Abstract Controller: not standing or too few brake pressure, denying gear change on mig";
			return oldGear;
		}
	}


	if (mEgoStateIn.connected() && mCarStateIn.connected()) {
		if (mCurEgoState.vehicleSpeed() == 0 && (mCurCarState.gasPosition < 0.1 || std::isnan(mCurCarState.gasPosition))) {
			return newGear;
		}
	}

	return oldGear;
}


flt AbstractController::getThrottleBrakePosition(flt curSpeed, flt wantedSpeed)
{
	Logger::In in("Controller");

	// prevent illegal output, if properties are set wrong
	assert(mMaxAccel.get() >= 0.0 && mMaxAccel.get() <= 1.0);
	assert(mMaxBrake.get() >= -1.0 && mMaxBrake.get() <= 0.0);

	flt output = controlThrottleBrake(curSpeed, wantedSpeed);

	if (std::isnan(output)) {
		output = 0;
	}

	output = rangeCut(mMaxBrake.get(), output, mMaxAccel.get());

	return output;
}



flt AbstractController::getSteeringPosition(Plan_ptr plan)
{
	Logger::In in("Controller");

	if (mSteerControlMode == CONSTANT_STEER) {					// WARNING constant steer is for testing purposes only
		return mConstantSteer.get();
	}

	//center wheel in standstill
// 	if(curControllerData.wantedSpeed == mFullStopSpeed.get()) {
// 		return 0;
// 	}

	flt output = controlSteering(plan);

	if (std::isnan(output)) {
		output = 0;
	}

	output = rangeCut(mMaxLeftSteer.get(), output, mMaxRightSteer.get());

	return output;
}





AuxDevicesData AbstractController::getAuxDevicesData(flt curSpeed, Plan_ptr plan)
{
	Logger::In in("Controller");

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

	boost::optional<Plan::action_descr> nextlaneChangeOrTurnAction = curPlan.findFirstAction(Plan::LANE_CHANGE, Plan::TURN, dom.first, dom.second);

	if (nextlaneChangeOrTurnAction) {
		Plan::action_descr action = nextlaneChangeOrTurnAction.get();

		//is there decision point prior
		boost::optional<Plan::action_descr> nextDecisionpointAction = curPlan.findFirstAction(Plan::DECISION_POINT, dom.first, dom.second);

		if (!nextDecisionpointAction || nextDecisionpointAction.get().get<1>() > action.get<1>()) {
			flt const actionStart = action.get<1>();
			flt const actionEnd = action.get<2>();
			int const actionIndex = any_cast<int>(action.get<3>());

			flt preDist = max(mMinTurnSignalDistance.get(), mTurnSignalTime.get() * curSpeed);

			if (actionStart - preDist <= mClosestParamOnTrajectory && mClosestParamOnTrajectory <= actionEnd) {
				if (actionIndex > 0) {
					auxDevicesCorrection.turnsignalState = AuxDevicesData::TURNSIGNAL_RIGHT;
				}
				else if (actionIndex < 0) {
					auxDevicesCorrection.turnsignalState = AuxDevicesData::TURNSIGNAL_LEFT;
				}
				else {
					auxDevicesCorrection.turnsignalState = AuxDevicesData::TURNSIGNAL_OFF;
				}
			}
		}
	}

	return auxDevicesCorrection;
}







/** States */

bool AbstractController::resumeDrive()
{
	Logger::In in("Controller");

	if (!mStateMachine) {
		mStateMachine = findPeer(this, "StateMachine");

		if (!mStateMachine) {
			return false;
		}
	}

	//get statemachine functions
	OperationCaller<void(int, std::string, std::string)> enterState(mStateMachine->getOperation("enterState"));

	enterState(STATE_DRIVE, rtti::typeName(typeid(*this)), "resuming drive by manual method call");

	return true;
}

bool AbstractController::pauseDrive()
{
	Logger::In in("Controller");

	//get statemachine functions
	OperationCaller<bool(int)> isInState(mStateMachine->getOperation("isInState"));
	OperationCaller<void(int, std::string, std::string)> enterState(mStateMachine->getOperation("enterState"));

	if (isInState(CONTEXT_STOP) || isInState(STATE_CONTROLLEDBRAKE_PAUSE)) {
		return true;
	}

	enterState(STATE_CONTROLLEDBRAKE_PAUSE, rtti::typeName(typeid(*this)), "pausing drive by manual method call");

	return true;
}



bool AbstractController::handleMalfunction(std::string reason)
{
	Logger::In in("Controller");

	//get statemachine functions
	OperationCaller<bool(int)> isInState(mStateMachine->getOperation("isInState"));
	OperationCaller<void(int, std::string, std::string)> enterState(mStateMachine->getOperation("enterState"));

	if (isInState(CONTEXT_STOP) || isInState(STATE_CONTROLLEDBRAKE_MALFUNCTION)) {
		return true;
	}

	if (reason == "") {
		reason = "unknown (reason not given)";
	}

	enterState(STATE_CONTROLLEDBRAKE_MALFUNCTION, rtti::typeName(typeid(*this)), (format("malfunction encountered: %1%") % reason).str());

	return true;
}

bool AbstractController::resetProcessedActions()
{
	processedActions.clear();
	return true;
}


/** Speed modes */

bool AbstractController::useFullStopMode()
{
	mSpeedControlMode = FULL_STOP;
	return true;
}

bool AbstractController::useConstantThrottleMode()
{
	mSpeedControlMode = CONSTANT_THROTTLE;
	return true;
}

bool AbstractController::useConstantSpeedMode()
{
	mSpeedControlMode = CONSTANT_SPEED;
	return true;
}

bool AbstractController::useVariableSpeedMode()
{
	mSpeedControlMode = VARIABLE_SPEED;
	return true;
}


/** Steer modes */

bool AbstractController::useConstantSteerMode()
{
	mSteerControlMode = CONSTANT_STEER;
	return true;
}

bool AbstractController::useVariableSteerMode()
{
	mSteerControlMode = VARIABLE_STEER;
	return true;
}

flt AbstractController::normalizeAngle(flt angle)
{
	if (angle < -PI) {
		return angle + 2 * PI;
	}
	else if (angle >= PI) {
		return angle - 2 * PI;
	}
	else {
		return angle;
	}

}


bool AbstractController::printOutput()
{
	//get statemachine functions
	OperationCaller<void(void)> printState(mStateMachine->getOperation("printState"));
	OperationCaller<void(void)> printControlMode(mStateMachine->getOperation("printControlMode"));


	std::cout << std::endl << "StateMachine: ";
	printState();
	printControlMode();
	std::cout << "";

	return true;
}

bool AbstractController::forceTrafficLightRed()
{
	if (!mTrafficLightVertexName.empty()) {
		try {
            vertex_descr trafficLightVertex = mARNDGraph.getVertex(mTrafficLightVertexName);

			if (trafficLightVertex != vertex_descr()) {
                aGraph & graph = const_cast<aGraph &>(mARNDGraph.getBoostGraph());
				boost::property_map<aGraph, vertex_data_t>::type vertexDataMap = get(vertex_data_t(), graph);
				VertexData & vertexData = vertexDataMap[trafficLightVertex];
				vertexData.metaData = VertexData::FORCE_RED;
			}

			logError() << "Force traffic light red.";
			return true;
		}
		catch (const boost::bad_any_cast &) {
			logError() << "Got exception. Failed to force traffic light red.";
			return false;
		}
	}

	logError() << "Failed to force traffic light red.";
	return false;
}

bool AbstractController::forceTrafficLightGreen()
{
	if (!mTrafficLightVertexName.empty()) {
		try {
            vertex_descr trafficLightVertex = mARNDGraph.getVertex(mTrafficLightVertexName);

			if (trafficLightVertex != vertex_descr()) {
                aGraph & graph = const_cast<aGraph &>(mARNDGraph.getBoostGraph());
				boost::property_map<aGraph, vertex_data_t>::type vertexDataMap = get(vertex_data_t(), graph);
				VertexData & vertexData = vertexDataMap[trafficLightVertex];
				vertexData.metaData = VertexData::FORCE_GREEN;
			}

			logError() << "Force traffic light green. ";
			return true;
		}
		catch (const boost::bad_any_cast &) {
			logError() << "Got exception. Failed to force traffic light green." << mTrafficLightVertexName;
			return false;
		}
	}

	logError() << "Failed to force traffic light green." << mTrafficLightVertexName;
	return false;
}

}
}
}
}
