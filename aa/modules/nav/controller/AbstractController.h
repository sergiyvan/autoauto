#pragma once

#include <util/RtTaskContext.h>
#include <util/Ports.h>
#include <core/TimedData.h>
#include <iostream>
#include <deque>

#include "Plan.h"
#include "ComfortSettings.h"
#include "data/ControllerData.h"

#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <modules/models/egostate/EgoState.h>
#include <modules/models/carstate/CarState.h>
#include <aa/modules/models/carstate/PassatCarState.h>
#include <modules/models/carstate/AuxDevicesData.h>
#include <aa/modules/nav/obstacles/ObstaclesOnSpline.h>



#define FULL_STOP					0
#define CONSTANT_THROTTLE			1
#define CONSTANT_SPEED				2
#define VARIABLE_SPEED				3

#define CONSTANT_STEER				10
#define VARIABLE_STEER				11


namespace aa
{
namespace modules
{
namespace nav
{
namespace controller
{

class AbstractController
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Vec4 Vec4;
	typedef TimedData< PlainDataHolder<flt> > TimedFlt;
	typedef TimedData< PlainDataHolder<bool> > TimedBool;

    typedef math::PolySpline<Vec3, flt, 4u> SplineType;
	typedef aa::modules::nav::obstacles::ObstaclesOnSpline<SplineType, BaseObstacleBundle> ObstacleMath;
	typedef ObstacleMath::InterferingObstacle InterferingObstacle;

	explicit AbstractController(std::string const & name);
	virtual ~AbstractController();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void errorHook();

	data::ControllerData simulateStep(Plan_ptr plan, TimedEgoState egoState, ::modules::models::carstate::TimedCarState carState, TimedBaseObstacleBundle_ptr obstacles);

private:

	/** @name OutputPorts: */
	//@{
	RTT::OutputPort< flt > mSteerOut;
	RTT::OutputPort< TimedFlt > mSpeedOut;
	RTT::OutputPort< int > mGearOut;
	RTT::OutputPort< AuxDevicesData > mAuxDevicesOut;
	RTT::OutputPort< bool > mActivationRequestOut;

	RTT::OutputPort< flt > mWantedSpeedOut;
	RTT::OutputPort< flt > mWantedSpeedByObstaclesOut;
	RTT::OutputPort< flt > mDistToCrashOut;
	RTT::OutputPort< data::TimedControllerData > mControllerDataOut;
//	RTT::OutputPort< ObstacleMath::InterferingObstacleList > mInterferingObstaclesOut;
	RTT::OutputPort< TimedBaseObstacleBundle_ptr > mInterferingObstaclesOut;

	RTT::OutputPort< unsigned int > mBrakingObstacleIdOut;		//for debug purpose

	//@}

protected:

//	/** @name OutputPorts: */
	//	RTT::OutputPort<Vec3> mLearningSuccessOut;
	RTT::OutputPort<bool> mResetRequestOut;

	/** @name InputPorts: */
	//@{
	RTT::InputPort< Plan_ptr > mPlanIn;
	RTT::InputPort< TimedEgoState > mEgoStateIn;
	RTT::InputPort< ::modules::models::carstate::TimedCarState > mCarStateIn;
	RTT::InputPort< aa::modules::models::carstate::TimedPassatCarState > mPassatCarStateIn;
	RTT::InputPort< TimedBaseObstacleBundle_ptr > mObstaclesIn;
//	RTT::InputPort<bool> mResetNewScenarioIn;
	RTT::InputPort<bool> mResumeDriveIn;
	RTT::InputPort< flt > mSideObstacleDistLeftIn;
	RTT::InputPort< flt > mSideObstacleDistRightIn;
	//@}


	/** @name Attributes: general attributes */
	RTT::Attribute<int> mCounter;
	RTT::Attribute<int> mDebugPrintWantedSpeedIntermediateSteps;


	/** @name Properties: general properties */
	RTT::Property<bool> mEngage;
	RTT::Property<flt> mMaxBrake;
	RTT::Property<flt> mMaxAccel;
	RTT::Property<flt> mMaxLeftSteer;
	RTT::Property<flt> mMaxRightSteer;
	RTT::Property<flt> mMinSpeed;
	RTT::Property<flt> mMaxSpeed;
	RTT::Property<flt> mMinZoneSpeed;
	RTT::Property<flt> mMaxZoneSpeed;
	RTT::Property<flt> mWalkingSpeed;
	RTT::Property<flt> mStopSpeed;
	RTT::Property<flt> mFullStopSpeed;
	RTT::Property<flt> mTrafficLightPointOfNoReturnRatio;
	bool mTrafficLightBrakeInFrontPointOfNoReturn;
	std::string mTrafficLightVertexName;
	RTT::Property<bool> mUseMissionSpeedLimits;
	RTT::Property<bool> mSimulationMode;

	RTT::Property<bool> mLogWantedSpeed;
	int mNumberOfCarStatePorts;

	/** loaded from ComfortSettings */
	flt mCentrifugalAccelerationComfort;
	flt mThrottleAccelerationComfort;
	flt mBrakingAccelerationComfort;
	flt mBrakingAccelerationForObstacles;
	flt mBrakingAccelerationForCurveEntries;
	flt mSampleCurvatureDistance;


	/** @name Properties: look ahead function properties */
	RTT::Property<flt> mLookAheadSlope;
	RTT::Property<flt> mLookAheadMax;


	/** @name Properties: speed proposal properties */
	RTT::Property<flt> mMinFreeTrackWidth;
	RTT::Property<flt> mFreeMargin;


	/** @name Properties: curvature properties */
	RTT::Property<flt> mCurvatureSlope;
	RTT::Property<flt> mCurvatureMin;

	/** @name Properties: throttle window by speed properties */
	RTT::Property<bool> mUseThrottleBySpeedWindow;
	RTT::Property<flt> mThrottleBySpeedWindowStart;
	RTT::Property<flt> mThrottleBySpeedWindowEnd;
	RTT::Property<flt> mMaxBrakeBeforeThrottleBySpeedWindow;
	RTT::Property<flt> mMaxAccelBeforeThrottleBySpeedWindow;
	RTT::Property<flt> mMaxBrakeAfterThrottleBySpeedWindow;
	RTT::Property<flt> mMaxAccelAfterThrottleBySpeedWindow;
	RTT::Property<flt> mMaxAccelStartThrottleBySpeed;

	/** @name Properties: steer window by speed properties */
	RTT::Property<bool> mUseSteerBySpeedWindow;
	RTT::Property<flt> mSteerBySpeedWindowStart;
	RTT::Property<flt> mSteerBySpeedWindowEnd;
	RTT::Property<flt> mMaxLeftSteerBeforeSteerBySpeedWindow;
	RTT::Property<flt> mMaxRightSteerBeforeSteerBySpeedWindow;
	RTT::Property<flt> mMaxLeftSteerAfterSteerBySpeedWindow;
	RTT::Property<flt> mMaxRightSteerAfterSteerBySpeedWindow;

	/** @name Properties: throttle window by steer properties */
	RTT::Property<bool> mUseThrottleBySteerWindow;
	RTT::Property<flt> mThrottleBySteerWindowStart;
	RTT::Property<flt> mThrottleBySteerWindowEnd;
	RTT::Property<flt> mMaxBrakeBeforeThrottleBySteerWindow;
	RTT::Property<flt> mMaxAccelBeforeThrottleBySteerWindow;
	RTT::Property<flt> mMaxBrakeAfterThrottleBySteerWindow;
	RTT::Property<flt> mMaxAccelAfterThrottleBySteerWindow;


	/** @name Properties: goal properties */
	RTT::Property<bool> mStopAtGoal;
	RTT::Property<flt> mGoalSafetyDist;
	RTT::Property<flt> mGoalRollingDist;
	RTT::Property<flt> mGoalRollingSpeed;
	RTT::Property<flt> mGoalMinLookaheadDist;

	/** @name Properties: end spline properties */
	RTT::Property<bool> mStopAtEndSpline;
	RTT::Property<flt> mEndSplineSafetyDist;
	RTT::Property<flt> mEndSplineRollingDist;
	RTT::Property<flt> mEndSplineRollingSpeed;
	RTT::Property<flt> mEndSplineMinLookaheadDist;

	/** @name Properties: traffic light sign properties */
	RTT::Property<bool> mStopAtTrafficLight;
	RTT::Property<flt> mTrafficLightSafetyDist;
	RTT::Property<flt> mTrafficLightRollingDist;
	RTT::Property<flt> mTrafficLightRollingSpeed;
	RTT::Property<flt> mTrafficLightMinLookaheadDist;

	/** @name Properties: stop sign properties */
	RTT::Property<bool> mStopAtStopSign;
	RTT::Property<flt> mStopSignSafetyDist;
	RTT::Property<flt> mStopSignRollingDist;
	RTT::Property<flt> mStopSignRollingSpeed;
	RTT::Property<flt> mStopSignMinLookaheadDist;

	/** @name Properties: give way properties */
	RTT::Property<bool> mStopAtGiveWay;
	RTT::Property<flt> mGiveWaySafetyDist;
	RTT::Property<flt> mGiveWayRollingDist;
	RTT::Property<flt> mGiveWayRollingSpeed;
	RTT::Property<flt> mGiveWayMinLookaheadDist;

	/** @name Properties: reverse point properties */
	RTT::Property<bool> mStopAtReversePoint;
	RTT::Property<flt> mReversePointSafetyDist;
	RTT::Property<flt> mReversePointRollingDist;
	RTT::Property<flt> mReversePointRollingSpeed;
	RTT::Property<flt> mReversePointMinLookaheadDist;

	/** @name Properties: decision point properties */
	RTT::Property<bool> mStopAtDecisionPoint;
	RTT::Property<flt> mDecisionPointSafetyDist;
	RTT::Property<flt> mDecisionPointRollingDist;
	RTT::Property<flt> mDecisionPointRollingSpeed;
	RTT::Property<flt> mDecisionPointMinLookaheadDist;


	/** @name Properties: obstacle properties */
	RTT::Property<bool> mUseObstacles;
	RTT::Property<bool> mCheckForObstacleMalfunction;
	RTT::Property<flt> mObstacleSafetyDist;
	RTT::Property<flt> mObstacleRollingDist;
	RTT::Property<flt> mObstacleRollingSpeed;
	RTT::Property<flt> mFollowSafetyDist;


	/** @name Properties: constant throttle controller properties */
	RTT::Property<flt> mConstantThrottle;

	/** @name Properties: constant speed controller properties */
	RTT::Property<flt> mConstantSpeed;

	/** @name Properties: constant steer controller properties */
	RTT::Property<flt> mConstantSteer;

	/** @name Properties: driver intervention properties */
	RTT::Property<bool> mAllowDriverIntervention;

	/** @name Properties: turn signal properties */
	RTT::Property<flt> mTurnSignalTime;
	RTT::Property<flt> mMinTurnSignalDistance;

	/** @name Properties: use obstacle speed */
	RTT::Property<bool> mUseObstacleSpeed;
	RTT::Property<bool> mSportySet;
	RTT::Property<bool> mUltimateSet;

	RTT::Property<bool> mSoftBrakeTrajectory;

	/** @name Properties: debug properties for PlotView */
	RTT::Property<flt> mDebug1;
	RTT::Property<flt> mDebug2;
	RTT::Property<flt> mDebug3;

	/** @name Properties: print Out Flag for Obstacles */
	RTT::Property<bool> mPrintObstacleInfo;

	/** @name Properties: predict movement of obstacles if tracker data is sparse in time */
	RTT::Property<bool> mPredictUnknownObstacles;
	RTT::Property<long long> mPredictUnknownObstaclesThreshold;



	/** members */
	Plan_ptr mCurPlan;
	TimedEgoState mCurEgoState;
	::modules::models::carstate::TimedCarState mCurCarState;
	aa::modules::models::carstate::TimedPassatCarState mCurPassatCarState;
	TimedBaseObstacleBundle_ptr mCurObstacles;

    ::aa::modules::models::rndf::RNDFGraph & mARNDGraph;
	ComfortSettings & mComfortSettings;
	RTT::TaskContext * mStateMachine;

	std::deque<data::TimedControllerData> mRecentControllerData;
	data::TimedControllerData mCurControllerData;
	data::TimedControllerData mLastControllerData;

#if 0
	SpeedRamp speedRamp;
	bool setRamp(std::vector< std::pair<flt, flt> > ramp, int successingState, bool force = false);
#endif

private:
	std::vector<Plan::action_descr> processedActions;
	ObstacleMath::InterferingObstacleList mCurObstaclesOnSpline;

	TimeStamp now;

protected:
	//position of front axle (main position)
	flt mClosestParamOnTrajectory;
	flt mClosestSqrDistToTrajectory;
    Vec3 mClosestPosOnTrajectory;

	//position of rear axle
	flt mClosestParamOnTrajectory_Rear;
	flt mClosestSqrDistToTrajectory_Rear;
    Vec3 mClosestPosOnTrajectory_Rear;

	//position for front tip of car
	flt mClosestParamOnTrajectory_Front;
	flt mClosestSqrDistToTrajectory_Front;
    Vec3 mClosestPosOnTrajectory_Front;

	//position for back tip of car
	flt mClosestParamOnTrajectory_Back;
	flt mClosestSqrDistToTrajectory_Back;
    Vec3 mClosestPosOnTrajectory_Back;

	//position of tip of the car (either front or back tip depending on gear)
	flt mClosestParamOnTrajectory_Tip;
	flt mClosestSqrDistToTrajectory_Tip;
    Vec3 mClosestPosOnTrajectory_Tip;

	flt lastVelocityFromCurvatureAndRoadSignPlanning; //for more smooth acceleration after curves and stop signs

	flt lateralErrorToTrajectoryCumulated;
	flt mSoftBrakeTrajectoryExp;
	flt mSoftBrakeTrajectoryLin;

	//find forward/reverse segment
	std::pair<flt, flt> getRelativeForwardReverseSegment(Plan_ptr plan, flt param_tip) const;

	//get speed limits
	std::pair<flt, flt> getSpeedLimitAtPlanStart(Plan_ptr plan) const;
	std::pair<flt, flt> getAvgSpeedLimitForConnectingEdge(::aa::modules::models::rndf::edge_descr const & conEdge) const;

	//window functions
	std::pair<flt, flt> getThrottleBySpeedWindow(flt curSpeed) const;
	std::pair<flt, flt> getSteerBySpeedWindow(flt curSpeed) const;
	std::pair<flt, flt> getThrottleBySteerWindow(flt steer) const;

	//slow down functions to build ramp, return vector of <speed,param>
	std::vector< std::pair<flt, flt> > slowDownLinear(flt curSpeed, flt stopSpeed, flt curParam, flt stopParam) const;
	std::vector< std::pair<flt, flt> > slowDownProgressive(flt curSpeed, flt stopSpeed, flt curParam, flt stopParam) const;
	std::vector< std::pair<flt, flt> > slowDownDegressive(flt curSpeed, flt stopSpeed, flt curParam, flt stopParam) const;
	flt getBrakingDistanceForConstantBrakeAcceleration(flt acceleration, flt startVelocity) const;
	flt getStartVelocityForConstantBrakeAcceleration(flt acceleration, flt stopSpeed, flt curParam, flt stopParam, bool includeSafety = false) const;

	bool isActionProcessed(Plan::action_descr action) const;



	//STEP 1: check state machine
	/** update the state machine for each iteration */
	void processStateMachine();

	//STEP 2: check obstacles
	/** mark interfering obstacles for each iteration
	* @param curSpeed current speed in m/s
	*/
	void processObstacles(flt curSpeed);


	//STEP 3: get a wanted speed porposal
	/** calculate a speed proposal for the current situation
	* @param curSpeed current speed in m/s
	* @return speed in m/s
	*/
	flt getWantedSpeed(flt curSpeed);

	/** calculate a speed proposal based on speed limits of the current edge and considers end of spline
	* @param curSpeed current speed in m/s
	* @param wantedSpeedSoFar wanted speed until now
	* @return speed in m/s
	*/
	flt getSpeedProposalByTrajectory(flt curSpeed, flt wantedSpeedSoFar);



	/** helper function: calculate a speed proposal based on distance to end of spline and comfortable braking acceleration */
	flt getSpeedProposalByEndSpline(flt distToEndSpline);

	/** helper function: calculate a speed proposal based on distance to goal and comfortable braking acceleration */
	flt getSpeedProposalByGoal(flt distToGoal);

	/** helper function: calculate a speed proposal based on distance to traffic light and comfortable braking acceleration */
	flt getSpeedProposalByTrafficLight(flt distToTrafficLight);

	/** helper function: calculate a speed proposal based on distance to stop sign and comfortable braking acceleration */
	flt getSpeedProposalByStopSign(flt distToStopSign);

	/** helper function: calculate a speed proposal based on distance to give way and comfortable braking acceleration */
	flt getSpeedProposalByGiveWay(flt distToGiveWay);

	/** helper function: calculate a speed proposal based on distance to reverse point and comfortable braking acceleration */
	flt getSpeedProposalByReversePoint(flt distToReversePoint);

	/** helper function: calculate a speed proposal based on distance to decision point and comfortable braking acceleration */
	flt getSpeedProposalByDecisionPoint(flt distToDecisionPoint);

	/** helper function: calculate a speed proposal based on distance to obstacle and comfortable braking acceleration */
//	flt getSpeedProposalByObstacle(flt distToObstacle);


	/** calculate a speed proposal based on future curvature and max centrifugal and brake acceleration
	 *
	 * @param curSpeed
	 * @param wantedSpeedSoFar
	 * @return
	 */
	flt getSpeedProposalByCentrifugalForce(flt curSpeed, flt wantedSpeedSoFar);

	/** calculate a speed proposal based on current curvature and future curvature
	* @param curSpeed current speed in m/s
	* @param wantedSpeedSoFar wanted speed until now
	* @return speed in m/s
	*/
	flt getSpeedProposalByCurvature(flt curSpeed, flt wantedSpeedSoFar);

	/** calculate a speed proposal based on nearby obstacles
	* @param curSpeed current speed in m/s
	* @param wantedSpeedSoFar wanted speed until now
	* @return speed in m/s
	*/
	flt getSpeedProposalByObstacles(flt curSpeed, flt wantedSpeedSoFar);

	/** calculate a speed proposal based on surface quality
	* @param curSpeed current speed in m/s
	* @param wantedSpeedSoFar wanted speed until now
	* @return speed in m/s
	*/
	flt getSpeedProposalBySurface(flt curSpeed, flt wantedSpeedSoFar);



	//STEP 4: get gear position
	//NOTE pure virtual function MUST be implemented by non-abstract child classes
	/** return wanted gear position
	* @return a value conforming to 1=P, 2=R, 4=N, 8=D (reserved: -1=error and 0=between gear)
	*/
	int getGearPosition();
	virtual int controlGear() = 0;



	//STEP 5: get a throttle position for wanted speed
	//NOTE pure virtual function MUST be implemented by non-abstract child classes
	/** calculate a throttle position given current and wanted speed (uses either linear, pid controller)
	* @param curSpeed current speed of the car in m/s
	* @param wantedSpeed desired speed of the car in m/s
	* @return a value between -1 and 1 indicating gas/brake pedal position
	*/
	flt getThrottleBrakePosition(flt curSpeed, flt wantedSpeed);
	virtual flt controlThrottleBrake(flt curSpeed, flt wantedSpeed) = 0;



	//STEP 6: get a steering position for wanted speed
	//NOTE pure virtual function MUST be implemented by non-abstract child classes
	/** calculate a steering position
	* @return a value between -1 and 1 indicating steering position
	*/
	flt getSteeringPosition(Plan_ptr plan);
	virtual flt controlSteering(Plan_ptr plan) = 0;



	//STEP 7: get auxiliary devices data (headlights, siren, wiper, turninglights)
	/** retrieve data for auxiliary devices
	 * @return a auxiliary device data
	 */
	AuxDevicesData getAuxDevicesData(flt curSpeed, Plan_ptr plan);




	/** resume to drive state when in timed full stop or permanent full stop */
	bool resumeDrive();
	bool pauseDrive();
	bool handleMalfunction(std::string reason = "");

	/** reset processedActions */
	bool resetProcessedActions();


	/** speed mode members */
	int mSpeedControlMode;
	/** use full stop mode: e.g. max brake all the time */
	bool useFullStopMode();
	/** use constant throttle mode: e.g. constant gas pedal position all the time defined in mConstantThrottle */
	bool useConstantThrottleMode();
	/** use constant speed mode: e.g. try to maintain a constant speed defined in mConstantSpeed */
	bool useConstantSpeedMode();
	/** use variable speed mode */
	bool useVariableSpeedMode();


	/** steer mode members */
	int mSteerControlMode;
	/** use constant steer mode: e.g. constant steering position all the time defined in mConstantSteer */
	bool useConstantSteerMode();
	/** use variable steer mode */
	bool useVariableSteerMode();

	flt normalizeAngle(flt angle);




	bool printOutput();

	bool forceTrafficLightRed();
	bool forceTrafficLightGreen();

};

}
}
}
}
