#include "ControllerData.h"

#include <data/VehicleData.h>
#include <math/PathSpline.h>
#include <math/AutoMath.h>

#include <aa/modules/nav/statemachine/StateMachine.h>

namespace aa
{
namespace modules
{
namespace nav
{
namespace controller
{
namespace data
{

using namespace std;
using namespace RTT;
using namespace boost;
using namespace ::math;
using namespace ::modules::models::carstate;

ControllerData::ControllerData()
	: carPos(NAN, NAN, NAN)
	, carDir(NAN, NAN, NAN)
	, curSpeed(NAN)
	, yawRateMeasured(NAN)
	, steeringAngleMeasured(NAN)
	, throttlePositionMeasured(NAN)

	, autonomousControl(false)
	, gearPosition(-1)

	, trajDir(NAN, NAN, NAN)
	, wantedCarDir(NAN, NAN)
	, yawRateTrajectory(NAN)

	, lookAheadDist(NAN)
	, wantedSpeed(NAN)
	, speedCorrection(NAN)
	, steerCorrection(NAN)
	, gearCorrection(-1)

	, idleZone(NAN, NAN)
	, comfortZone(NAN, NAN)
	, throttleWindow(NAN, NAN)
	, steerWindow(NAN, NAN)

	, lateralError(NAN)
	, headingError(NAN)

	, projectedFrontAxlePos(NAN, NAN, NAN)
	, projectedRearAxlePos(NAN, NAN, NAN)
	, projectedFrontTipPos(NAN, NAN, NAN)
	, projectedBackTipPos(NAN, NAN, NAN)

	, minSpeedLimit(NAN)
	, maxSpeedLimit(NAN)

	, curStateName("")
{
}

ControllerData::ControllerData(const TimedEgoState & curEgoState, TimedCarState const & curCarState, std::string state)
	: carPos(getCarPos(curEgoState))
	, carDir(getCarDir(curEgoState))
	, curSpeed(getCurSpeed(curEgoState))
	, yawRateMeasured(getYawRateMeasured(curEgoState))
	, steeringAngleMeasured(getSteeringAngleMeasured(curCarState))
	, throttlePositionMeasured(getThrottlePositionMeasured(curCarState))

	, autonomousControl(curCarState.autonomousControl)
	, gearPosition(curCarState.gearPosition)

	, trajDir(NAN, NAN, NAN)
	, wantedCarDir(NAN, NAN)
	, yawRateTrajectory(NAN)

	, lookAheadDist(NAN)
	, wantedSpeed(NAN)
	, speedCorrection(NAN)
	, steerCorrection(NAN)
	, gearCorrection(-1)

	, idleZone(NAN, NAN)
	, comfortZone(NAN, NAN)
	, throttleWindow(NAN, NAN)
	, steerWindow(NAN, NAN)

	, lateralError(NAN)
	, headingError(NAN)

	, projectedFrontAxlePos(NAN, NAN, NAN)
	, projectedRearAxlePos(NAN, NAN, NAN)
	, projectedFrontTipPos(NAN, NAN, NAN)
	, projectedBackTipPos(NAN, NAN, NAN)

	, minSpeedLimit(NAN)
	, maxSpeedLimit(NAN)

	, curStateName(state)
{
}

ControllerData::ControllerData(TimedEgoState const & curEgoState, TimedCarState const & curCarState, Plan_ptr plan, flt param, flt sqrDist,
                               flt lookAheadDist, flt wantedSpeed, flt speedCorrection, flt steerCorrection, flt gearCorrection, std::string state)
	: carPos(getCarPos(curEgoState))
	, carDir(getCarDir(curEgoState))
	, curSpeed(getCurSpeed(curEgoState))
	, yawRateMeasured(getYawRateMeasured(curEgoState))
	, steeringAngleMeasured(getSteeringAngleMeasured(curCarState))
	, throttlePositionMeasured(getThrottlePositionMeasured(curCarState))

	, autonomousControl(curCarState.autonomousControl)
	, gearPosition(curCarState.gearPosition)

	, lookAheadDist(lookAheadDist)
	, wantedSpeed(wantedSpeed)
	, speedCorrection(speedCorrection)
	, steerCorrection(steerCorrection)
	, gearCorrection(gearCorrection)

	, idleZone(NAN, NAN)
	, comfortZone(NAN, NAN)
	, throttleWindow(NAN, NAN)
	, steerWindow(NAN, NAN)

	, minSpeedLimit(NAN)
	, maxSpeedLimit(NAN)

	, curStateName(state)
{
	Plan & curPlan = *plan;

	this->trajDir = getTrajDir(plan, param);
	this->wantedCarDir = getWantedCarDir(plan, param, lookAheadDist);
	this->yawRateTrajectory = getYawRateTrajectory(plan, param, lookAheadDist);

	this->lateralError = getLateralError(plan, param, sqrDist, this->carPos);
	this->headingError = getHeadingError(plan, param, lookAheadDist, this->carDir);
}

ControllerData::~ControllerData()
{
}

ControllerData::ControllerData(ControllerData const  & other)
	: carPos(other.carPos)
	, carDir(other.carDir)
	, curSpeed(other.curSpeed)
	, yawRateMeasured(other.yawRateMeasured)
	, steeringAngleMeasured(other.steeringAngleMeasured)
	, throttlePositionMeasured(other.throttlePositionMeasured)

	, autonomousControl(other.autonomousControl)
	, gearPosition(other.gearPosition)

	, trajDir(other.trajDir)
	, wantedCarDir(other.wantedCarDir)
	, yawRateTrajectory(other.yawRateTrajectory)

	, lookAheadDist(other.lookAheadDist)
	, wantedSpeed(other.wantedSpeed)
	, speedCorrection(other.speedCorrection)
	, steerCorrection(other.steerCorrection)
	, gearCorrection(other.gearCorrection)

	, idleZone(other.idleZone)
	, comfortZone(other.comfortZone)
	, throttleWindow(other.throttleWindow)
	, steerWindow(other.steerWindow)

	, lateralError(other.lateralError)
	, headingError(other.headingError)

	, projectedFrontAxlePos(other.projectedFrontAxlePos)
	, projectedRearAxlePos(other.projectedRearAxlePos)
	, projectedFrontTipPos(other.projectedFrontTipPos)
	, projectedBackTipPos(other.projectedBackTipPos)

	, minSpeedLimit(other.minSpeedLimit)
	, maxSpeedLimit(other.maxSpeedLimit)

	, curStateName(other.curStateName)
{
}

ControllerData & ControllerData::operator=(ControllerData const & other)
{
	if (this == &other) {
		return *this;
	}

	carPos = other.carPos;
	carDir = other.carDir;
	curSpeed = other.curSpeed;
	yawRateMeasured = other.yawRateMeasured;
	steeringAngleMeasured = other.steeringAngleMeasured;
	throttlePositionMeasured = other.throttlePositionMeasured;

	autonomousControl = other.autonomousControl;
	gearPosition = other.gearPosition;

	trajDir = other.trajDir;
	wantedCarDir = other.wantedCarDir;
	yawRateTrajectory = other.yawRateTrajectory;

	lookAheadDist = other.lookAheadDist;
	wantedSpeed = other.wantedSpeed;
	speedCorrection = other.speedCorrection;
	steerCorrection = other.steerCorrection;
	gearCorrection = other.gearCorrection;

	idleZone = other.idleZone;
	comfortZone = other.comfortZone;
	throttleWindow = other.throttleWindow;
	steerWindow = other.steerWindow;

	lateralError = other.lateralError;
	headingError = other.headingError;

	projectedFrontAxlePos = other.projectedFrontAxlePos;
	projectedRearAxlePos = other.projectedRearAxlePos;
	projectedFrontTipPos = other.projectedFrontTipPos;
	projectedBackTipPos = other.projectedBackTipPos;

	minSpeedLimit = other.minSpeedLimit;
	maxSpeedLimit = other.maxSpeedLimit;

	curStateName = other.curStateName;

	return *this;
}



Vec3 ControllerData::getCarPos(TimedEgoState const & curEgoState)
{
	flt const frontShaftDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontShaftDistance")->value();

	return curEgoState.position() + frontShaftDistance * normalized(curEgoState.forwardDirection());
}

Vec3 ControllerData::getCarDir(TimedEgoState const & curEgoState)
{
	flt const frontShaftDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontShaftDistance")->value();

	return normalized(curEgoState.forwardDirection());
}

flt ControllerData::getCurSpeed(TimedEgoState const & curEgoState)
{
	flt curSpeed = std::abs(curEgoState.vehicleSpeed());

	return curSpeed;
}

Vec3 ControllerData::getTrajDir(Plan_ptr plan, flt closestParamOnTrajectory)
{
	Plan & curPlan = *plan;

	// direction of trajectory at nearest point to car
	Vec3  trajDir = normalized(curPlan.firstDerivative(closestParamOnTrajectory));

	return trajDir;
}

Vec2  ControllerData::getWantedCarDir(Plan_ptr plan, flt closestParamOnTrajectory, flt lookAheadDist)
{
	Plan & curPlan = *plan;

	// direction of trajectory at look ahead distance
	Vec2  wantedCarDir = normalized(head(curPlan.firstDerivative(rangeCut(closestParamOnTrajectory, closestParamOnTrajectory + lookAheadDist, curPlan.domain().second))));

	return wantedCarDir;
}

flt ControllerData::getLateralError(Plan_ptr plan, flt closestParamOnTrajectory, flt closestSqrDistToTrajectory, Vec3 pos)
{
	Plan & curPlan = *plan;

	Vec3  closestPosOnTrajectory = curPlan(closestParamOnTrajectory);
	Vec3  trajectoryDir = getTrajDir(plan, closestParamOnTrajectory);

	// cross track error in meter: distance to nearest position on trajectory
	// negative = left of traj, positive = right of traj
// 	flt const sign = cross2d(wantedCarDir, closestPosOnTrajectory - carPos);
	flt const sign = -::math::angle(trajectoryDir, pos - closestPosOnTrajectory);
// 	flt lateralError = copysign(sqrt(closestSqrDistToTrajectory), sign);
	flt lateralError = copysign(sin(::math::angle(trajectoryDir, pos - closestPosOnTrajectory)) * sqrt(closestSqrDistToTrajectory) , sign);

	return lateralError;
}

flt ControllerData::getHeadingError(Plan_ptr plan, flt closestParamOnTrajectory, flt lookAheadDist, Vec3 carDir)
{
	Vec2 wantedDir = getWantedCarDir(plan, closestParamOnTrajectory, lookAheadDist);

	// heading error in radian: difference between car orientation and orientation of the looked ahead point on trajectory
	// negative = left of traj, positive = right of traj
	flt headingError = ::math::angle(head(carDir), wantedDir);

	return headingError;
}

flt ControllerData::getYawRateTrajectory(Plan_ptr plan, flt closestParamOnTrajectory, flt lookAheadDist)
{
	flt const maxSteer = ::data::theVehicleData::instance().getPropertyType<flt>("maxSteer")->value();		//maximal angle of wheel in radian

	Plan & curPlan = *plan;

	// yaw rate for trajectory
	Vec3  d1 = curPlan.firstDerivative(rangeCut(closestParamOnTrajectory, closestParamOnTrajectory, curPlan.domain().second));
	Vec3  d2 = curPlan.firstDerivative(rangeCut(closestParamOnTrajectory, closestParamOnTrajectory + lookAheadDist, curPlan.domain().second));
	flt yawRateTrajectory = rangeCut(-maxSteer, ::math::angle(d1, d2), maxSteer);

	return yawRateTrajectory;
}

flt ControllerData::getYawRateMeasured(TimedEgoState const & curEgoState)
{
	// measured yaw rate
	flt yawRateMeasured = -curEgoState.angularRates()[2];

	return yawRateMeasured;
}

flt ControllerData::getSteeringAngleMeasured(TimedCarState const & curCarState)
{
	flt const maxSteer = ::data::theVehicleData::instance().getPropertyType<flt>("maxSteer")->value();		//maximal angle of wheel in radian

	// measured steering angle in radian
	//WARNING wheel direction is set in simulator and with console only. else it is 0
	// negative = turning left, positive = turning right
	flt steeringAngleMeasured = curCarState.wheelPosition * maxSteer;

	return steeringAngleMeasured;
}

flt ControllerData::getThrottlePositionMeasured(TimedCarState const & curCarState)
{
	flt throttlePositionMeasured = curCarState.gasPosition;
	return throttlePositionMeasured;
}

ostream & operator<< (ostream & os, const ControllerData & c)
{
	return os 	<< c.carPos << " "
	       << c.carDir << " "
	       << c.curSpeed << " "
	       << c.yawRateMeasured << " "
	       << c.steeringAngleMeasured << " "
	       << c.throttlePositionMeasured << " "

	       << c.trajDir << " "
	       << c.wantedCarDir << " "
	       << c.yawRateTrajectory << " "

	       << c.lookAheadDist << " "
	       << c.wantedSpeed << " "
	       << c.speedCorrection << " "
	       << c.steerCorrection << " "

	       << c.lateralError << " "
	       << c.headingError << " "

	       << c.minSpeedLimit << " "
	       << c.maxSpeedLimit << " "

	       << c.curStateName;
}

LOGPORT_NAMES(TimedControllerData,
              (LOGPORT_RTT1x_NAME("TimedData<modules::nav::controller::data::ControllerData>"))
              (LOGPORT_RTT1x_NAME("TimedData<ControllerData>"))
              (LOGPORT_RTT1x_NAME("TimedData< ::modules::nav::controller::data::ControllerData> >"))
              ("RTT::OutputPort<TimedData<modules::nav::controller::data::ControllerData> >")) //Shouldn't be logged, but is.
}
}
}
}
}
