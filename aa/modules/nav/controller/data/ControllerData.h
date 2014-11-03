#pragma once

#include <core/TimedData.h>
#include <math/AutoMath.h>
#include <iostream>
#include <fstream>

#include <aa/modules/nav/controller/Plan.h>
#include <modules/models/egostate/EgoState.h>
#include <modules/models/carstate/CarState.h>


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

class ControllerData;

typedef TimedData<ControllerData> TimedControllerData;

struct SpeedRamp {
	typedef ::math::flt flt;
	std::vector< std::pair<flt, flt> > ramp;		//array of < param, wantedSpeed >
	int successingState;
};

class ControllerData
{
public:
	typedef ::math::flt flt;
    typedef ::math::Vec2 Vec2;
    typedef ::math::Vec3 Vec3;

	ControllerData();
	ControllerData(TimedEgoState const & curEgoState, ::modules::models::carstate::TimedCarState const & curCarState, std::string state);
	ControllerData(TimedEgoState const & curEgoState, ::modules::models::carstate::TimedCarState const & curCarState, Plan_ptr plan, flt param, flt sqrDist,
				   flt lookAheadDist, flt wantedSpeed, flt speedCorrection, flt steerCorrection, flt gearCorrection, std::string state);
	~ControllerData();

	ControllerData(ControllerData const & other);

	ControllerData & operator=(ControllerData const & other);

	//data taken from given egostate
    Vec3 carPos;							//absolute position of car, taken from egostate
    Vec3 carDir;							//orientation of car, taken from egostate
	flt curSpeed;							//current speed of vehicle in m/s, taken from egostate
	flt yawRateMeasured;					//measured yaw rate of car, taken from egostate
	flt steeringAngleMeasured;				//measured steering angle in radian - WARNING wheel direction is set in simulator and with console only. else it is 0
	flt throttlePositionMeasured;

	//data taken from given carstate
	bool autonomousControl;					//indicate if controller has control over the car or manual
	int gearPosition;						//real gear position: -1=error, 0=between gears, 1=P, 2=R, 4=N, 8=D

	//data taken from given plan
    Vec3 trajDir;							//orientation of trajectory, taken from a given plan
	Vec2 wantedCarDir;						//orientation at look ahead distance of trajectory, taken from a given plan
	flt yawRateTrajectory;					//measured yaw rate of given plan at car position

	//additional inputs
	flt lookAheadDist;						//look ahead distance on trajectory in meter
	flt wantedSpeed;						//wanted speed proposal in m/s
	flt speedCorrection;					//speed command signal [-1.0,1.0]
	flt steerCorrection;					//steer command signal [-1.0,1.0]
	flt gearCorrection;						//gear command signal: -1=error, 0=between gears, 1=P, 2=R, 4=N, 8=D

	std::pair<flt, flt> idleZone;			//zone where input has no effect
	std::pair<flt, flt> comfortZone;		//zone where inputs are humanly comfortable
	std::pair<flt, flt> throttleWindow;		//window where throttle is allowed
	std::pair<flt, flt> steerWindow;		//window where steer is allowed


	flt lateralError;						//measured lateral error, computed in regard of given plan
	flt headingError;						//measured heading error, computed in regard of given plan

    Vec3 projectedFrontAxlePos;				//front axle projected point on trajectory
    Vec3 projectedRearAxlePos;				//rear axle projected point on trajectory
    Vec3 projectedFrontTipPos;				//front tip projected point on trajectory
    Vec3 projectedBackTipPos;				//back tip projected point on trajectory

	flt minSpeedLimit;						//minimum speed limit
	flt maxSpeedLimit;						//maximum speed limit

	std::string curStateName;				//current state of state machine



private:
    static Vec3 getCarPos(TimedEgoState const & curEgoState);
    static Vec3 getCarDir(TimedEgoState const & curEgoState);
	static flt getCurSpeed(TimedEgoState const & curEgoState);
	static flt getYawRateMeasured(TimedEgoState const & curEgoState);

	static flt getSteeringAngleMeasured(::modules::models::carstate::TimedCarState const & curCarState);
	static flt getThrottlePositionMeasured(::modules::models::carstate::TimedCarState const & curCarState);

    Vec3 getTrajDir(Plan_ptr plan, flt closestParamOnTrajectory);
	Vec2 getWantedCarDir(Plan_ptr plan, flt closestParamOnTrajectory, flt lookAheadDist);
	flt getYawRateTrajectory(Plan_ptr plan, flt closestParamOnTrajectory, flt lookAheadDist);

    flt getLateralError(Plan_ptr plan, flt closestParamOnTrajectory, flt closestSqrDistToTrajectory, math::Vec3 pos);
    flt getHeadingError(Plan_ptr plan, flt closestParamOnTrajectory, flt lookAheadDist, Vec3 carDir);

	friend std::ostream & operator<<(std::ostream & os, ControllerData const & c);
};

}
}
}
}
}

//namespace aa::modules::nav::controller::data


#include <modules/io/logger/LogPort.h>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/vector.hpp>


namespace boost
{
namespace serialization
{

template<class Archive>
void serialize(Archive & ar, aa::modules::nav::controller::data::TimedControllerData & o, const unsigned int version)
{
	if (version == 0) {
		// version from 11.12.2009
		ar & make_nvp("speedCorrection", o.speedCorrection);
		ar & make_nvp("steerCorrection", o.steerCorrection);
		ar & make_nvp("wantedSpeed", o.wantedSpeed);
		ar & make_nvp("curSpeed", o.curSpeed);

	}
	else if (version == 1) {
		// version from 10.01.2010
		ar & make_nvp("speedCorrection", o.speedCorrection);
		ar & make_nvp("wantedSpeed", o.wantedSpeed);
		ar & make_nvp("curSpeed", o.curSpeed);
		ar & make_nvp("steerCorrection", o.steerCorrection);
		ar & make_nvp("lateralError", o.lateralError);
		ar & make_nvp("wheelDirection", o.steeringAngleMeasured);
		ar & make_nvp("headingError", o.headingError);

	}
	else if (version == 2) {
		// version from 17.02.2010
		::math::Vec2f carPos;
		carPos[0] = o.carPos[0];
		carPos[1] = o.carPos[1];
		::math::Vec2f carDir;
		carDir[0] = o.carDir[0];
		carDir[1] = o.carDir[1];
		ar & make_nvp("carPos[0]", carPos[0]);
		ar & make_nvp("carPos[1]", carPos[1]);
		ar & make_nvp("carDir[0]", carDir[0]);
		ar & make_nvp("carDir[1]", carDir[1]);
		o.carPos[0] = carPos[0];
		o.carPos[1] = carPos[1];
		o.carDir[0] = carDir[0];
		o.carDir[1] = carDir[1];
		ar & make_nvp("curSpeed", o.curSpeed);
		ar & make_nvp("yawRateMeasured", o.yawRateMeasured);
		ar & make_nvp("steeringAngleMeasured", o.steeringAngleMeasured);

		::math::Vec2f trajDir;
		trajDir[0] = o.trajDir[0];
		trajDir[1] = o.trajDir[1];
		::math::Vec2f wantedCarDir;
		wantedCarDir[0] = o.wantedCarDir[0];
		wantedCarDir[1] = o.wantedCarDir[1];
		ar & make_nvp("trajDir[0]", trajDir[0]);
		ar & make_nvp("trajDir[1]", trajDir[1]);
		ar & make_nvp("wantedCarDir[0]", wantedCarDir[0]);
		ar & make_nvp("wantedCarDir[1]", wantedCarDir[1]);
		o.trajDir[0] = trajDir[0];
		o.trajDir[1] = trajDir[1];
		o.wantedCarDir[0] = wantedCarDir[0];
		o.wantedCarDir[1] = wantedCarDir[1];

		ar & make_nvp("yawRateTrajectory", o.yawRateTrajectory);

		ar & make_nvp("lookAheadDist", o.lookAheadDist);
		ar & make_nvp("wantedSpeed", o.wantedSpeed);
		ar & make_nvp("speedCorrection", o.speedCorrection);
		ar & make_nvp("steerCorrection", o.steerCorrection);

		ar & make_nvp("lateralError", o.lateralError);
		ar & make_nvp("headingError", o.headingError);

		ar & make_nvp("curStateName", o.curStateName);
	}
	else {// >= 3
		//current version
		::math::Vec2f carPos;
		carPos[0] = o.carPos[0];
		carPos[1] = o.carPos[1];
		::math::Vec2f carDir;
		carDir[0] = o.carDir[0];
		carDir[1] = o.carDir[1];
		ar & make_nvp("carPos[0]", carPos[0]);
		ar & make_nvp("carPos[1]", carPos[1]);
		ar & make_nvp("carDir[0]", carDir[0]);
		ar & make_nvp("carDir[1]", carDir[1]);
		o.carPos[0] = carPos[0];
		o.carPos[1] = carPos[1];
		o.carDir[0] = carDir[0];
		o.carDir[1] = carDir[1];

		ar & make_nvp("curSpeed", o.curSpeed);
		ar & make_nvp("yawRateMeasured", o.yawRateMeasured);
		ar & make_nvp("steeringAngleMeasured", o.steeringAngleMeasured);
		ar & make_nvp("throttlePositionMeasured", o.throttlePositionMeasured);

		::math::Vec2f trajDir;
		trajDir[0] = o.trajDir[0];
		trajDir[1] = o.trajDir[1];
		::math::Vec2f wantedCarDir;
		wantedCarDir[0] = o.wantedCarDir[0];
		wantedCarDir[1] = o.wantedCarDir[1];
		ar & make_nvp("trajDir[0]", trajDir[0]);
		ar & make_nvp("trajDir[1]", trajDir[1]);
		ar & make_nvp("wantedCarDir[0]", wantedCarDir[0]);
		ar & make_nvp("wantedCarDir[1]", wantedCarDir[1]);
		o.trajDir[0] = trajDir[0];
		o.trajDir[1] = trajDir[1];
		o.wantedCarDir[0] = wantedCarDir[0];
		o.wantedCarDir[1] = wantedCarDir[1];

		ar & make_nvp("yawRateTrajectory", o.yawRateTrajectory);

		ar & make_nvp("lookAheadDist", o.lookAheadDist);
		ar & make_nvp("wantedSpeed", o.wantedSpeed);
		ar & make_nvp("speedCorrection", o.speedCorrection);
		ar & make_nvp("steerCorrection", o.steerCorrection);

		if (version >= 5) {
			ar & make_nvp("autonomousControl", o.autonomousControl);
			ar & make_nvp("gearPosition", o.gearPosition);
			ar & make_nvp("gearCorrection", o.gearCorrection);
		}

		if (version >= 4) {
			ar & make_nvp("idleZone.first", o.idleZone.first);
			ar & make_nvp("idleZone.second", o.idleZone.second);
			ar & make_nvp("comfortZone.first", o.comfortZone.first);
			ar & make_nvp("comfortZone.second", o.comfortZone.second);
			ar & make_nvp("throttleWindow.first", o.throttleWindow.first);
			ar & make_nvp("throttleWindow.second", o.throttleWindow.second);
			ar & make_nvp("steerWindow.first", o.steerWindow.first);
			ar & make_nvp("steerWindow.second", o.steerWindow.second);
		}

		ar & make_nvp("lateralError", o.lateralError);
		ar & make_nvp("headingError", o.headingError);

		ar & make_nvp("curStateName", o.curStateName);

		if (version >= 6) {
			ar & make_nvp("projectedFrontAxlePos[0]", o.projectedFrontAxlePos[0]);
			ar & make_nvp("projectedFrontAxlePos[1]", o.projectedFrontAxlePos[1]);
			ar & make_nvp("projectedRearAxlePos[0]", o.projectedRearAxlePos[0]);
			ar & make_nvp("projectedRearAxlePos[1]", o.projectedRearAxlePos[1]);
			ar & make_nvp("projectedFrontTipPos[0]", o.projectedFrontTipPos[0]);
			ar & make_nvp("projectedFrontTipPos[1]", o.projectedFrontTipPos[1]);
			ar & make_nvp("projectedBackTipPos[0]", o.projectedBackTipPos[0]);
			ar & make_nvp("projectedBackTipPos[1]", o.projectedBackTipPos[1]);
		}
	}
}

} // namespace serialization
} // namespace boost

//version
BOOST_CLASS_VERSION(aa::modules::nav::controller::data::TimedControllerData, 6)
