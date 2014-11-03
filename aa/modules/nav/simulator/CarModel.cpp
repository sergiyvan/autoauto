#include "CarModel.h"
#include <rtt/Property.hpp>

using namespace aa::modules::nav::simulator;
using namespace modules::models::carstate;
using namespace ::math;

flt asFloat(RTT::PropertyBag const & prop, std::string const & name)
{
	RTT::Property<flt> p(prop.getProperty(name));
	return p.ready() ? p.get() : std::numeric_limits<flt>::signaling_NaN();
}

CarModel::CarModel(RTT::PropertyBag const & prop)
	: currSpeed(0.0)
	, currSteer(0.0)
	, pos(Vec3::Zero())
	, orientation(0.0f, 0.0f, 0.0f, 1.0f)
	, movementDirection(1.0f, 0.0f, 0.0f)
	, mGear(CarState::GEAR_PARK)
	, mFrontShaftDistance(asFloat(prop, "frontShaftDistance"))
	, mRearShaftDistance(asFloat(prop, "rearShaftDistance"))
	, mShaftDistance(asFloat(prop, "shaftDistance"))
	, mMaxSpeedChange(asFloat(prop, "maxSpeedChange"))
	, mMaxBreakChange(asFloat(prop, "maxBrakeChange"))
	, mMaxSteer(asFloat(prop, "maxSteer"))
	, mMaxSteerChange(asFloat(prop, "maxSteerChange"))
	, mMaxSteerAngleAcceleration(asFloat(prop, "maxSteerAngleAcceleration"))
	, mMaxSpeed(asFloat(prop, "maxSpeed"))
	, mMaxReverseSpeed(asFloat(prop, "maxReverseSpeed"))
	, mTireStiffness(asFloat(prop, "tireStiffness"))
	, mTireRadius(asFloat(prop, "tireRadius"))
	, mCarName(prop.getPropertyType<std::string>("carName")->rvalue())
	, mYawRate(0.0)
	, mCurvature(0.0)
	, mWheelDir(1.0, 0.0)
	, mCurrentSteerSpeed(0.0)
	, mIndexOfCommandRequested(0)
	, mSteerIntegral(0)
	, mSteerIntegralLimit(0.257) //0.557 ...1000 to 530 = x to 0.59.;  and 100 to 50 Hz --> divided by 2
	, mCurrentSteerError(0)
	, mOldSteerError(0)
	, mSteerMomentum(0)
{
	assert(prop.getProperty("carName") != 0 && prop.getProperty("carName")->ready());
}
