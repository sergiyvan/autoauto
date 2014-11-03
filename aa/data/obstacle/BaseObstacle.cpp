#include "BaseObstacle.h"
#include <boost/concept_check.hpp>
#include <math/AutoMath.h>
#include <core/VirtualClock.h>


using namespace aa::data::obstacle;
using namespace aa::data::obstacle::util;

BaseObstacle::BaseObstacle()
	: mId(0)
	, mCurrent()
	, mClassification(Classification::UNCLASSIFIED)
	, mLocation(LARGELY_ON_LANE)
	, mBoundingCircleCentre(0.0, 0.0)
	, mBoundingCircleRadius(0.0)
	, mMovementState(MOVEMENT_STATE_UNKNOWN)
	, mMovementStateHoldingTime(0)
	, mMaxPredictionTime(0.0)
	, mVelocity(0.0, 0.0, 0.0)
	, mAcceleration(0.0, 0.0, 0.0)
{

}


std::string BaseObstacle::movementStateString() const
{
	switch (mMovementState) {
	case MOVEMENT_STATE_STATIC:
		return "S";
	case MOVEMENT_STATE_DYNAMIC:
		return "D";
	case MOVEMENT_STATE_UNKNOWN:
		return "U";
	default:
		return "";
	}
}


BaseObstacle::Vec3 const & BaseObstacle::velocity() const
{
	return mVelocity;
}


BaseObstacle::Vec3 const & BaseObstacle::acceleration() const
{
	return mAcceleration;
}


BaseObstacle::Vec3 BaseObstacle::translation(flt dT) const
{
	return mVelocity * dT;
}


bool BaseObstacle::predict(flt dT, BaseObstacle & prediction) const
{
	flt dTAllowed = std::min(dT, mMaxPredictionTime);
	Vec2 trans = math::head(translation(dTAllowed));
	prediction = *this;
	prediction.mContour.translate(trans);
	prediction.mBoundingCircleCentre += trans;
	prediction.mMaxPredictionTime -= dTAllowed;
	VirtualClock::advance(prediction.mCurrent, RTT::os::TimeService::nsecs2ticks(dTAllowed * 1000000000));

	return true;
}


bool BaseObstacle::predictAndUpdateWithoutMeasurement(flt dT, BaseObstacle & prediction) const
{
	bool valid = predict(dT, prediction);
	return valid;
}

bool BaseObstacle::predictAndUpdateWithoutMeasurementPlusBoundingBox(flt dT, BaseObstacle & prediction) const
{
    flt dTAllowed = std::min(dT, mMaxPredictionTime);
    Vec3 trans3= translation(dTAllowed);
    Vec2 trans = math::head(trans3);
    prediction = *this;
    prediction.mContour.translate(trans);
    prediction.mBoundingCircleCentre += trans;
    prediction.mMaxPredictionTime -= dTAllowed;
    prediction.mBoundingBox.translate(trans3);
//    prediction.mBoundingBox.primaryComponent() = prediction.mBoundingBox.vertex(3)-prediction.mBoundingBox.vertex(0);
//    prediction.mBoundingBox.secondaryComponent() = prediction.mBoundingBox.vertex(1)-prediction.mBoundingBox.vertex(0);
    VirtualClock::advance(prediction.mCurrent, RTT::os::TimeService::nsecs2ticks(dTAllowed * 1000000000));

    return true;

}
