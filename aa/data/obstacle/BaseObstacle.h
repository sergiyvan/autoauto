#pragma once
#include "util/Location.h"
#include <aa/data/obstacle/util/LinearFilter.h>
#include <aa/data/obstacle/util/Contour.h>
#include <core/TimeStamp.h>
#include <aa/data/obstacle/Classification.h>
//20.10 2013, add by shuiying, necessary for statelattice
#include <data/geometry/BoundingBox3D.h>

namespace aa
{
namespace data
{
namespace obstacle
{

/**
*  \author Michael Schnürmacher
*	Base class that contains the minimum information the behaviour needs of an obstacle.
**/
class BaseObstacle
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;
	typedef util::Contour Contour;
	typedef util::LinearFilter LinearFilter;


	BaseObstacle();

	bool operator< (BaseObstacle const & other) const
	{
		return id() < other.id();
	}

	bool operator== (BaseObstacle const & other) const
	{
		return id() == other.id();
	}


	enum MovementState {
		// Means that the obstacle has not moved from some time
		MOVEMENT_STATE_STATIC	= 0,
		// Means that the obstacle is currently moving or did move not long ago
		MOVEMENT_STATE_DYNAMIC	= 1,
		// Means that the obstacle was not observed long enough to judge its movement state
		MOVEMENT_STATE_UNKNOWN	= 2
	};


	// Getter
	TimeStamp const & current() const
	{
		return mCurrent;
	}
	uint id() const
	{
		return mId;
	}
	Contour const & contour() const
	{
		return mContour;
	}
	Classification::ClassificationType classification() const
	{
		return mClassification;
	}
	util::Location location() const
	{
		return mLocation;
	}
	bool onLane() const
	{
		return (mLocation == util::SPARSELY_ON_LANE) || (mLocation == util::LARGELY_ON_LANE);
	}
	Vec2 const & boundingCircleCentre() const
	{
		return mBoundingCircleCentre;
	}
	flt boundingCircleRadius() const
	{
		return mBoundingCircleRadius;
	}
	MovementState movementState() const
	{
		return mMovementState;
	}
	flt movementStateHoldingTime() const
	{
		return mMovementStateHoldingTime;
	}
	flt maxPredictionTime() const
	{
		return mMaxPredictionTime;
	}
	//20.10 2013, add by shuiying, necessary for statelattice
	::data::geometry::BoundingBox3D const & boundingBox() const
	{
		return mBoundingBox;
	}

	// Setter
	void setId(uint id)
	{
		mId = id;
	}
	void setContour(Contour const & contour)
	{
		mContour = contour;
	}
	void setClassification(Classification::ClassificationType classification)
	{
		mClassification = classification;
	}
	void setLocation(util::Location location)
	{
		mLocation = location;
	}
	void setBoundingCircleCentre(Vec2 const & centre)
	{
		mBoundingCircleCentre = centre;
	}
	void setBoundingCircleRadius(flt radius)
	{
		mBoundingCircleRadius = radius;
	}
	void setMovementState(MovementState state)
	{
		mMovementState = state;
	}
	void setMovementStateHoldingTime(flt time)
	{
		mMovementStateHoldingTime = time;
	}
	void setMaxPredictionTime(flt t)
	{
		mMaxPredictionTime = t;
	}
	void setVelocity(Vec3 const & vel)
	{
		mVelocity = vel;
	}
	void setAcceleration(Vec3 const & acc)
	{
		mAcceleration = acc;
	}
	void setCurrent(TimeStamp const & time)
	{
		mCurrent = time;
	}
	//20.10 2013, add by shuiying, necessary for statelattice
	void setBoundingBox(::data::geometry::BoundingBox3D const & box)
	{
		mBoundingBox = box;
	}


	/**
	  * Returns a string represantation of the movement state.
	  */
	std::string movementStateString() const;


	/**
	  *
	  */
	Vec3 const & velocity() const;


	/**
	  *
	  */
	Vec3 const & acceleration() const;


	/**
	  * Predicts this base obstacle for "dT" seconds. Returns false, if
	  * the "dT" is too small or too big.
	  */
	Vec3 translation(flt dT) const;
	bool predictAndUpdateWithoutMeasurement(flt dT, BaseObstacle & prediction) const;
	bool predictAndUpdateWithoutMeasurement(TimeStamp const & t, BaseObstacle & prediction) const
	{
		flt dT = RTT::os::TimeService::ticks2nsecs(t - mCurrent) / 1000000000.0;
		return predictAndUpdateWithoutMeasurement(dT, prediction);
	}
	bool predictAndUpdateWithoutMeasurementPlusBoundingBox(flt dT, BaseObstacle & prediction) const;


protected:

	bool predict(flt dT, BaseObstacle & prediction) const;
	bool predict(TimeStamp const & t, BaseObstacle & prediction) const
	{
		flt dT = RTT::os::TimeService::ticks2nsecs(t - mCurrent) / 1000000000.0;
		return predict(dT, prediction);
	}

	// Unique identifier
	uint mId;
	// Time stamp at which the obstacle is currently valid.
	TimeStamp mCurrent;
	// Contour points
	util::Contour mContour;
	// Classification
	Classification::ClassificationType mClassification;
	// Location relative to the RNDF
	util::Location mLocation;
	// All points of the obstacle are guaranteed to be in the following bounding circle
	// with centre...
	Vec2 mBoundingCircleCentre;
	// ...and radius
	flt mBoundingCircleRadius;
	// Current movement state
	MovementState mMovementState;
	// Time the obstacle stayed in the current state until now [s]
	flt mMovementStateHoldingTime;
	// Maximum time, that the obstacle can be reliably predicted [s]
	// (reflects the quality of the dynamic paramters computed in the tracking step).
	flt mMaxPredictionTime;
	// Velocity
	Vec3 mVelocity;
	// Acceleration
	Vec3 mAcceleration;
	//20.10 2013, add by shuiying, necessary for statelattice
	::data::geometry::BoundingBox3D mBoundingBox;




private:


};

}
}
}
