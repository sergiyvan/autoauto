#include "Obstacle.h"

#include "util/IbeoData.h"
#include "util/ScannerSystem.h"
#include "util/Contour.h"

#include <rtt/Logger.hpp>
#include <math/AutoMath.h>
#include <math/IbeoMath.h>

using namespace aa::data::obstacle;
using namespace math;
using namespace aa::data::obstacle::util;


Obstacle::Obstacle()
	: BaseObstacle()
	, mShape(SHAPE_UNKNOWN)
	, mBoundingBox()
	, mNumScanPoints(0)
	, mHideType(HIDDEN_NONE)
	, mCenterOfGravity(0.0, 0.0, 0.0)
	, mCurrentSensorType(SENSOR_TYPE_UNKNOWN)
	, mPreviousSensorType(SENSOR_TYPE_UNKNOWN)
	, mPotentialPhantom(false)
	, mUpdatedCounter(0)
{

}


Obstacle::Obstacle(Obstacle const & obstacle)
	: BaseObstacle(obstacle)
	, mShape(obstacle.mShape)
	, mBoundingBox(obstacle.mBoundingBox)
	, mNumScanPoints(obstacle.mNumScanPoints)
	, mHideType(obstacle.mHideType)
	, mCenterOfGravity(obstacle.mCenterOfGravity)
	, mCurrentSensorType(obstacle.mCurrentSensorType)
	, mPreviousSensorType(obstacle.mPreviousSensorType)
	, mPotentialPhantom(obstacle.mPotentialPhantom)
	, mUpdatedCounter(obstacle.mUpdatedCounter)
//	, mDistance(obstacle.mDistance)
//	, mDtToEgoState(obstacle.mDtToEgoState)
{
}

Obstacle & Obstacle::operator=(Obstacle const & obstacle)
{
	if (&obstacle == this) {
		return *this;
	}

	BaseObstacle::operator=(obstacle);

	mShape = obstacle.mShape;
	mBoundingBox = obstacle.mBoundingBox;
	mNumScanPoints = obstacle.mNumScanPoints;
	mHideType = obstacle.mHideType;
	mCenterOfGravity = obstacle.mCenterOfGravity;
	mCurrentSensorType = obstacle.mCurrentSensorType;
	mPreviousSensorType = obstacle.mPreviousSensorType;
	mPotentialPhantom = obstacle.mPotentialPhantom;
	mUpdatedCounter = obstacle.mUpdatedCounter;

	return *this;
}

Obstacle::Obstacle(
    ::data::geometry::BoundingBox3D const & boundingBox,
    TimeStamp const & created,
    SensorType sensortype,
    Vec3 const & velocity)
	: BaseObstacle()
	, mShape(SHAPE_UNKNOWN)
	, mBoundingBox(boundingBox)
	, mNumScanPoints(1)
	, mHideType(HIDDEN_NONE)
	, mCenterOfGravity(0.0, 0.0, 0.0)
	, mCurrentSensorType(sensortype)
	, mPreviousSensorType(sensortype)
	, mPotentialPhantom(false)
	, mUpdatedCounter(0)
{
	mCurrent = created;
	mVelocity = velocity;
	computeBoundingCircle();
}

Obstacle::Obstacle(
    ::data::geometry::BoundingBox3D const & boundingBox,
    unsigned int id,
    Classification::ClassificationType classification,
    unsigned int numScanPoints,
    TimeStamp const & created,
    SensorType sensortype,
    Vec3 const & velocity)
	: BaseObstacle()
	, mShape(SHAPE_UNKNOWN)
	, mBoundingBox(boundingBox)
	, mNumScanPoints(numScanPoints)
	, mHideType(HIDDEN_NONE)
	, mCenterOfGravity(0.0, 0.0, 0.0)
	, mCurrentSensorType(sensortype)
	, mPreviousSensorType(sensortype)
	, mPotentialPhantom(false)
	, mUpdatedCounter(0)
{
	mCurrent = created;
	mVelocity = velocity;
	mId = id;
	mClassification = classification;
	computeBoundingCircle();
}


Obstacle::~Obstacle()
{

}


void Obstacle::computeHideType(flt & coveragePercentage, Obstacle const & other, ScannerSystem const & scanner)
{
	Vec2 scannerPos = math::head(scanner.getGlobalPosition());

	boost::array<unsigned int, 3> indicesOther = other.boundingBox().visibleVertexIndices(scannerPos);
	// Gather information of the other obstacle
	Vec2 otherLeft         = other.boundingBox().vertex2D(indicesOther[0]);
	Vec2 otherRight        = other.boundingBox().vertex2D(indicesOther[2]);
	Vec2 otherLeftmostVec  = otherLeft - scannerPos;
	Vec2 otherRightmostVec = otherRight - scannerPos;

	boost::array<unsigned int, 3> indicesThis = mBoundingBox.visibleVertexIndices(scannerPos);
	// Gather information of this obstacle
	Vec2 thisLeft			= mBoundingBox.vertex2D(indicesThis[0]);
	Vec2 thisRight			= mBoundingBox.vertex2D(indicesThis[2]);
	Vec2 thisLeftmostVec	= thisLeft - scannerPos;
	Vec2 thisRightmostVec	= thisRight - scannerPos;
	Vec2 leftToRight		= thisRight - thisLeft;
	Vec2 thisLeftToOtherRight	= otherRight  - thisLeft;
	Vec2 thisLeftToOtherLeft	= otherLeft  - thisLeft;

	bool otherNearerThanThis =
	    math::det<flt>(leftToRight, thisLeftToOtherRight) < 0.0 &&
	    math::det<flt>(leftToRight, thisLeftToOtherLeft) < 0.0;

	bool leftOfOtherInsideOfThis =
	    math::det<flt>(thisLeftmostVec, otherLeftmostVec) < 0.0 &&
	    math::det<flt>(otherLeftmostVec, thisRightmostVec) < 0.0;

	bool rightOfOtherInsideOfThis =
	    math::det<flt>(thisLeftmostVec, otherRightmostVec) < 0.0 &&
	    math::det<flt>(otherRightmostVec, thisRightmostVec) < 0.0;

	bool leftOfThisInsideOfOther =
	    math::det<flt>(otherLeftmostVec, thisLeftmostVec) < 0.0 &&
	    math::det<flt>(thisLeftmostVec, otherRightmostVec) < 0.0;

	bool rightOfThisInsideOfOther =
	    math::det<flt>(otherLeftmostVec, thisRightmostVec) < 0.0 &&
	    math::det<flt>(thisRightmostVec, otherRightmostVec) < 0.0;

	bool hidden =
	    (leftOfOtherInsideOfThis || rightOfOtherInsideOfThis || leftOfThisInsideOfOther || rightOfThisInsideOfOther) &&
	    otherNearerThanThis;

	HideType hideType = HIDDEN_NONE;

	if (hidden) {
		flt baseAngle = acos(
		                    dot_product(
		                        normalized(thisLeft - scannerPos),
		                        normalized(thisRight - scannerPos))
		                );

		if (leftOfThisInsideOfOther && rightOfThisInsideOfOther) {
			coveragePercentage = 1.0f;
			hideType = HIDDEN_COMPLETE;
		}
		else if (rightOfOtherInsideOfThis && leftOfThisInsideOfOther) {
			flt angle = acos(
			                dot_product(
			                    normalized(thisLeft - scannerPos),
			                    normalized(otherRight - scannerPos)));
			coveragePercentage = std::abs(angle / baseAngle);

			if (coveragePercentage != 0.0) {
				hideType = HIDDEN_PARTIAL_LEFT;
			}
			else {
				hideType = HIDDEN_NONE;
			}
		}
		else if (leftOfOtherInsideOfThis && rightOfThisInsideOfOther) {
			flt angle = acos(
			                dot_product(
			                    normalized(otherLeft - scannerPos),
			                    normalized(thisRight - scannerPos)));
			coveragePercentage = std::abs(angle / baseAngle);

			if (coveragePercentage != 0.0) {
				hideType = HIDDEN_PARTIAL_RIGHT;
			}
			else {
				hideType = HIDDEN_NONE;
			}
		}
		else if (leftOfOtherInsideOfThis && rightOfOtherInsideOfThis) {
			flt angle = acos(
			                dot_product(
			                    normalized(otherLeft - scannerPos),
			                    normalized(otherRight - scannerPos)));
			coveragePercentage = std::abs(angle / baseAngle);

			if (coveragePercentage != 0.0) {
				hideType = HIDDEN_PARTIAL_MIDDLE;
			}
			else {
				hideType = HIDDEN_NONE;
			}
		}
	}
	else {
		coveragePercentage = 0.0f;
		hideType = HIDDEN_NONE;
	}

	mergeHideType(hideType);
}




bool Obstacle::predict(flt dT, Obstacle & prediction) const
{
	bool p = BaseObstacle::predict(dT, prediction);

	Vec3 trans = translation(dT);
	prediction.mShape = mShape;
	prediction.mBoundingBox = mBoundingBox;
	prediction.mBoundingBox.translate(trans);
	prediction.mNumScanPoints = mNumScanPoints;
	prediction.mHideType = mHideType;
	prediction.mCenterOfGravity = mCenterOfGravity;
	prediction.mCenterOfGravity += trans;
	prediction.mCurrentSensorType = mCurrentSensorType;
	prediction.mPreviousSensorType = mPreviousSensorType;
	prediction.mPotentialPhantom = mPotentialPhantom;
	prediction.mUpdatedCounter = mUpdatedCounter;

	return p;
}


bool Obstacle::predictAndUpdateWithoutMeasurement(flt dT, Obstacle & prediction) const
{
	bool p = predict(dT, prediction);

	return p;
}





/** **************************************************************************************
							Information appending
*****************************************************************************************/

void Obstacle::appendVelocity(std::string & info) const
{
	math::ibeomath::appendF(info, velocity().norm());
	info += "m/s";
}


void Obstacle::appendShape(std::string & info) const
{
	if	(mShape == SHAPE_CONVEX) {
		info += "Convex";
	}
	else if (mShape == SHAPE_CONCAVE) {
		info += "Concave";
	}
	else if (mShape == SHAPE_STRAIGHT) {
		info += "Straight";
	}
	else if (mShape == SHAPE_CLOUDED) {
		info += "Clouded";
	}
	else {
		info += "Unknown";
	}
}

void Obstacle::appendId(std::string & info) const
{
	math::ibeomath::append(info, mId);
}

void Obstacle::appendGroundArea(std::string & info) const
{
	info += "Area:";
	math::ibeomath::append(info, (int)mBoundingBox.groundArea());
}

void Obstacle::appendLocation(std::string & info) const
{
	if (onLane()) {
		info += "OnLane";
	}
	else if (mLocation == OFF_LANE) {
		info += "OffLane";
	}
	else if (mLocation == NEAR_LANE) {
		info += "NearLane";
	}
	else if (mLocation == IN_ZONE) {
		info += "InZone";
	}
	else {
		info += "LocationUnknown";
	}
}
