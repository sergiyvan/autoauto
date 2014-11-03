#include "SingleScanner.h"

#include <math/AutoMath.h>
#include <math/IbeoMath.h>

using namespace math;
using namespace aa::data::obstacle::util;

SingleScanner::SingleScanner()
	: mPositionOffset(0.0, 0.0, 0.0)
	, mAngleRPYOffset(0.0, 0.0, 0.0)
	, mMinSpacingToSensorSquared(0.0)
	, mGlobalPosition(0.0, 0.0, 0.0)
	, mGlobalOrientation2D(0.0, 0.0)
	, mDeviceId(0)
	, mStartAngle(0.0)
	, mEndAngle(0.0)
	, mModel(SENSOR_MODEL_UNKNOWN)
{
}

SingleScanner::SingleScanner(Affine3 const & localToGlobalMatrix,
							 Vec2 const & globalOrientation2D,
							 Vec3 const & positionOffset,
							 Vec3 const & angleRPYOffset,
							 flt minSpacingToSensor,
							 unsigned int deviceId,
							 flt startAngle,
							 flt endAngle,
							 SensorModel model)
	: mPositionOffset(positionOffset)
	, mAngleRPYOffset(angleRPYOffset)
	, mMinSpacingToSensorSquared(minSpacingToSensor * minSpacingToSensor)
	, mDeviceId(deviceId)
	, mStartAngle(startAngle)
	, mEndAngle(endAngle)
	, mModel(model)
{
	computeLocalToGlobalMatrix(localToGlobalMatrix);
	computeGlobalPosition();
	computeGlobalOrientation(globalOrientation2D);
	computeYawAngleRotationMatrix(globalOrientation2D);
}

bool SingleScanner::isInsideOfSightField2D(Vec2 const & point, flt angleOffset) const
{
	if ((mStartAngle - mEndAngle) > 350.0 * math::D2R) {
		return true;
	}

	Vec2 dirScannerPoint = point - globalPosition2D();
	flt relativeAngle = math::angle(mGlobalOrientation2D, dirScannerPoint);
	return (relativeAngle > (mEndAngle + epsilon)) && (relativeAngle < (mStartAngle - epsilon));
}

flt SingleScanner::getScanAngle(Vec3 const & point) const
{
	Vec2 centerDirection2D = math::head(point - mGlobalPosition);
	return math::angle(mGlobalOrientation2D, centerDirection2D);
}



//flt SingleScanner::getResolution(Vec3 const & p) const
//{
//	flt angle = getScanAngle(p);
//	unsigned int i = 0;

//	while (i < NUM_RESOLUTION_AREAS && angle <= mResolution[i][0]) {
//		i++;
//	}

//	return mResolution[i-1][1];
//}

bool SingleScanner::completelySees(Vec2 const & v1, Vec2 const & v2) const
{
	Vec2 position = globalPosition2D();

	if ((v1 - position).squaredNorm() > 400.0 || (v2 - position).squaredNorm() > 400.0) {
		return false;
	}

	flt epsilon = 5.0 * math::D2R;

	if (!isInsideOfSightField2D(v1, epsilon) || !isInsideOfSightField2D(v2, epsilon)) {
		return false;
	}

	Vec2 left;
	Vec2 right;

	if (math::ibeomath::isLeftTo(v1 - position, v2 - position)) {
		left = v1;
		right = v2;
	}
	else {
		left = v2;
		right = v1;
	}

	Vec2 forward	= (((right + left) * flt(0.5)) - position);
	Vec2 forwardT	= Vec2(forward[1], -forward[0]);
	Vec2 vecLR		= (right - left);

	if (std::abs(math::angle(forwardT, vecLR)) > flt(50.0 * math::D2R)) {
		return false;
	}

	return true;
}


/** **************************************************************************************
*
*	Private
*
*****************************************************************************************/

void SingleScanner::computeLocalToGlobalMatrix(Affine3 const & localToGlobalMatrix)
{
	mLocalToGlobalMatrix = localToGlobalMatrix * getScannerMatrix();
}

void SingleScanner::computeGlobalPosition()
{
	mGlobalPosition = ibeomath::localToGlobal(Vec3(0.0, 0.0, 0.0), mLocalToGlobalMatrix);
}

void SingleScanner::computeGlobalOrientation(Vec2 const & systemOrientation)
{
	flt yaw = mAngleRPYOffset[2];
	mGlobalOrientation2D = normalized(Vec2(
										  systemOrientation[0] * cos(yaw) - systemOrientation[1] * sin(yaw),
										  systemOrientation[0] * sin(yaw) + systemOrientation[1] * cos(yaw)));
}

void SingleScanner::computeYawAngleRotationMatrix(Vec2 const & systemOrientation)
{
	flt yaw = mAngleRPYOffset[2] + std::atan2(systemOrientation[1], systemOrientation[0]);

	mYawRotationMatrix(0, 0) =  cos(yaw);
	mYawRotationMatrix(0, 1) = -sin(yaw);
	mYawRotationMatrix(1, 0) =  sin(yaw);
	mYawRotationMatrix(1, 1) =  cos(yaw);
}

Affine3 SingleScanner::getScannerMatrix() const
{
#if defined(USE_EIGEN)
	return ::math::fromPositionRpy(mPositionOffset, mAngleRPYOffset);
#else
	math::Affine3 matScanner;
	matScanner.setIdentity();
	matScanner.set_rotation_roll_pitch_yaw(mAngleRPYOffset[0], mAngleRPYOffset[1], mAngleRPYOffset[2]);
	matScanner.set_translation(mPositionOffset[1], mPositionOffset[0], mPositionOffset[2]);
	return matScanner;
#endif

}
