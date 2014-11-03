#include "BaseObject.h"

using namespace aa::data::obstacle::util;
using namespace aa::data::obstacle;
using namespace math;

BaseObject::BaseObject()
	: mCreated()
	, mPosition(Vec3::Zero())
	, mVelocity(Vec3::Zero())
	, mAcceleration(Vec3::Zero())
	, mPositionCovariance(Mat3x3::Zero())
	, mVelocityCovariance(Mat3x3::Zero())
	, mContour()
	, mBoundingBox()
	, mSensorType(SENSOR_TYPE_UNKNOWN)
	, mSensorModel(SENSOR_MODEL_UNKNOWN)
	, mClassification()
{

}

BaseObject::BaseObject(
    ::data::geometry::BoundingBox3D const & box,
    Vec3 const & position,
    Vec3 const & velocity,
    Vec3 const & acceleration,
    Mat3x3 const & positionCovarianzMatrix,
    Mat3x3 const & velocityCovarianzMatrix,
    Contour const & contour,
    SensorType const & sensortype,
    TimeStamp t)
	: mCreated(t)
	, mPosition(position)
	, mVelocity(velocity)
	, mAcceleration(acceleration)
	, mPositionCovariance(positionCovarianzMatrix)
	, mVelocityCovariance(velocityCovarianzMatrix)
	, mContour(contour)
	, mBoundingBox(box)
	, mSensorType(sensortype)
	, mSensorModel(SENSOR_MODEL_UNKNOWN)
	, mClassification()
{

}

BaseObject::BaseObject(
    ::data::geometry::BoundingBox3D const & box,
    Vec3 const & position,
    Vec3 const & velocity,
    Vec3 const & acceleration,
    Mat3x3 const & positionCovarianzMatrix,
    Mat3x3 const & velocityCovarianzMatrix,
    Contour const & contour,
    SensorType const & sensortype,
    SensorModel const & sensorModel,
    TimeStamp t)
	: mCreated(t)
	, mPosition(position)
	, mVelocity(velocity)
	, mAcceleration(acceleration)
	, mPositionCovariance(positionCovarianzMatrix)
	, mVelocityCovariance(velocityCovarianzMatrix)
	, mContour(contour)
	, mBoundingBox(box)
	, mSensorType(sensortype)
	, mSensorModel(sensorModel)
	, mClassification()
{

}


BaseObject::~BaseObject()
{

}
