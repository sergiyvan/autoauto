#pragma once

/**
*  Base class that contains the minimum information for objects abstracted from sensors
**/

#include "util/Contour.h"
#include "core/TimeStamp.h"
#include "util/SensorType.h"
#include "util/SensorModel.h"
#include "Classification.h"

#include <data/geometry/BoundingBox3D.h>

namespace aa
{
namespace data
{
namespace obstacle
{

class BaseObject
{

public:
	typedef math::Vec3 Vec3;
	typedef math::Mat3x3 Mat3x3;


	BaseObject();
	BaseObject(
	    ::data::geometry::BoundingBox3D const &,
	    Vec3 const & position,
	    Vec3 const & velocity,
	    Vec3 const & acceleration,
	    Mat3x3 const & positionCovarianceMatrix,
	    Mat3x3 const & velocityCovarianceMatrix,
	    aa::data::obstacle::util::Contour const &,
	    aa::data::obstacle::util::SensorType const &,
	    TimeStamp t
	);
	BaseObject(
	    ::data::geometry::BoundingBox3D const &,
	    Vec3 const & position,
	    Vec3 const & velocity,
	    Vec3 const & acceleration,
	    Mat3x3 const & positionCovarianceMatrix,
	    Mat3x3 const & velocityCovarianceMatrix,
	    aa::data::obstacle::util::Contour const &,
	    aa::data::obstacle::util::SensorType const &,
	    aa::data::obstacle::util::SensorModel const &,
	    TimeStamp t
	);
	~BaseObject();

	//getter
	::data::geometry::BoundingBox3D const & boundingBox() const
	{
		return mBoundingBox;
	}
	Vec3 const & position() const
	{
		return mPosition;
	}
	Vec3 const & velocity() const
	{
		return mVelocity;
	}
	Vec3 const & acceleration() const
	{
		return mAcceleration;
	}
	TimeStamp created() const
	{
		return mCreated;
	}
	Mat3x3 const & positionCovarianceMatrix() const
	{
		return mPositionCovariance;
	}
	Mat3x3 const & velocityCovarianceMatrix() const
	{
		return mVelocityCovariance;
	}
	aa::data::obstacle::util::Contour const  & contour() const
	{
		return mContour;
	}
	aa::data::obstacle::util::SensorType const & sensorType() const
	{
		return mSensorType;
	}
	aa::data::obstacle::util::SensorModel const & sensorModel() const
	{
		return mSensorModel;
	}

	//setter
	/**
	  * set the bounding box, this should represent the maximal dimensions of the measurement
	  **/
	void setBoundingBox(::data::geometry::BoundingBox3D & box)
	{
		mBoundingBox = box;
	}
	/**
	  * set the position of the object, this does not have to be the center of gravity
	  **/
	void setPosition(math::Vec3 position)
	{
		mPosition = position;
	}
	/**
	  * set the global velocity of the object
	  **/
	void setVelocity(math::Vec3 velocity)
	{
		mVelocity = velocity;
	}
	/**
	  * set the timestamp of the measurement
	  **/
	void setCreated(TimeStamp t)
	{
		mCreated = t;
	}
	/**
	  * set the covarianz matrix for position, this has not to be the sensorcovarianzmatrix
	  * but should represent the variance for each dimension in x,y,z cartesian coordinates
	  **/
	void setPositionCovarianceMatrix(math::Mat3x3 matrix)
	{
		mPositionCovariance = matrix;
	}
	/**
	  * set the covarianz matrix for velocity, this has not to be the sensorcovarianzmatrix
	  * but should represent the variance for each dimension in x,y,z cartesian coordinates
	  **/
	void setVelocityCovarianceMatrix(math::Mat3x3 matrix)
	{
		mVelocityCovariance = matrix;
	}
	/**
	  * set the contour of the object, this can be only one point or complete point cloud of the contour
	  **/
	void setContour(aa::data::obstacle::util::Contour & contour)
	{
		mContour = contour;
	}
	/**
	  * names the Sensortype of the measurement like radar, lidar, camera
	  **/
	void setSensorType(aa::data::obstacle::util::SensorType & sensortype)
	{
		mSensorType = sensortype;
	}
	/**
	  * names the Sensormodel which specify a single sensor in a sensorsystem, like fronttrw, kinect...
	  **/
	void setSensorModel(aa::data::obstacle::util::SensorModel & sensormodel)
	{
		mSensorModel = sensormodel;
	}



protected:

	//
	TimeStamp mCreated;
	//
	Vec3 mPosition;
	//
	Vec3 mVelocity;
	//
	Vec3 mAcceleration;
	//
	Mat3x3 mPositionCovariance;
	//
	Mat3x3 mVelocityCovariance;
	//
	Classification mClassification;
	//
	::data::geometry::BoundingBox3D mBoundingBox;
	//
	aa::data::obstacle::util::Contour mContour;
	//
	aa::data::obstacle::util::SensorType mSensorType;
	//
	aa::data::obstacle::util::SensorModel mSensorModel;


private:

};

}
}
}
