#pragma once

#include "BaseObstacle.h"
#include "util/SensorType.h"

#include <data/geometry/BoundingBox3D.h>

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{
class ScannerSystem;
}

/**
* \author Sebastian Hempel, Michael Schnürmacher
* Representation of obstacles. Contains all the information needed to construct realistic
* and non-intersecting obstacles. It contains a bounding box.
**/
class Obstacle
	: public BaseObstacle
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	enum Shape {
		SHAPE_UNKNOWN	= 0,
		SHAPE_CONVEX 	= 1,
		SHAPE_CONCAVE	= 2,
		SHAPE_STRAIGHT	= 3,
		SHAPE_CLOUDED	= 4
	};

	// Binary Type
	enum HideType {
		HIDDEN_NONE				= 0,
		HIDDEN_PARTIAL_LEFT		= 1,
		HIDDEN_PARTIAL_RIGHT 	= 2,
		HIDDEN_PARTIAL_MIDDLE 	= 4,
		HIDDEN_COMPLETE			= HIDDEN_PARTIAL_LEFT | HIDDEN_PARTIAL_RIGHT | HIDDEN_PARTIAL_MIDDLE
	};


	Obstacle();


	/**
	* \param[in] obstacle
	*/
	Obstacle(Obstacle const & obstacle);

	/**
	* \param[in] boundingbox	contains all the geometric information
	* \param[in] id				id of this obstacle
	* \param[in] classification	obstacle classification
	**/
	Obstacle(
	    ::data::geometry::BoundingBox3D const & boundingBox,
	    unsigned int id,
	    Classification::ClassificationType classification,
	    unsigned int numScanPoints,
	    TimeStamp const & created,
	    aa::data::obstacle::util::SensorType sensortype,
	    Vec3 const & velocity = Vec3(0.0, 0.0, 0.0));

	explicit Obstacle(
	    ::data::geometry::BoundingBox3D const & boundingBox,
	    TimeStamp const & created,
	    aa::data::obstacle::util::SensorType sensortype,
	    Vec3 const & velocity = Vec3(0.0, 0.0, 0.0));

	~Obstacle();


	Obstacle & operator=(Obstacle const & obstacle);

	// Getter
	::data::geometry::BoundingBox3D const & boundingBox() const
	{
		return mBoundingBox;
	}
	::data::geometry::BoundingBox3D & boundingBox()
	{
		return mBoundingBox;
	}
	util::Contour const & contour() const
	{
		return mContour;
	}
	util::Contour & contour()
	{
		return mContour;
	}
	unsigned int numScanPoints() const
	{
		return mNumScanPoints;
	}
	Shape shape() const
	{
		return mShape;
	}
	unsigned int currentSensorType() const
	{
		return mCurrentSensorType;
	}
	unsigned int previousSensorType() const
	{
		return mPreviousSensorType;
	}
	Vec3 const & centerOfGravity() const
	{
		return mCenterOfGravity;
	}
	bool potentialPhantom() const
	{
		return mPotentialPhantom;
	}
	unsigned int updatedCounter() const
	{
		return mUpdatedCounter;
	}

	// Setter
	void setBoundingBox(::data::geometry::BoundingBox3D const & box)
	{
		mBoundingBox = box;
	}
	void setNumScanPoints(unsigned int numScanPoints)
	{
		mNumScanPoints = numScanPoints;
	}
	void setShape(Shape shape)
	{
		mShape = shape;
	}
	void setHideType(unsigned int hidetype)
	{
		mHideType = hidetype;
	}
	void setCenterOfGravity(Vec3 const & center)
	{
		mCenterOfGravity = center;
	}
	void setUpdated(TimeStamp const & t)
	{
		mCurrent = t;
		mUpdatedCounter++;
	}
	void setPotentialPhantom(bool potentialPhantom)
	{
		mPotentialPhantom = potentialPhantom;
	}
	void setUpdatedCounter(uint c)
	{
		mUpdatedCounter = c;
	}

	// Hide type
	void mergeHideType(HideType hidetype)
	{
		mHideType = mHideType | hidetype;
	}
	bool isPartlyHidden() const
	{
		return mHideType != HIDDEN_NONE;
	}
	bool isCompletelyHidden() const
	{
		return mHideType == HIDDEN_COMPLETE;
	}

	// Sensor Type
	void setSensorType(unsigned int sensortype)
	{
		mCurrentSensorType = sensortype;
		mPreviousSensorType = sensortype;
	}
	void mergeSensorType(unsigned int sensortype)
	{
		mPreviousSensorType = sensortype;
		mCurrentSensorType = mCurrentSensorType | sensortype;
	}
	bool sensorTypePureIbeo() const
	{
		return mCurrentSensorType == util::SENSOR_TYPE_IBEO;
	}
	bool sensorTypeIbeo() const
	{
		return mCurrentSensorType & util::SENSOR_TYPE_IBEO;
	}
	bool sensorTypePureRadar() const
	{
		return mCurrentSensorType == util::SENSOR_TYPE_RADAR;
	}
	bool sensorTypePureVelodyne() const
	{
		return mCurrentSensorType == util::SENSOR_TYPE_VELODYNE;
	}
	bool sensorTypeVelodyne() const
	{
		return mCurrentSensorType & util::SENSOR_TYPE_VELODYNE;
	}

	// Bounding Circle
	void computeBoundingCircle()
	{
		mBoundingCircleCentre = mBoundingBox.center2D();
		mBoundingCircleRadius = mBoundingBox.radius2D();
	}


//	void setFilter(LinearFilter const & filter) {
//		mFilter = filter;
//	}
//	LinearFilter const & filter() const {
//		return mFilter;
//	}


	/**
	*
	*/
	void computeHideType(
	    flt & coverag,
	    Obstacle const & other,
	    aa::data::obstacle::util::ScannerSystem const & scanner);


	/**
	  *
	  */
	bool predict(flt dT, Obstacle & prediction) const;
	bool predictAndUpdateWithoutMeasurement(flt dT, Obstacle & prediction) const;
	bool predict(TimeStamp const & t, Obstacle & prediction) const
	{
		flt dT = RTT::os::TimeService::ticks2nsecs(t - mCurrent) / 1000000000.0;
		return predict(dT, prediction);
	}
	bool predictAndUpdateWithoutMeasurement(TimeStamp const & t, Obstacle & prediction) const
	{
		flt dT = RTT::os::TimeService::ticks2nsecs(t - mCurrent) / 1000000000.0;
		return predictAndUpdateWithoutMeasurement(dT, prediction);
	}


	// Append information
	void appendShape(
	    std::string & info) const;
	void appendId(
	    std::string & info) const;
	void appendGroundArea(std::string & info) const;
	void appendLocation(std::string & info) const;
	void appendVelocity(std::string & info) const;

protected:

	// Shape
	Shape mShape;
	// Its bounding box
	::data::geometry::BoundingBox3D mBoundingBox;
	// Number of scan points this obstacle was computed from
	unsigned int mNumScanPoints;
	// The way the obstacle is hidden by other obstacles
	unsigned int mHideType;
	// Center of gravity (the points center of gravity or center of the boundin box)
	Vec3 mCenterOfGravity;
	// Sensor origin; might have many when fused
	unsigned int mCurrentSensorType;

	unsigned int mPreviousSensorType;
	// Indicates that the obstacle might be a phantom
	bool mPotentialPhantom;


	unsigned int mUpdatedCounter;

private:


};


}
}
}