#pragma once
#include "AbstractOcclusionMap.h"
#include <math/Types.h>
#include <boost/array.hpp>
#include "FieldOfView.h"
#include "SensorModel.h"


namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

class SingleScanner
{
public:
	typedef ::math::flt flt;
	typedef ::math::Affine3 Affine3;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	enum {
		NUM_RESOLUTION_AREAS = 8
	};


	// Constructors
	SingleScanner();
	SingleScanner(
		Affine3 const & localToGlobalMatrix,
		Vec2 const & globalOrientation2D,
		Vec3 const & positionOffset,
		Vec3 const & angleRPYOffset,
		flt minSpacingToSensor,
		unsigned int deviceId,
		flt startAngle,
		flt endAngle,
		SensorModel model);



	/**
	*	\return True, if the given point lies very close to one end of the sight field
	*   TODO: Revise
	*/
// 	bool isCloseToSightFieldBoundary(Vec2 const& point) const;

	/**
	*	\return True, if the given point lies inside of the sight field
	*/
	bool isInsideOfSightField2D(
		Vec2 const & point,
		flt angleOffset = 0.0) const;

	/**
	*	Returns the scan angle of the given point in the scanners coordinate system
	*	[-pi,pi]
	*/
	flt getScanAngle(
		Vec3 const & point) const;


	/**
	*	Returns the resolution of the scanner in the area that contains the 'p'.
	*/
//	flt getResolution(
//		Vec3 const & p) const;


	/**
	*	Returns true if the scanner completely sees the line from v1 to v2.
	*/
	bool completelySees(Vec2 const & v1, Vec2 const & v2) const;


	// Setter
	void setResolution(
		unsigned int i,
		Vec2 const & resolution) {
		mResolution[i] = resolution;
	}
	void setOcclusionMap(SharedOcclusionMap om) {
		mOcclusionMap = om;
	}

	// Getter
	/// Offsets
	Vec3 const & positionOffset() const {
		return mPositionOffset;
	}
	Vec3 const & angleRPYawOffset() const {
		return mAngleRPYOffset;
	}
	/// Angular range
	flt endAngle() const {
		return mEndAngle;
	}
	flt startAngle() const {
		return mStartAngle;
	}
	/// Min spacing
	flt minSpacingToSensorSquared() const {
		return mMinSpacingToSensorSquared;
	}
	/// Global position
	Vec3 const & globalPosition() const {
		return mGlobalPosition;
	}
	Vec2 globalPosition2D() const {
		return Vec2(mGlobalPosition[0], mGlobalPosition[1]);
	}
	/// Global orientation
	Vec2 const & globalOrientation2D() const {
		return mGlobalOrientation2D;
	}
	/// Local to global matrix
	Affine3 const & localToGlobalMatrix() const {
		return mLocalToGlobalMatrix;
	}
	/// Scanner id
	unsigned int deviceId() const {
		return mDeviceId;
	}
	math::Mat2x2 const & yawAngleRotationMatrix() const {
		return mYawRotationMatrix;
	}
	SensorModel model() const {
		return mModel;
	}
//	FieldOfView & fieldOfViewRef() {
//		return mFieldOfView;
//	}
//	FieldOfView const & fieldOfView() {
//		return mFieldOfView;
//	}
	SharedOcclusionMap occlusionMap() {
		return mOcclusionMap;
	}
	SharedConstOcclusionMap occlusionMap() const {
		return mOcclusionMap;
	}

protected:

	// Translation offset to the IMU of the car (translation to the car coordinate system)
	Vec3 mPositionOffset;
	// Rotation offset to the IMU of the car (translation to the car coordinate system)
	Vec3 mAngleRPYOffset;
	// Sight field angles in the coordinate system of the scanner [rad] "right"
	flt mEndAngle;
	// Sight field angles in the coordinate system of the scanner [rad] "left"
	flt mStartAngle;
	// Resolution scan fields of this scanner
	boost::array<Vec2, NUM_RESOLUTION_AREAS> mResolution;
	// Global position of the scanner at the time when the scan data was received
	Vec3 mGlobalPosition;
	// Global direction of the scanner at the time when the scan data was received
	Vec2 mGlobalOrientation2D;
	// Matrix that computes a local scan point of this scanner in a point in the global
	// coordinate system of the scanner
	Affine3 mLocalToGlobalMatrix;
	// A sensor does not only have a maximal range but also a minimal. Points that emerge
	// beneath this bound are invalid
	flt mMinSpacingToSensorSquared;
	// Id of the sensor
	unsigned int mDeviceId;

	math::Mat2x2 mYawRotationMatrix;

	SensorModel mModel;

//	FieldOfView mFieldOfView;

	SharedOcclusionMap mOcclusionMap;

private:

	/**
	*	Computes the transformation matrix needed to transform a local point to global
	*	coordinates
	*/
	void computeLocalToGlobalMatrix(Affine3 const & localToGlobalMatrix);

	/**
	*	Computes the global position of the scanner  based on the global position of
	*	the scanner system it belongs to.
	*/
	void computeGlobalPosition();

	/**
	*	Computes the global direction of the scanner  based on the global position of
	*	the scanner system it belongs to.
	*/
	void computeGlobalOrientation(Vec2 const & globalOrientation2D);

	/**
	*	Computes the transformation matrix that contains the position and angle offset
	*	to the scanner system
	*/
	Affine3 getScannerMatrix() const;

	/**
	  *
	  */
	void computeYawAngleRotationMatrix(Vec2 const & systemOrientation);

};

}
}
}
}
