#pragma once
#include "SingleScanner.h"
#include "RegionOfInterest.h"
#include "SensorType.h"

#include <core/TimedData.h>
#include <math/AutoMath.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <osgTerrain/Layer>


namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

class ScannerSystem
	: public std::vector<SingleScanner>
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Affine3 Affine3;

	enum {
		MAX_NUM_SINGLE_SENSORS = 8
	};

	/// Constructors
	ScannerSystem();
	ScannerSystem(
		Affine3 const & localToGlobal,
		SensorType type,
		flt scanFrequency = 0,
		SharedRoi roi = SharedRoi());


	/// Getter
	flt scanFrequency() const {
		return mScanFrequency;
	}
	flt distance(Vec2 const & point) const {
		return sqrt(distanceSqr(point));
	}
	flt distanceSqr(Vec2 const & point) const {
		return ::math::ssd(::math::head(mGlobalPosition), point);
	}
	Vec3 const & getGlobalPosition() const {
		return mGlobalPosition;
	}
	Vec2 getGlobalPosition2D() const {
		return Vec2(mGlobalPosition[0], mGlobalPosition[1]);
	}
	Vec2 getGlobalOrientation2D() const {
		return math::head(mGlobalOrientation);
	}
	Affine3 const & getLocalToGlobalMatrix() const {
		return mLocalToGlobalMatrix;
	}
	SensorType type() const {
		return mType;
	}
	flt getScanAngle(Vec3 const & point) const {
		return math::angle(math::head(mGlobalOrientation), math::head(point - mGlobalPosition));
	}
	flt minDistOfLeftGroundPoints() const {
		return mMinDistOfLeftGroundPoints;
	}
	flt minDistOfRightGroundPoints() const {
		return mMinDistOfRightGroundPoints;
	}
	flt groundPointsAngle() const {
		return mGroundPointsAngle;
	}
	SharedRoi roi() const {
		return mRoi;
	}
	SharedRoi roi() {
		return mRoi;
	}
//	FieldOfView const & fieldOfView() const {
//		return mFieldOfView;
//	}


	/// Setter
	void setRoi(SharedRoi roi) {
		mRoi = roi;
	}


	/**
	*	\return True, if the given point lies inside of the sight field
	*/
	bool isInsideOfSightField2D(
		Vec2 const & point) const;


	/**
	  *	Computes the blind regions of the scanner system around the car.
	  */
//	void computeBlindRegion();


	/**
	  * Computes field of view from the single scanner field of views.
	  */
//	void computeFieldOfView(unsigned int n, vgl_clip_type clipType);


	/**
	  *
	  */
	void setMinDistOfGroundPoints(flt left, flt right, flt angle) {
		mMinDistOfLeftGroundPoints = left;
		mMinDistOfRightGroundPoints = right;
		mGroundPointsAngle = angle;
	}

	flt occludedProbability(Vec3 const &) const;


protected:

	// Scan frequency of the scanner
	flt mScanFrequency;
	// Global position of the scanner at the time when the scan data was received
	Vec3 mGlobalPosition;
	// Global orientation of the scanner at the time when the scan data was received
	Vec3 mGlobalOrientation;
	// Matrix that transforms a scan point in the coordinate system of this scanner in
	// the global coordinate system of the IMU
	Affine3 mLocalToGlobalMatrix;
	// Contour of the car to the given state of the scanner system
//	Vec3 mCarContour[4];

	// Only relevant for alasca
	flt mMinDistOfLeftGroundPoints;
	flt mMinDistOfRightGroundPoints;
	flt mGroundPointsAngle;

	// Correction to the ego state cleaned from the movement of the own car
	Vec3 mEgoPositionCorrection;

	SharedRoi mRoi;

	SensorType mType;

//	std::vector<Vec3> mBlindRegion;

//	FieldOfView mFieldOfView;

private:

	/**
	* 	Computes the contour of the car to the state of the scanner system
	*/
//	void computeCarContour();
};

typedef boost::shared_ptr<ScannerSystem> ScannerSystemPtr;
}
}
}
}
