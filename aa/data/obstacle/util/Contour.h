#pragma once
#include <math/Types.h>
#include <vector>
#include <util/aligned_allocator.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace data
{
namespace geometry
{
class BoundingBox3D;
}
}


namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{


class ScannerSystem;

/**
*	\author Michael Schnürmacher
*	A contour of an obstacle is an angular sorted list of points on the surface of the
*	obstacle such that the complete obstacle lies behind the contour from the point of
*	view of a reference point. Sometimes only the bounding box of an obstacle is known.
*	In that case the contour points lie on the bounding box.
*	A contour can be the convex hull the obstacle, which allows a minimal angle of 0°
*	at the joint of three adjacent vertices. A convex hull often is not a good approximation
*	to the contour of an obstacle (e.g. concave obstacles). So we allow smaller minimal
*	angles.
*/
class Contour
    : public std::vector< ::math::Vec2>
{

public:

	friend class boost::serialization::access;

	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	// Maximal distance allowed between two adjacent points on the contour
	static const flt MAX_CONTOUR_POINT_DISTANCE;
	static const flt MAX_CONTOUR_POINT_DISTANCE_SQR;

	enum {
		MAX_NUM_CONTOUR_POINTS = 200 //80, // 24
	};

	Contour();



	// Getter
	flt minInclinationAngle() const
	{
		return mMinInclinationAngle;
	}
	flt height() const
	{
		return mHeight;
	}
	Vec2 const & referencePoint() const
	{
		return mReferencePoint;
	}
	// Setter
    void setReferencePoint(Vec2 const & point)
	{
		mReferencePoint = point;
	}


	/**modules
	  *
	  */
    void push_back(Vec2 const & point)
	{
        std::vector<Vec2>::push_back(point);
	}


    /**
    * Computes the contour based on the given bounding box. It uses the scanner to eliminate
    * hidden points.
    */
    void compute(::data::geometry::BoundingBox3D const & boundingBox, ScannerSystem const & scanner);


    /**
    *	Computes a contour from a list of points that is a convex hull and the position
    *	that mean the point of view.
    */
    void compute(std::vector< Vec2> const & convexhull, Vec2 const & referencePoint);


    /**
    *	Computes a contour from a list of points and does *not* eliminate hidden points
    */
    void compute(std::vector< Vec2> const & convexhull);


	/**
	*	\param[in] sorting Angular-Sorted list of points
	*	\param[in]
	*	Computes a contour from a angular-sorted list of points and a minimal inclination
	*	angle.
	*/
    void compute(std::vector<Vec2> const & sorting, Vec2 const & referencePoint,
	             flt minInclinationAngle);


	void computeFromUnsortedList(
	    std::vector<Vec2> list,
        Vec2 const & refPoint,
	    flt minInclinationAngle);


	/**
	  *
	  */
	void buildFromSortedList(
        std::vector<Vec2> const & sorting,
        Vec2 const & referencePoint,
	    flt sampleDistance);


	Contour merge(Contour const & c1, Contour const & c2);


	/**
	*	\return true if both given points lie on the same side of the contour
	*/
    bool liesBehindContour(Vec2 const & point) const;


	/**
	*	\return true if "point" lies inside of the cone defined by the contour and
	*	the reference point.
	*/
    bool liesInsideContourCone(Vec2 const & point) const;


	/**
	*	\return true if the contour occludes "other".
	*/
	bool occludes(Contour const & other) const;


	/**
	  * Translates the all contour point by "translation".
	  */
    void translate(Vec2 const & translation);


protected:

	// Point of view of the one who observes the obstacle (sensor/car)
	Vec2 mReferencePoint;
	// Minimal allowed angle of the junction between three incident point on the contour
	flt mMinInclinationAngle;
	// Average height of the points used to create the contour
	flt mHeight;

private:

	/**
	*	Samles the given contour, such that the minimal distance between two point on
	*	the contour is kept.
	*/
	void sample();


	/**
	*	\return true, if "point" lies inside of the cone defined by contour points "cpLeft" and
	*	"cpRight" and the reference point of the contour.
	*/
	bool liesInsideContourCone(Vec2 const & point, Vec2 const & cpLeft, Vec2 const & cpRight) const;


	void validate() const;

	void addSamplePoints(Vec2 p1, Vec2 p2, flt sampleDist);


	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object< std::vector< math::Vec2> >(*this);
		ar & mReferencePoint;
		ar & mMinInclinationAngle;
		ar & mHeight;
	}
};

}
}
}
}
