#pragma once

#include <stdio.h>
#include <math/LaneSpline.h>
#include <boost/shared_ptr.hpp>


namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{


class ObstacleRegionOfInterest;


/**
 *  shared region of interest
 */
typedef boost::shared_ptr<ObstacleRegionOfInterest> SharedRoi;


/**
    Region of interest for obstacle detection and tracking.
 */
class ObstacleRegionOfInterest
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	ObstacleRegionOfInterest();

	/**
	 *   \brief Updates the current state. Call this method BEFORE "hasPoint" is
	 *   called!
	 *
	 *   \brief position IMU position
	 *   \brief direction car's forward direction
	 *   \brief speed speed of the car
	 */
	virtual void update(const Vec3 & position,
						const Vec3 & direction,
						flt speed);


	/**
	 *  \brief Creates a copy.
	 */
	virtual SharedRoi copy() const = 0;


	/**
	 *  \brief Checks wheather a given point lies within the ROI.
	 *         IPLEMENT THIS METHOD IN DERIVING CLASSES!
	 *  \param point point to check
	 */
	virtual bool hasPoint(Vec3 const & point) const = 0;


	/**
	 *  \brief Checks wheather a given point lies within the ROI,
	 *		   excluding the given border.
	 *         IPLEMENT THIS METHOD IN DERIVING CLASSES!
	 *  \param point point to check
	 *	\param borderwidth points that lie on the border do not lie
	 *		   inside of the region.
	 */
	virtual bool hasPoint(Vec3 const & point, flt borderwidth) const = 0;


	bool hasPointX(Vec3 point) const {
		return hasPoint(point);
	}


	/**
	 *  \brief Calculates the sourrounding rectangle edge lengths.
	 *         IMPLEMENT THIS METHOD IN DERIVING CLASSES!
	 */
	virtual Vec2 getBoundingRectangle() const = 0;


	/**
	 *   \brief Calculates the center of the sourrounding rectangle (in global
	 *          coordinates).
	 *          IMPLEMENT THIS METHOD IN DERIVING CLASSES IF POSITION DIFFERS
	 *          FROM "mPosition".
	 */
	virtual Vec2 getBoundingPosition() const {
		return Vec2(mPosition[0],
					mPosition[1]);
	}

	/**
	 *  \brief Access to position.
	 */
	Vec3 getPosition() const {
		return mPosition;
	}


	/**
	 *  \brief Access to direction.
	 */
	Vec3 getDirection() const {
		return mDirection;
	}


	/**
	 *  \brief Access to orthogonal component.
	 */
	Vec3 getOrthogonal() const {
		return mOrthogonal;
	}

protected:
	//state information
	Vec3 mPosition,
		 mDirection,
		 mOrthogonal;

	flt mSpeed;


	/**
	 *  \brief Projects a point onto direction and orthogonal.
	 *  \param[in] x point to project
	 *  \param position origin of the coordinate system
	 */
	Vec2 project(Vec3 x, Vec3 const & position) const {
		x -= position;

		return Vec2(x[0] * mDirection[0] + x[1] * mDirection[1],
					x[0] * mOrthogonal[0] + x[1] * mOrthogonal[1]);
	}


	/**
	 *   \brief Projects a point onto direction and orthogonal.
	 *   \param[in] x point to project
	 */
	Vec2 project(Vec3 const & x) const {
		return project(x, mPosition);
	}


	/**
	 *  \brief Converts a local position into a global one.
	 *  \param[in] a forward direction
	 *  \param[in] b orthogonal direction
	 */
	Vec3 toGlobal(flt a, flt b) const {
		return toGlobal(mPosition, a, b);
	}


	/**
	 *  \brief Converts a local position into a global one.
	 *  \param[in] pos position
	 *  \param[in] a forward direction
	 *  \param[in] b orthogonal direction
	 */
	Vec3 toGlobal(Vec3 const & pos, flt a, flt b) const {
		return pos + (mDirection * a) + (mOrthogonal * b);
	}


	/**
	 *  \brief Calculates orhtogonal direction by forward direction.
	 */
	void orthoByForward() {
		mOrthogonal = Vec3(mDirection[1],
						   -mDirection[0],
						   0.0f);
	}
};


/* ----------------------- some ROI inmplementations ------------------------ */


/**
 *  \brief Simple circle.
 */
class CircleROI : public ObstacleRegionOfInterest
{
public:
	/**
	 * \param[in] radius radius of the circle
	 */
	CircleROI(flt radius);

	virtual bool hasPoint(Vec3 const & point) const;
	virtual bool hasPoint(Vec3 const & point, flt borderwidth) const;
	virtual Vec2 getBoundingRectangle() const;

	virtual SharedRoi copy() const;

protected:
	CircleROI(const flt radius, const float squaredRadius);

	//radius of the region
	flt mRadius,
		mSquaredRadius;
};


/**
 * \brief Ellipse ;-)
 */
class EllipseROI : public ObstacleRegionOfInterest
{
public:
	/**
	 *  \param[in] length length of the ellipse (longitudinal to car direction)
	 *  \param[in] width width
	 */
	EllipseROI(flt length, flt width);

	virtual bool hasPoint(Vec3 const & point) const;
	virtual bool hasPoint(Vec3 const & point, flt borderwidth) const;
	virtual Vec2 getBoundingRectangle() const;
	virtual SharedRoi copy() const;


protected:
	EllipseROI(const flt length,
			   const flt width,
			   const flt sqrdHalfLength,
			   const flt sqrdHalfWidth);

	//diameters of the ellipse
	flt mLength,
		mWidth,
		mSqrdHalfLength,
		mSqrdHalfWidth;
};


class ParabolaROI : public ObstacleRegionOfInterest
{
public:
	/**
	 *  \brief f(x) = a0 + a1*x + a2*x^2
	 *  \param[in] a0 coefficient
	 *  \param[in] a1 coefficient
	 *  \param[in] a2 coefficient
	 *  \param[in] size size of the parabola; f(x) <= size
	 */
	ParabolaROI(flt a0, flt a1, flt a2, flt size);

	virtual bool hasPoint(Vec3 const & point) const;
	virtual bool hasPoint(Vec3 const & point, flt borderwidth) const;
	virtual Vec2 getBoundingRectangle() const;
	virtual Vec2 getBoundingPosition() const;
	virtual SharedRoi copy() const;

protected:
	ParabolaROI(const flt a0, const flt a1, const flt a2, const flt size,
				const flt minX, const flt maxX);

	//coefficients and look ahead
	flt mA0, mA1, mA2, mSize;

	//minimal and maximal x
	flt mMinX, mMaxX;

	//evaluates the function at point x
	flt eval(flt x) const;

	//calculate min and max xs
	void calcXs();
};


class OffsetROI : public ObstacleRegionOfInterest
{
public:
	/**
	 *  \param[in] other region to shift
	 *  \param[in] offset local offset (forward, side, ignored)
	 */
	OffsetROI(SharedRoi other,
			  Vec3 & offset);

	/**
	 *  \param[in] other region to shift
	 *  \param[in] offset local offset (forward, side, ignored)
	 */
	OffsetROI(SharedRoi other,
			  Vec3 offset);

	virtual bool hasPoint(Vec3 const & point) const;
	virtual bool hasPoint(Vec3 const & point, flt borderwidth) const;
	virtual Vec2 getBoundingRectangle() const;
	virtual Vec2 getBoundingPosition() const;
	virtual void update(const Vec3 & position,
						const Vec3 & direction,
						flt speed);
	virtual SharedRoi copy() const;


protected:
	SharedRoi mOther;
	Vec3 mOffset;

private:
	/**
	 *  \param[in] other region to shift
	 *  \param[in] offset local offset (forward, side, ignored)
	 */
	void init(SharedRoi other,
			  Vec3 & offset);
};


/**
 *  \brief Inverts a region of interest.
 *         NOTE: The bounding box will not be resized!
 */
class NotROI : public ObstacleRegionOfInterest
{
public:
	/**
	 *  \param[in] region region of interest
	 */
	NotROI(SharedRoi region);
	~NotROI();

	virtual bool hasPoint(Vec3 const & point) const;
	virtual bool hasPoint(Vec3 const & point, flt borderwidth) const;
	virtual Vec2 getBoundingRectangle() const;
	virtual Vec2 getBoundingPosition() const;
	virtual void update(const Vec3 & position,
						const Vec3 & direction,
						flt speed);
	virtual SharedRoi copy() const;

private:
	/**
	 * inverted region
	 */
	SharedRoi mRegion;
};


/**
 *  \brief Binary operator for Regions of Interest.
 */
class BinaryROI : public ObstacleRegionOfInterest
{
public:
	/**
	 *  \param[in] first first ROI
	 *  \param[in] second second ROI
	 */
	BinaryROI(SharedRoi fist,
			  SharedRoi second);
	~BinaryROI();

	virtual Vec2 getBoundingRectangle() const;
	virtual Vec2 getBoundingPosition() const;
	virtual void update(const Vec3 & position,
						const Vec3 & direction,
						flt speed);

protected:
	BinaryROI(const SharedRoi first,
			  const SharedRoi second,
			  const Vec2 & rectDimensions,
			  const Vec2 & rectPosition);

	SharedRoi mFirst;
	SharedRoi mSecond;

	//rectangle dimensions and position
	Vec2 mRectDimensions,
		 mRectPosition;
};


/**
 *   \brief Creates a new binary operator.
 *          Sample usage for union:
 *              MKBINOP(Union, a || b)
 *   \param[in] name Name of the operator; class name will be nameROI; e.g.:
 *                  name = Union => class name = UnionROI
 *   \param[in] op operator; variables a in b a given implicit to check if a
 *                 point lies in the first region (a) or the the second one (b).
 *                 op must be a boolean expresssion; e.g. for union:
 *                 op = a || b
 */
#define MKBINOP(name, op) class name ## ROI : public BinaryROI {        \
	public:                                                             \
		name ## ROI(SharedRoi first,                                    \
					SharedRoi second)                                   \
			: BinaryROI(first, second) {}                           \
		\
		virtual bool hasPoint(Vec3 const& point) const {                \
			bool a = mFirst->hasPoint(point),                       \
					 b = mSecond->hasPoint(point);                      \
			\
			return (op);                                            \
		}												\
		virtual bool hasPoint(Vec3 const& point, flt borderwidth) const {                \
			bool a = mFirst->hasPoint(point, borderwidth),                       \
					 b = mSecond->hasPoint(point, borderwidth);                      \
			\
			return (op);                                            \
		}\
		virtual SharedRoi copy() const {                                \
			return SharedRoi(new name ## ROI(mFirst, mSecond));         \
		}                                                               \
	};


MKBINOP(Union, a || b)
MKBINOP(Intersection, a && b)
MKBINOP(Xor, a != b)
MKBINOP(Minus, a && !b)


/**
 *  \brief Axis aligned region of interest.
 */
class RectangleROI : public ObstacleRegionOfInterest
{
public:
	/**
	    Parameters:
	            length - length of the rectangle (along car direction)
	            width  - width
	**/
	RectangleROI(flt length, flt width);

	virtual bool hasPoint(Vec3 const & point) const;
	virtual bool hasPoint(Vec3 const & point, flt borderwidth) const;
	virtual Vec2 getBoundingRectangle() const;
	virtual SharedRoi copy() const;

private:
	//sizes of the rectangle
	flt mLength,
		mWidth;
};


/**
	Speed dependant parabola.
**/
class SpeedParabolaROI : public ParabolaROI
{
public:
	/**
	    f(x) = a0 + a1*x + a2*x^2

	    Parameters:
	    a0, a1, a2	-	coefficients
	**/
	SpeedParabolaROI(flt a0, flt a1, flt a2)
		: ParabolaROI(a0, a1, a2, 0.0f) { }

	virtual void update(const Vec3 & position,
						const Vec3 & direction,
						flt speed);
	virtual SharedRoi copy() const;

protected:
	SpeedParabolaROI(const flt a0, const flt a1, const flt a2, const flt size,
					 const flt minX, const flt maxX);

	/**
	  Updates parabola and size.
	 **/
	void update();
};




}
}
}
}
