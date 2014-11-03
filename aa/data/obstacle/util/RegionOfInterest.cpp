#include "RegionOfInterest.h"
#include <set>
#include <cmath>
#include <rtt/Logger.hpp>
#include <math/AutoMath.h>
#include <boost/foreach.hpp>

using namespace RTT;
using namespace ::math;
using namespace aa::data::obstacle::util;


ObstacleRegionOfInterest::ObstacleRegionOfInterest()
{

}


void ObstacleRegionOfInterest::update(const Vec3 & position,
									  const Vec3 & direction,
									  flt speed)
{
	mPosition = position;
	mDirection = direction;
	mSpeed = speed;

	mDirection[2] = 0.0f;
	mDirection.normalize();

	orthoByForward();
}


/* ----------------------- some ROI inmplementations ------------------------ */

CircleROI::CircleROI(flt radius)
	: ObstacleRegionOfInterest()
{
	mRadius = radius;
	mSquaredRadius = radius * radius;
}


bool CircleROI::hasPoint(Vec3 const & point) const
{
	return (mPosition - point).squaredNorm() <= mSquaredRadius;
}


bool CircleROI::hasPoint(Vec3 const & point, flt borderwidth) const
{
	return (mPosition - point).squaredNorm() <= (mSquaredRadius - (borderwidth * fabs(borderwidth)));
}


Vec2 CircleROI::getBoundingRectangle() const
{
	flt size = 2.0f * mRadius;

	return Vec2(size, size);
}


SharedRoi CircleROI::copy() const
{
	return SharedRoi(new CircleROI(mRadius, mSquaredRadius));
}


CircleROI::CircleROI(
	const flt radius,
	const float squaredRadius)
	: mRadius(radius)
	, mSquaredRadius(squaredRadius)
{

}




EllipseROI::EllipseROI(flt length, flt width)
	: ObstacleRegionOfInterest()
{
	mLength = length;
	mWidth = width;
	mSqrdHalfLength = length * length * 0.25f;
	mSqrdHalfWidth = width * width * 0.25f;
}


bool EllipseROI::hasPoint(Vec3 const & point) const
{
	Vec2 p = project(point);

	return (p[0] * p[0] / mSqrdHalfLength + p[1] * p[1] / mSqrdHalfWidth) <= 1;
}


bool EllipseROI::hasPoint(Vec3 const & point, flt borderwidth) const
{
	flt borderwidth2 = fabs(borderwidth) * borderwidth;

	if (borderwidth2 >= mSqrdHalfLength || borderwidth2 >= mSqrdHalfWidth) {
		return false;
	}

	Vec2 p = project(point);

	return (p[0] * p[0] / (mSqrdHalfLength - borderwidth2) + p[1] * p[1] / (mSqrdHalfWidth - borderwidth2)) <= 1;
}


Vec2 EllipseROI::getBoundingRectangle() const
{
	return Vec2(mLength, mWidth);
}


SharedRoi EllipseROI::copy() const
{
	return SharedRoi(new EllipseROI(
						 mLength,
						 mWidth,
						 mSqrdHalfLength,
						 mSqrdHalfWidth));
}


EllipseROI::EllipseROI(
	const flt length,
	const flt width,
	const flt sqrdHalfLength,
	const flt sqrdHalfWidth)
	: mLength(length)
	, mWidth(width)
	, mSqrdHalfLength(sqrdHalfLength)
	, mSqrdHalfWidth(sqrdHalfWidth)
{

}


ParabolaROI::ParabolaROI(flt a0, flt a1, flt a2, flt size)
	: ObstacleRegionOfInterest()
{
	mA0 = a0;
	mA1 = a1;
	mA2 = a2;
	mSize = size;

	calcXs();
}


void ParabolaROI::calcXs()
{
	//where does it end?
	//f(x) = size
	// <=> x^2 + (a1/a2)*x + (a0-size)/a2 = 0
	// => p = (a1/a2)
	//    q =(a0-size)/a2
	// => x1/2 = -p/2 [+/-] sqrt(p*p/4 - q)

	flt p = mA1 / mA2,
		q = (mA0 - mSize) / mA2,
		t = sqrt(p * p * 0.25f - q),
		p2 = p * 0.5f;

	mMinX = p2 - t;
	mMaxX = p2 + t;
}


int sgn(flt x)
{
	return (x > 0.0f) - (x < 0.0f);
}


bool ParabolaROI::hasPoint(Vec3 const & point) const
{
	Vec2 r = project(point);
	flt f = eval(r[1]);

	//return sgn(r[0] - f) == sgn(mA2);
	return r[0] >= f && r[0] <= mSize;
}


bool ParabolaROI::hasPoint(Vec3 const & point, flt borderwidth) const
{
	Vec2 r = project(point);
	flt f = eval(r[1]);

	f += borderwidth;

	//return sgn(r[0] - f) == sgn(mA2);
	return r[0] >= f && r[0] <= mSize;
}


Vec2 ParabolaROI::getBoundingRectangle() const
{
	flt x0 = -0.5f * mA1 / mA2,
		s = eval(x0);

	return Vec2(mSize - s, mMaxX - mMinX);
}

Vec2 ParabolaROI::getBoundingPosition() const
{
	flt x0 = -0.5f * mA1 / mA2,
		s = eval(x0);

	flt x = (mMaxX + mMinX) * 0.5f,
		y = (mSize - s) * 0.5f + s;

	Vec3 p = mPosition +
			 (mDirection * y) +
			 (mOrthogonal * x);

	return Vec2(p[0], p[1]);
}



flt ParabolaROI::eval(flt x) const
{
	return mA0 + mA1 * x + mA2 * x * x;
}


SharedRoi ParabolaROI::copy() const
{
	return SharedRoi(new ParabolaROI(
						 mA0,
						 mA1,
						 mA2,
						 mSize,
						 mMinX,
						 mMaxX));
}


ParabolaROI::ParabolaROI(
	const flt a0,
	const flt a1,
	const flt a2,
	const flt size,
	const flt minX,
	const flt maxX)
	: mA0(a0)
	, mA1(a1)
	, mA2(a2)
	, mSize(size)
	, mMinX(minX)
	, mMaxX(maxX)
{

}


OffsetROI::OffsetROI(
	SharedRoi other,
	Vec3 & offset)
	: ObstacleRegionOfInterest()
{
	init(other, offset);
}


OffsetROI::OffsetROI(
	SharedRoi other,
	Vec3 offset)
	: ObstacleRegionOfInterest()
{
	init(other, offset);
}


void OffsetROI::init(
	SharedRoi other,
	Vec3 & offset)
{
	mOther = other;
	mOffset = offset;
}


bool OffsetROI::hasPoint(Vec3 const & point) const
{
	return mOther->hasPoint(point);
}


bool OffsetROI::hasPoint(Vec3 const & point, flt borderwidth) const
{
	return mOther->hasPoint(point, borderwidth);
}


Vec2 OffsetROI::getBoundingRectangle() const
{
	return mOther->getBoundingRectangle();
}


Vec2 OffsetROI::getBoundingPosition() const
{
	return mOther->getBoundingPosition();
}


void OffsetROI::update(const Vec3 & position,
					   const Vec3 & direction,
					   flt speed)
{

	//project offset
	Vec3 offset(direction[0] * mOffset[0] + direction[1] * mOffset[1],
				direction[1] * mOffset[0] - direction[0] * mOffset[1],
				mOffset[2]);

	mPosition = position + offset;
	mOther->update(mPosition,
				   direction,
				   speed);
}


SharedRoi OffsetROI::copy() const
{
	return SharedRoi(new OffsetROI(
						 mOther,
						 mOffset));
}


BinaryROI::BinaryROI(
	SharedRoi first,
	SharedRoi second)
	: mFirst(first)
	, mSecond(second)
{
}


BinaryROI::~BinaryROI()
{
}


Vec2 BinaryROI::getBoundingRectangle() const
{
	return mRectDimensions;
}


Vec2 BinaryROI::getBoundingPosition() const
{
	return mRectPosition;
}

flt dot23(Vec2 & a, Vec3 & b)
{
	return a[0] * b[0] + a[1] * b[1];
}


void BinaryROI::update(const Vec3 & position,
					   const Vec3 & direction,
					   flt speed)
{
	mFirst->update(position, direction, speed);
	mSecond->update(position, direction, speed);

	mDirection = direction;
	mSpeed = speed;
	orthoByForward();


	//update rectangle dimensions
	Vec2 a = mFirst->getBoundingRectangle(),
		 b = mSecond->getBoundingRectangle(),
		 pb = mSecond->getBoundingPosition(),
		 pa = mFirst->getBoundingPosition(),
		 d = pb - pa;

	a *= 0.5f;
	b *= 0.5f;

	flt p = dot23(d, mDirection),
		q = dot23(d, mOrthogonal),
		back = std::min(-a[0], p - b[0]),
		front = std::max(a[0], p + b[0]),
		right = std::min(-a[1], q - b[1]),
		left = std::max(a[1], q + b[1]),
		length = front - back,
		width = left - right;

	mRectPosition =
		pa +
		(back + length * 0.5f) * Vec2(mDirection[0], mDirection[1]) +
		(right + width * 0.5f) * Vec2(mOrthogonal[0], mOrthogonal[1]);

	mRectDimensions = Vec2(length, width);
}


BinaryROI::BinaryROI(
	const SharedRoi first,
	const SharedRoi second,
	const Vec2 & rectDimensions,
	const Vec2 & rectPosition)
	: mFirst(first->copy())
	, mSecond(second->copy())
	, mRectDimensions(rectDimensions)
	, mRectPosition(rectPosition)
{

}



RectangleROI::RectangleROI(flt length, flt width)
	: ObstacleRegionOfInterest()
{
	mLength = length;
	mWidth = width;
}


bool RectangleROI::hasPoint(Vec3 const & point) const
{
	Vec2 x = project(point);

	flt l = mLength * 0.5f,
		w = mWidth * 0.5f;

	return (x[0] >= -l) && (x[0] <= l) && (x[1] >= -w) && (x[1] <= w);
}


bool RectangleROI::hasPoint(Vec3 const & point, flt borderwidth) const
{
	Vec2 x = project(point);

	flt l = mLength * 0.5f,
		w = mWidth * 0.5f;

	if (borderwidth >= l || borderwidth >= w) {
		return false;
	}

	l = l - borderwidth;
	w = w - borderwidth;

	return (x[0] >= -l) && (x[0] <= l) && (x[1] >= -w) && (x[1] <= w);
}


Vec2 RectangleROI::getBoundingRectangle() const
{
	return Vec2(mLength, mWidth);
}

SharedRoi RectangleROI::copy() const
{
	return SharedRoi(new RectangleROI(
						 mLength,
						 mWidth));
}


NotROI::NotROI(SharedRoi region)
	: mRegion(region)
{

}


NotROI::~NotROI()
{

}


SharedRoi NotROI::copy() const
{
	return SharedRoi(new NotROI(mRegion));
}


Vec2 NotROI::getBoundingRectangle() const
{
	return mRegion->getBoundingRectangle();
}


Vec2 NotROI::getBoundingPosition() const
{
	return mRegion->getBoundingPosition();
}

void NotROI::update(const Vec3 & position,
					const Vec3 & direction,
					flt speed)
{
	mRegion->update(position, direction, speed);
}


bool NotROI::hasPoint(Vec3 const & point) const
{
	return !mRegion->hasPoint(point);
}


bool NotROI::hasPoint(Vec3 const & point, flt borderwidth) const
{
	return !mRegion->hasPoint(point, borderwidth);
}


void SpeedParabolaROI::update()
{
	mSize = std::min(std::max(flt(20), flt(2) * sqr(mSpeed)), flt(60));
}


void SpeedParabolaROI::update(const Vec3 & position,
							  const Vec3 & direction,
							  flt speed)
{
	ParabolaROI::update(position, direction, speed);

	update();

	calcXs();
}


SharedRoi SpeedParabolaROI::copy() const
{
	return SharedRoi(new SpeedParabolaROI(
						 mA0,
						 mA1,
						 mA2,
						 mSize,
						 mMinX,
						 mMaxX));
}


SpeedParabolaROI::SpeedParabolaROI(
	const flt a0,
	const flt a1,
	const flt a2,
	const flt size,
	const flt minX,
	const flt maxX)
	: ParabolaROI(a0, a1, a2, size, minX, maxX)
{

}



