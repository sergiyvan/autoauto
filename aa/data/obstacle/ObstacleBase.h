#if 0

#pragma once

#include <vector>
#include <core/TimedData.h>
#include <util/aligned_allocator.h>
#include <util/PooledObjectTemplate.h>
#include <rtt/Port.hpp>


class ObstacleBase
{
public:
	enum Classification {
		UNCLASSIFIED,
		UNKNOWN,
		CAR
	};

	ObstacleBase() { }
	ObstacleBase(Vec3 & position,
				 Vec3 & primaryDirection,
				 Vec3 & secondaryDirection,
				 Vec3 & size) {
		init(position,
			 primaryDirection, secondaryDirection,
			 size,
			 UNCLASSIFIED);
	}

	ObstacleBase(Vec3 & position,
				 Vec3 & primaryDirection,
				 Vec3 & secondaryDirection,
				 Vec3 & size,
				 Classification classification) {
		init(position,
			 primaryDirection, secondaryDirection,
			 size,
			 classification);
	}


	bool isClassified() {
		return mClass != UNCLASSIFIED;
	}


	bool isUnknown() {
		return mClass == UNKNOWN;
	}


	bool isCar() {
		return mClass == CAR;
	}


	ObstacleBase & operator=(const ObstacleBase & other) {
		if (this == &other) {
			return *this;
		}

		mPosition = other.mPosition;
		mPrimaryDirection = other.mPrimaryDirection;
		mClass = other.mClass;

		return *this;
	}

	//obstacle's position and size
	Vec3 mPosition,
		 mSize;

	//obstacle's orientations
	Vec3 mPrimaryDirection,
		 mSecondaryDirection;

	//classification
	Classification mClass;

private:
	void init(Vec3 & position,
			  Vec3 & primaryDirection, Vec3 & secondaryDirection,
			  Vec3 & size,
			  Classification classification) {
		mPosition = position;
		mPrimaryDirection = primaryDirection;
		mSecondaryDirection = secondaryDirection;
		mSize = size;
		mClass = classification;
	}
};


class ObstacleBundle
	: public std::vector<ObstacleBase, util::aligned_allocator<ObstacleBase, 16u> >
{
public:
	typedef std::vector<ObstacleBase, util::aligned_allocator<ObstacleBase, 16u> > parent_type;

	ObstacleBundle() { }

	ObstacleBundle(size_t size) : parent_type(size) { }

	~ObstacleBundle() { }


	void setCarPosition(Vec3 position) {
		mCarPosition = position;
	}

	Vec3 getCarPosition() {
		return mCarPosition;
	}

protected:
	//car's position
	Vec3 mCarPosition;
};

TIMEDPOOLEDOBJECT(ObstacleBundle, 8);

namespace RTT
{
extern template class InputPort<TimedObstacleBundle_ptr>;
extern template class OutputPort<TimedObstacleBundle_ptr>;
}

#endif