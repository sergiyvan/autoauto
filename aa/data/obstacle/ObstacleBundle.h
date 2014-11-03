#pragma once

#include "Obstacle.h"

#include <vector>
#include <core/TimedData.h>
#include <rtt/Port.hpp>
#include <util/aligned_allocator.h>
#include <util/PooledObjectTemplate.h>

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
  Bundle of raw obstacles.
**/
class ObstacleBundle
	: public std::vector< aa::data::obstacle::Obstacle, ::util::aligned_allocator< aa::data::obstacle::Obstacle, 16u> >
{
public:
	typedef boost::shared_ptr<util::ScannerSystem> ScannerSystemPtr;

	// Type shortcut
	typedef std::vector< aa::data::obstacle::Obstacle, ::util::aligned_allocator< aa::data::obstacle::Obstacle, 16u> > parent_type;

	ObstacleBundle();
	~ObstacleBundle();

	/// Scanner
	ScannerSystemPtr const scanner() const {
		return mScanner;
	}
	void setScanner(ScannerSystemPtr scanner) {
		mScanner = scanner;
	}

	/**
	*	Validates the obstacle bundle
	*/
	bool isValid() const;

	void setTimeStampForEachObstacle(TimeStamp const & t);

private:

	// Scanner system this obstacle bundle was created with
	ScannerSystemPtr mScanner;

};

}
}
}

typedef aa::data::obstacle::ObstacleBundle ObstacleBundle;
TIMEDPOOLEDOBJECT(ObstacleBundle, 8);

namespace RTT
{
extern template class InputPort<TimedObstacleBundle_ptr>;
extern template class OutputPort<TimedObstacleBundle_ptr>;
}
