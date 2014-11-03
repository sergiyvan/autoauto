#pragma once

#include "BaseObstacle.h"

#include <vector>
#include <core/TimedData.h>
#include <rtt/Port.hpp>
#include <util/aligned_allocator.h>
#include <util/PooledObjectTemplate.h>
#include "ObstacleSkdTree.h"

/**
Bundle of raw obstacles.
**/
class BaseObstacleBundle
	: public std::vector < aa::data::obstacle::BaseObstacle, ::util::aligned_allocator< aa::data::obstacle::BaseObstacle, 16u> >
	, boost::noncopyable
{
public:
	//a type shortcut
	typedef std::vector < aa::data::obstacle::BaseObstacle, ::util::aligned_allocator< aa::data::obstacle::BaseObstacle, 16u> > parent_type;
	typedef aa::data::obstacle::ObstacleSkdTree< aa::data::obstacle::BaseObstacle, 2, aa::data::obstacle::ObstacleContourPtr< aa::data::obstacle::BaseObstacle, 2> > SkdTree;

	BaseObstacleBundle();

	enum Sizes {
		MAX_NUM_OBSTACLES = 400
	};

	void buildTree();

	SkdTree const & skdTree() const {
		return mSkdTree;
	}

	void clear();

private:
	SkdTree mSkdTree;
};

TIMEDPOOLEDOBJECT(BaseObstacleBundle, 8);


namespace RTT
{
extern template class InputPort<TimedBaseObstacleBundle_ptr>;
extern template class OutputPort<TimedBaseObstacleBundle_ptr>;
}
