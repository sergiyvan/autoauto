
#include "BaseObstacleBundle.h"


BaseObstacleBundle::BaseObstacleBundle()
{

}

void BaseObstacleBundle::clear()
{
	parent_type::clear();
	mSkdTree.clear();
}

void BaseObstacleBundle::buildTree()
{
	mSkdTree.rebuildTree(*this);
}


namespace RTT
{
template class InputPort<TimedBaseObstacleBundle_ptr>;
template class OutputPort<TimedBaseObstacleBundle_ptr>;
}
