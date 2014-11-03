
#include "ObstacleBundle.h"

namespace aa
{
namespace data
{
namespace obstacle
{

ObstacleBundle::ObstacleBundle()
{

}


ObstacleBundle::~ObstacleBundle()
{

}


bool ObstacleBundle::isValid() const
{
	for (unsigned int i = 0; i < size(); i++) {
		if (!at(i).boundingBox().isValid()) {
			return false;
		}
	}

	return true;
}

void ObstacleBundle::setTimeStampForEachObstacle(TimeStamp const & t)
{
	for (unsigned int i = 0; i < size(); i++) {
		at(i).setUpdated(t);
	}
}


}
}
}

namespace RTT
{
template class InputPort<TimedObstacleBundle_ptr>;
template class OutputPort<TimedObstacleBundle_ptr>;
}
