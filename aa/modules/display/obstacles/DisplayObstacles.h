#pragma once

#include "DisplayObstaclesBase.h"
#include <aa/data/obstacle/Obstacle.h>
#include <aa/data/obstacle/ObstacleBundle.h>
#include <aa/modules/fusion/obstaclefusion/TrackedObstacle.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace obstacles
{

class DisplayObstacles
	: public DisplayObstaclesBase
{
public:

	explicit DisplayObstacles(std::string const & name);

protected:

	//incoming obstacles
	RTT::InputPort<TimedObstacleBundle_ptr> mObstacleBundleIn;

	//last seen bundle
	TimedObstacleBundle_ptr mCurrentBundle;

	//renders all obstacles
	virtual void renderObstacles(bool boxes, bool solid, bool ids,
	                             bool center, bool orientations, bool contour);


	//called if new obstacles are available
	virtual void updateObstacles();

	osg::BoundingBox computeBound() const;
};

}
}
}
}
