#pragma once
/**
 * \file ObstaclePrediction.h
 * \brief Used to predict the movement of a bundle of obstacles based on their predict functionality
 * \author Georg Bremer <bremer@inf.fu-berlin.de>
 */

#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <core/TimeStamp.h>
#include <math/Types.h>

namespace aa
{
namespace modules
{
namespace nav
{

namespace obstacles
{

bool predictMovement(BaseObstacleBundle & in, TimeStamp const & t)
{
	bool ret = true;

	BaseObstacleBundle::iterator it_in(in.begin());

	aa::data::obstacle::BaseObstacle obst;

	for (; it_in != in.end(); ++it_in) {
		::math::flt maxPredTime = it_in->maxPredictionTime();
		::math::flt predTime = RTT::os::TimeService::ticks2nsecs(t - it_in->current()) / 1000000000.0;

		if (maxPredTime > 0.0 && predTime > 0.0) {
			predTime = std::min(predTime, maxPredTime);
			it_in->predictAndUpdateWithoutMeasurement(t, obst);
			*it_in = obst;
		}
	}

	in.buildTree();
	return ret;
}

}

}

}

}

