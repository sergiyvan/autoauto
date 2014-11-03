#include "DisplayObstacles.h"
#include "ObstacleColorScheme.h"

#include <rtt/Logger.hpp>
#include <rtt/OperationCaller.hpp>
#include <util/TaskContextFactory.h>
#include <aa/modules/display/renderer/obstacle/Obstacle.h>

using namespace RTT;
using namespace aa::data::obstacle;
using namespace math;
using namespace aa::modules::display::obstacles;

REGISTERTASKCONTEXT(DisplayObstacles);



DisplayObstacles::DisplayObstacles(std::string const & name)
	: DisplayObstaclesBase(name)
	, mObstacleBundleIn("ObstacleBundle")
{
	ports()->addEventPort(mObstacleBundleIn);

	mCurrentBundle = AutoObstacleBundle();
}


void DisplayObstacles::updateObstacles()
{
	if (!mObstacleBundleIn.connected()) {
		RTT::log() << Logger::Error
				   << "Not connected with obstacle data!"
				   << RTT::Logger::endl;
		return;
	}

	TimedObstacleBundle_ptr bundle;
	mObstacleBundleIn.read(bundle);

	if (!bundle || mCurrentBundle == bundle) {
		return;
	}

	this->dirtyBound();
	mCurrentBundle = bundle;
}

osg::BoundingBox DisplayObstacles::computeBound() const
{
	Vec3 center;

	if (mCurrentBundle->size() > 0) {
		center = mCurrentBundle->at(0).boundingBox().center();
	}
	else {
		center = Vec3(0.0f, 0.0f, 0.0f);
	}

	return osg::BoundingBox(center[0] - 1000.0f,
							center[1] - 1000.0f,
							center[2] - 1000.0f,
							center[0] + 1000.0f,
							center[1] + 1000.0f,
							center[2] + 1000.0f);
}


void DisplayObstacles::renderObstacles(bool boxes, bool solid, bool ids,
									   bool center, bool orientations, bool contour)
{
	ObstacleColorScheme colorScheme(boxes,
									solid,
									ids,
									center,
									orientations,
									contour);

	Obstacle * obstacle = &mCurrentBundle->front();

	for (int i = 0; i < mCurrentBundle->size(); i++, obstacle++) {
		const Obstacle & o = * obstacle;

		ColorConfiguration config = colorScheme.make(o);

		modules::display::renderer::obstacle::Obstacle(*obstacle).render(config.mRenderBoundingBox,
				config.mBoundingBoxColor,
				config.mRenderSolid,
				config.mSolidBottomColor,
				config.mSolidTopColor,
				config.mRenderId,
				config.mIdColor,
				config.mRenderCenter,
				config.mCenterColor,
				config.mRenderOrientation,
				config.mOrientationFirstColor,
				config.mOrientationSecondColor,
				config.mRenderContourPoints,
				config.mContourColor,
				config.mContourWidth);
	}
}
