#include "DisplayObstaclesBase.h"
#include <rtt/Logger.hpp>
#include <rtt/OperationCaller.hpp>

using namespace aa::modules::display::obstacles;
using namespace RTT;

DisplayObstaclesBase::DisplayObstaclesBase(std::string const & name)
	: Painter3DTask(name)
	, mRenderBoundingBoxes("renderBoxes", true)
	, mRenderSolidBoxes("renderSolid", true)
	, mRenderObstacleIds("renderIds", true)
	, mRenderObstacleCenters("renderCenters", true)
	, mRenderOrientations("renderOrientations", true)
	, mRenderContourPoints("renderContourPoints", true)
{
	addAttribute(mRenderBoundingBoxes);
	addAttribute(mRenderSolidBoxes);
	addAttribute(mRenderObstacleIds);
	addAttribute(mRenderObstacleCenters);
	addAttribute(mRenderOrientations);
	addAttribute(mRenderContourPoints);


	addOperation("showBoxes", &DisplayObstaclesBase::setShowBoxes, this, RTT::ClientThread).doc("showBoxes").arg("value", "");

	addOperation("showSolid", &DisplayObstaclesBase::setShowSolid, this, RTT::ClientThread).doc("showSolid").arg("value", "");

	addOperation("showIds", &DisplayObstaclesBase::setShowIds, this, RTT::ClientThread).doc("showIds").arg("value", "");

	addOperation("showCenters", &DisplayObstaclesBase::setShowCenters, this, RTT::ClientThread).doc("showCenters").arg("value", "");

	addOperation("showOrientations", &DisplayObstaclesBase::setShowOrientations, this, RTT::ClientThread).doc("showOrientations").arg("value", "");

	addOperation("showContourPoints", &DisplayObstaclesBase::setShowContourPoints, this, RTT::ClientThread).doc("showContourPoints").arg("value", "");
}


bool DisplayObstaclesBase::startHook()
{
	return true;
}


void DisplayObstaclesBase::draw3D(DrawArg)
{
	mLock.lock();

	bool 	boxes = mRenderBoundingBoxes.get(),
			solid = mRenderSolidBoxes.get(),
			ids = mRenderObstacleIds.get(),
			obstacles = mRenderObstacleCenters.get(),
			orientations = mRenderOrientations.get(),
			contour = mRenderContourPoints.get();

	glPushAttrib(GL_LIGHTING);
	glDisable(GL_LIGHTING);

	renderObstacles(boxes, solid, ids, obstacles, orientations, contour);

	glPopAttrib();

	mLock.unlock();
}

void DisplayObstaclesBase::updateHook()
{
	mLock.lock();

	updateObstacles();

	mLock.unlock();
}
