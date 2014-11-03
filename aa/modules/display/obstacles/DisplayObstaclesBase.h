#pragma once

#include <gui/Painter3DTask.h>
#include <qmutex.h>
#include <modules/models/egostate/EgoState.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace obstacles
{

class DisplayObstaclesBase
	: public gui::Painter3DTask
{
public:
	explicit DisplayObstaclesBase(std::string const & name);
	virtual bool startHook();
	virtual void updateHook();
	virtual void draw3D(DrawArg);

protected:
	RTT::Attribute<bool> mRenderBoundingBoxes,
		mRenderSolidBoxes,
		mRenderObstacleIds,
		mRenderObstacleCenters,
		mRenderOrientations,
		mRenderContourPoints;

	//lock the data
	QMutex mLock;

	//renders all obstacles
	virtual void renderObstacles(bool boxes, bool solid, bool ids,
								 bool center, bool orientations, bool contour) = 0;

	//is called if there are new obstacles available
	virtual void updateObstacles() = 0;

	inline bool setShowBoxes(const bool & value) {
		mLock.lock();
		mRenderBoundingBoxes.set(value);
		mLock.unlock();

		return true;
	}

	inline bool setShowSolid(const bool & value) {
		mLock.lock();
		mRenderSolidBoxes.set(value);
		mLock.unlock();

		return true;
	}

	inline bool setShowIds(const bool & value) {
		mLock.lock();
		mRenderObstacleIds.set(value);
		mLock.unlock();

		return true;
	}

	inline bool setShowCenters(const bool & value) {
		mLock.lock();
		mRenderObstacleCenters.set(value);
		mLock.unlock();

		return true;
	}

	inline bool setShowOrientations(const bool & value) {
		mLock.lock();
		mRenderOrientations.set(value);
		mLock.unlock();

		return true;
	}

	inline bool setShowContourPoints(const bool & value) {
		mLock.lock();
		mRenderContourPoints.set(value);
		mLock.unlock();

		return true;
	}
};

}
}
}
}
