/**
 *  \author Sebastian Hempel
 *  \brief Display safety distances to obstacles.
 */

#pragma once


#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <gui/Painter3DTask.h>
#include <math/AutoMath.h>
#include <modules/models/egostate/EgoState.h>
#include <qmutex.h>

#include "Object.h"


namespace aa
{
namespace modules
{
namespace display
{
namespace obstacledistances
{


class DisplayObstacleDistances
	: public gui::Painter3DTask
{
public:
	explicit DisplayObstacleDistances(std::string const & name);
	virtual ~DisplayObstacleDistances();

protected:
	virtual void updateHook();
	virtual void draw3D(DrawArg);


	/**
	 *  short for obstacle pointer
	 */
	typedef TimedBaseObstacleBundle_ptr Obstacles_ptr;


	/**
	 *  bunch of objects
	 */
	typedef std::vector<ObstacleObject> Objects;


	/**
	 *  incoming ego state
	 */
	RTT::InputPort<TimedEgoState> mEgoStateIn;


	/**
	 *  incoming obstacles
	 */
	RTT::InputPort<Obstacles_ptr> mObstaclesIn;


	/**
	 *  use obstacle height for rendering?
	 */
	RTT::Property<bool> mUseObstacleHeight;


	/**
	 *  safety distance
	 */
	RTT::Property<math::flt> mSafetyDistance;


	/**
	 *  render obstacle?
	 */
	RTT::Property<bool> mRenderObstacle;


	/**
	 *  render safety area?
	 */
	RTT::Property<bool> mRenderSafetyArea;


	/**
	 *  display back fac of obstacles?
	 */
	RTT::Property<bool> mRenderBackFace;


	/**
	 *  \brief Lock global lock.
	 */
	void lock();


	/**
	 *  \brief Unlocks global lock.
	 */
	void unlock();


	/**
	 *  \brief Updates module.
	 */
	void updateSomething();


	/**
	 *  \brief Renders data.
	 */
	void render();


	/**
	 *  current obstacles
	 */
	Obstacles_ptr mObstacles;


	/**
	 *  objects generated from obstacles
	 */
	Objects mObjects;


private:
	/**
	 *  \brief Creates ports.
	 */
	void createPorts();


	/**
	 *  \brief Creates methods.
	 */
	void createMethods();


	/**
	 *  \brief Creates properties.
	 */
	void createProperties();


	/**
	 *  \brief Sets obstacle color.
	 *  \param[in] red red
	 *  \param[in] green green
	 *  \param[in] blue blue
	 */
	void setObstacleColor(const float red, const float green, const float blue);


	/**
	 *  \brief Sets boundary color.
	 *  \param[in] red red
	 *  \param[in] green green
	 *  \param[in] blue blue
	 */
	void setBoundaryColor(const float red, const float green, const float blue);


	/**
	 *  \brief Sets border color.
	 *  \param[in] red red
	 *  \param[in] green green
	 *  \param[in] blue blue
	 */
	void setBorderColor(const float red, const float green, const float blue);


	/**
	 *  \brief Reads obstacles from port.
	 */
	bool readObstacles();


	/**
	 *  \brief Reads data from ego state.
	 */
	bool readEgoState();


	/**
	 *  color of obstacle area
	 */
	math::Vec3f mObstacleColor;


	/**
	 *  color of obstacle boundary
	 */
	math::Vec3f mBoundaryColor;


	/**
	 *  color of safety margin border
	 */
	math::Vec3f mBorderColor;


	/**
	 *  current ego state
	 */
	TimedEgoState mEgoState;


	/**
	 *  global lock
	 */
	QMutex mLock;
};


}
}
}
}
