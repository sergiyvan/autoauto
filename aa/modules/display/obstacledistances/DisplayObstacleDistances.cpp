/**
 *  \author Sebastian Hempel
 *  \brief Display safety distances to obstacles.
 */


#include "DisplayObstacleDistances.h"
#include <util/TaskContextFactory.h>
#include <rtt/Logger.hpp>
#include <math/Types.h>
#include <util/Ports.h>
#include <GL/gl.h>

using namespace aa::modules::display::obstacledistances;


REGISTERTASKCONTEXT(DisplayObstacleDistances);


DisplayObstacleDistances::DisplayObstacleDistances(std::string const & name)
	: gui::Painter3DTask(name)
	, mObstacleColor(1.0f, 0.0f, 0.0f)
	, mBoundaryColor(1.0f, 0.0f, 0.0f)
	, mBorderColor(0.0f, 0.0f, 1.0f)
	, mEgoStateIn("EgoState")
	, mObstaclesIn("Obstacles")
	, mUseObstacleHeight(
		"useObstacleHeight",
		"use height of obstacles as ground for margin?",
		false)
	, mSafetyDistance(
		"safetyDistance",
		"safety distance in meters",
		1.5)
	, mRenderObstacle(
		"renderObstacle",
		"render obstacles?",
		false)
	, mRenderSafetyArea(
		"renderSafetyArea",
		"render safety area?",
		true)
	, mRenderBackFace(
		"renderBackFace",
		"render back face of obstacles?",
		false)

{
	createPorts();
	createMethods();
	createProperties();

	mObstacles = AutoBaseObstacleBundle();
	mObstacles->clear();
}


void DisplayObstacleDistances::createPorts()
{
	ports()->addEventPort(mObstaclesIn);
	ports()->addPort(mEgoStateIn);
}


void DisplayObstacleDistances::createMethods()
{
	addOperation("setObstacleColor", &DisplayObstacleDistances::setObstacleColor, this, RTT::ClientThread).doc("sets color of obstaces").arg("red", "red channel").arg("green", "green channel").arg("blue", "blue channel");

	addOperation("setBoundaryColor", &DisplayObstacleDistances::setBoundaryColor, this, RTT::ClientThread).doc("sets color of boundaries").arg("red", "red channel").arg("green", "green channel").arg("blue", "blue channel");

	addOperation("setBorderColor", &DisplayObstacleDistances::setBorderColor, this, RTT::ClientThread).doc("sets color of borders").arg("red", "red channel").arg("green", "green channel").arg("blue", "blue channel");
}


void DisplayObstacleDistances::createProperties()
{
	addProperty(mUseObstacleHeight);
	addProperty(mSafetyDistance);
	addProperty(mRenderObstacle);
	addProperty(mRenderSafetyArea);
	addProperty(mRenderBackFace);
}


DisplayObstacleDistances::~DisplayObstacleDistances()
{

}


void DisplayObstacleDistances::updateHook()
{
	lock();
	{
		updateSomething();
	}
	unlock();
}


void DisplayObstacleDistances::draw3D(DrawArg)
{
	lock();
	{
		render();
	}
	unlock();
}


void DisplayObstacleDistances::setObstacleColor(
	const float red,
	const float green,
	const float blue)
{
	mObstacleColor = math::Vec3f(red, green, blue);
}


void DisplayObstacleDistances::setBoundaryColor(
	const float red,
	const float green,
	const float blue)
{
	mBoundaryColor = math::Vec3f(red, green, blue);
}


void DisplayObstacleDistances::setBorderColor(
	const float red,
	const float green,
	const float blue)
{
	mBorderColor = math::Vec3f(red, green, blue);
}


void DisplayObstacleDistances::lock()
{
	mLock.lock();
}


void DisplayObstacleDistances::unlock()
{
	mLock.unlock();
}


void DisplayObstacleDistances::updateSomething()
{
	if (!readObstacles()) {
		return;
	}

	if (!readEgoState()) {
		return;
	}

	const math::flt safetyDistance = mSafetyDistance.value();

	mObjects.clear();

	for (Obstacles_ptr::element_type::const_iterator it = mObstacles->begin();
			it != mObstacles->end();
			++it) {
		const ObstacleObject::Obstacle & obstacle = *it;
		mObjects.push_back(ObstacleObject(obstacle, safetyDistance));
	}
}


void DisplayObstacleDistances::render()
{
	const bool renderObstacles = mRenderObstacle.value();
	const bool renderSafetyArea = mRenderSafetyArea.value();
	const bool renderBackFace = mRenderBackFace.value();

	const math::flt z = mUseObstacleHeight.value()
						? 0.0 : mEgoState.position()[2];

	glPushAttrib(GL_LIGHTING);
	glDisable(GL_LIGHTING);

	for (Objects::iterator it = mObjects.begin();
			it != mObjects.end();
			++it) {
		ObstacleObject & object = *it;
		object.render(z,
					  renderObstacles,
					  mObstacleColor,
					  renderSafetyArea,
					  mBoundaryColor,
					  mBorderColor,
					  renderBackFace);
	}

	glPopAttrib();
}


bool DisplayObstacleDistances::readObstacles()
{
	if (!mObstaclesIn.connected()) {
		RTT::log()
				<< RTT::Logger::Error
				<< "Obstacles are not connected"
				<< RTT::Logger::endl;

		return false;
	}

	mObstaclesIn.read(mObstacles);

	if (!mObstacles) {
		return false;
	}

	return true;
}


bool DisplayObstacleDistances::readEgoState()
{
	if (mUseObstacleHeight.value()) {
		return true;
	}

	if (!mEgoStateIn.connected()) {
		RTT::log()
				<< RTT::Logger::Error
				<< "EgoState ist not connected"
				<< RTT::Logger::endl;

		return false;
	}

	mEgoStateIn.read(mEgoState);

	return true;
}
