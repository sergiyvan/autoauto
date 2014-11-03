#include "SimulatorEngine.h"

#include "AutoSimulant.h"
#include "Simulator.h"

// #include <ctime>
// #include <iostream>
// #include <fstream>
#include <math/AutoMath.h>
#include <data/VehicleData.h>
#include <aa/data/obstacle/BaseObstacle.h>


#if defined(PHYSX)
#include "PhysXBackend.h"
#else
#include "SimpleBackend.h"
#endif
#include <QMutex>

#include <iostream>
#include <vector>

#include <boost/concept_check.hpp>

using namespace aa::modules::nav::simulator;
using namespace modules::models::carstate;
using namespace RTT;
using namespace boost;
using namespace std;
using namespace ::math;

MATH_TYPES_MAT(flt, 4, 4) genDrawData(std::string const & model, Vec3 const & position, Quaternion const & q,  Vec3 const & scale);

/**
* @brief We keep the data for each object in the world in here
*/

/**
 * Register simple objects like obstacles.
 *
 * @param fixed
 * @param pos
 * @param size
 * @param orientation
 * @param model
 * @param modelscale
 * @param mat
 * @param name
 * @return Simulant ID
 */
unsigned int SimulatorEngine::registerObject(bool fixed,  Vec3 const & pos,  Vec3 const & size,
		Quaternion const & orientation,  std::string const & model, Vec3 const & modelscale,
		Vec4 const & mat, std::string const & name)
{
	Logger::In in("SimulatorEngine");

	Simulant * s = backend->getSimulant();
	s->material = mat;
	s->size = size;
	s->setPosition(pos);
	s->setOrientation(orientation);
	s->model = model;
	s->modelscale = modelscale;
	s->name = name;
	s->id = objects.size();
	s->immovable = fixed;

	Logger::log(Logger::Debug) << "registered object " << s->name << " with id: " << s->id << " at position: " << s->pos() << Logger::endl;

	objects.push_back(s);
	return s->id;

}

/**
 * Create and register new AutoSimulant instance
 * @param pos
 * @param orientation
 * @return AutoSimulant ID
 */
unsigned int SimulatorEngine::registerAuto(Vec3 const & pos,  Quaternion const & orientation)
{
	Simulant * s = backend->getAutoSimulant();
	s->setPosition(pos);
	s->setOrientation(orientation);;
	s->name = "Auto";
	s->model = "car.ac";
	s->modelscale = Vec3(1, 1, 1);
	s->id = objects.size();
	s->size = Vec3(3, 2, 1.8);
	objects.push_back(s);
	Logger::log(Logger::Debug) << "registered auto " << s->name << " with id: " << s->id << " at position: " << s->pos() << Logger::endl;
	return s->id;
}

/**
 * Delete a registered object by its ID
 * @param objectId of the object you want to get rid of
 */
void SimulatorEngine::destroyObject(unsigned int objectId)
{

	// delete object in list
	objects.erase(objects.begin() + objectId);

	// since simulant objects are accessed by there ID,
	// we need to correct it after deleting an object
	for (int i = objectId; i < objects.size(); i++) {
		objects[i]->id -= 1;
	}
}

SimulatorEngine::SimulatorEngine()
// 	:  mObjects("Objects")
#if defined(PHYSX)
	:  backend(new PhysXBackend())
#else
	:  backend(new SimpleBackend())
#endif
{
}

SimulatorEngine::~SimulatorEngine()
{
}

void SimulatorEngine::setPosition(unsigned int id, Vec3 const & pos, bool force)
{
	objects[id]->setPosition(pos);
}

void SimulatorEngine::setOrientation(unsigned int id, Quaternion const & ori, bool force)
{
	objects[id]->setOrientation(ori);
}

void SimulatorEngine::setVelocity(unsigned int id, Vec3 const & vel, bool force)
{
	objects[id]->setVelocity(vel);
}

void SimulatorEngine::setSteer(unsigned int id, flt const & steer, bool force)
{
	objects[id]->setSteer(steer);
}

void SimulatorEngine::setAcceleration(unsigned int id, Vec3 const & acc, bool force)
{
	objects[id]->setAcceleration(acc);
}

void SimulatorEngine::setAuxDevicesData(unsigned int id, AuxDevicesData const & auxData)
{
	objects[id]->setAuxDevicesData(auxData);
}

Vec3 SimulatorEngine::getPosition(unsigned int id) const
{
	return objects[id]->pos();
}

Vec3 SimulatorEngine::getMovementDirection(unsigned int id) const
{
	return ((AutoSimulant const *)objects[id])->model.movementDirection;
}

Quaternion SimulatorEngine::getOrientation(unsigned int id) const
{
	return objects[id]->orientation();
}

Vec3 SimulatorEngine::getVelocity(unsigned int id) const
{
	return objects[id]->velocity();
}

Vec3 SimulatorEngine::getAcceleration(unsigned int id) const
{
	return objects[id]->acceleration();
}

flt SimulatorEngine::getSpeed(unsigned int id) const
{
	return ((AutoSimulant const *)objects[id])->model.currSpeed;
}

flt SimulatorEngine::getSteer(unsigned int id) const
{
	return ((AutoSimulant const *)objects[id])->model.currSteer;
}

int SimulatorEngine::getGear(unsigned int id) const
{
	return ((AutoSimulant const *)objects[id])->model.mGear;
}

flt SimulatorEngine::getYawRate(unsigned int id) const
{
// 	return ((AutoSimulant const *)objects[id])->model.mYawRate;
	return 0.0f;
}

std::string SimulatorEngine::getInfos(unsigned int id) const
{
	return ((AutoSimulant const *)objects[id])->infos;
}

/**
 *
 * @param id
 * @param gear
 * @param speed
 * @param brake
 * @param steer
 */
void SimulatorEngine::control(unsigned int id, int gear, flt speed, flt brake, flt steer)
{
	AutoSimulant * pAuto = (AutoSimulant *)objects[id];

    assert(!std::isnan(speed));
    assert(!std::isnan(brake));
    assert(!std::isnan(steer));
	assert(CarState::isValidGear(gear));

	pAuto->speedWish = speed;
	pAuto->brakeWish = brake;
	pAuto->steerWish = steer;
	pAuto->gearWish = gear;

	switch (gear) {
	case CarState::GEAR_BETWEEN:	// we are changing gears,  so we are on the brake
	case CarState::GEAR_PARK:		// we are parking,  so we are on the brake
		pAuto->speedWish = 0.0f;
		pAuto->brakeWish = 2.0f;
		break;
	case CarState::GEAR_NEUTRAL:				// N Mode no speed
		pAuto->speedWish = 0.0f;
		pAuto->brakeWish = 0.0f;
		break;
	case CarState::GEAR_REVERSE:				// reverse mode,  speed is now backward
		break;
	case CarState::GEAR_DRIVE:
		break;								// D and normal driving
	case CarState::GEAR_SPORT:				// S and normal driving
	default:
		break;
	}

	pAuto->dirty = true;
}

flt SimulatorEngine::getCurrentFrameTime() const
{
	return currentFrameTime;
}

void SimulatorEngine::pause(bool p)
{
	if (p) {
		currentFrameTime = 0.f;
	}

	lastTimeStamp.stamp();
}


void SimulatorEngine::update()
{
	TimeStamp now;
	now.stamp();
	//std::cout << "********************** Shaft Distance: " << SimulatorEngine::mShaftDistance << std::endl;

	// check if all autonoumous cars are dirty
	for (objects_collection_type::const_iterator it = objects.begin(); it != objects.end(); it++) {
		if ((*it)->typ == 1)
			if (!((AutoSimulant *)(*it))->dirty) {
				return;
			}
	}

	if (lastTimeStamp == TimeStamp()) {
		currentFrameTime = 0.001f;
	}
	else {
		// TODO Hack by Daniel - until we can use getperiod here!
		currentFrameTime = 0.02f;
		//currentFrameTime = getPeriod();//(RTT::os::TimeService::ticks2nsecs(now - lastTimeStamp) / 1000000) / 1000.0f;
	}

	lastTimeStamp = now;

    std::vector<boost::tuple<Mat4x4,  std::string,  Vec4> > blocks;
	assert(!std::isnan(currentFrameTime));

	int simuls = 0, autosimuls = 0;

	// update Simulant-Objects
	for (objects_collection_type::const_iterator it = objects.begin(); it != objects.end(); it++) {
		switch ((*it)->typ) {
		case 0 :	// Simulant is dumb object,  needs nothing to be done
			(*it)->simulate(currentFrameTime);
			simuls++;
			break;
		case 1 :	//AutoSimulant
			AutoSimulant * a = ((AutoSimulant *) * it);
			a->simulate(currentFrameTime);
			generateObstacles((*it)->id);
			autosimuls++;
			break;
		}

		Mat4x4 drawData = genDrawData((*it)->name, (*it)->pos(), (*it)->orientation(), (*it)->modelscale);
        blocks.push_back(boost::make_tuple(drawData, (*it)->name, (*it)->material));
	}

	// Update backend
	backend->update(currentFrameTime);

	/*/ Timing tests
	static int timecounter = 0;
	long long newTimeStamp = RTT::os::TimeService::Instance()->getNSecs();
	flt time = (( newTimeStamp - currentTimeStamp) / 1000000.0);
	std::ofstream fos("timings.csv", (timecounter>0?std::ios_base::out|std::ios_base::app:std::ios_base::out));
	if (timecounter==5)
		fos << backend->getName() << "_" << objects.size() << std::endl;
	if (timecounter > 4)
		fos << time << std::endl;
	fos.close();
	timecounter++;
	if (timecounter>1005)
		exit(0);
	/*/ // Ende der Timing Tests
// 	mObjects.write( blocks );

	for (objects_collection_type::const_iterator it = objects.begin(); it != objects.end(); it++) {
		if ((*it)->typ == 1) {
			((AutoSimulant *)(*it))->dirty = false;
		}
	}
}

/**
 *
 * @param id
 */
void SimulatorEngine::generateObstacles(unsigned int id)
{

	TimedBaseObstacleBundle_ptr oData = AutoBaseObstacleBundle();
	oData->clear();

	for (unsigned int i = 0; i < objects.size(); i++) {
		if (i == id) {
			continue;
		}

        aa::data::obstacle::BaseObstacle obstacle;
        aa::data::obstacle::util::Contour points;

		Vec2 pos = head(objects[i]->pos());

		const Vec3  & obstacleSize = objects[i]->size;
		Vec2 size = head(obstacleSize);

		Quaternion orientation = objects[i]->orientation();
		Vec2 o = normalized(head(orientation.toRotationMatrix() * Vec3(1, 0, 0)));
		Vec2 p(o[1], -o[0]); //rotate2d(o, M_PI_2);

		std::vector< ::math::Vec2> setForConvexhull;

		setForConvexhull.push_back(pos + size(0) * o + size(1) * p);
		setForConvexhull.push_back(pos - size(0) * o + size(1) * p);
		setForConvexhull.push_back(pos - size(0) * o - size(1) * p);
		setForConvexhull.push_back(pos + size(0) * o - size(1) * p);

		// compute contour points based on a convex hull and
		// the current position of the car
		points.compute(setForConvexhull, head(objects[id]->pos()));

		// TODO do not set classification to CAR, use obstacles type of model
//		if (objects[i]->immovable)  // isStatic
        obstacle.setClassification(aa::data::obstacle::Classification::CAR);
		obstacle.setId(objects[i]->id);
		obstacle.setContour(points);
        obstacle.setLocation(aa::data::obstacle::util::LARGELY_ON_LANE);
		obstacle.setBoundingCircleCentre(pos);
        obstacle.setBoundingCircleRadius(sqrt(size(0) * size(0) + size(1) * size(1)));
        obstacle.setVelocity(objects[i]->velocity());

// 		obstacle.setAcceleration( Vecto_flt(objects[j]->acceleration()) );
		if (objects[i]->velocity().norm() == 0.f && objects[i]->acceleration().norm() == 0.f) {
			obstacle.setMovementState(data::obstacle::BaseObstacle::MOVEMENT_STATE_STATIC);
		}
		else {
			obstacle.setMovementState(data::obstacle::BaseObstacle::MOVEMENT_STATE_DYNAMIC);
		}

		oData->push_back(obstacle);
	}

	oData->buildTree();
	oData->stamp();

	((AutoSimulant *) objects[id])->timedObstacleBundle = oData;
}

void SimulatorEngine::draw()
{
	backend->draw();
}

TimedBaseObstacleBundle_ptr SimulatorEngine::getObstacles(unsigned int id) const
{
	return ((AutoSimulant const *)objects[id])->timedObstacleBundle;
}


Mat4x4 genDrawData(std::string const & model, Vec3 const & position, Quaternion const & q,  Vec3 const & scale)
{
	Mat4x4 Rt(Mat4x4::Zero());
	Mat3x3 R = q.toRotationMatrix().transpose();

// 	for (uint i = 0; i < 3; ++i) {
// 		Rt(i, 0) = R(i, 0) * scale(i);
// 		Rt(i, 1) = R(i, 1) * scale(i);
// 		Rt(i, 2) = R(i, 2) * scale(i);
// 	}
	Rt.block<3, 3>(0, 0) = R;

	Rt(0, 3) = position(0);
	Rt(1, 3) = position(1);
	Rt(2, 3) = position(2);
	Rt(3, 3) = flt(1);
	return Rt;
}

