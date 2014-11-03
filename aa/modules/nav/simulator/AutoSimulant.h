#pragma once
#include "Simulant.h"
#include "CarModel.h"
// #include <data/ObstacleData.h>
#include <aa/data/obstacle/BaseObstacleBundle.h>

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class AutoSimulant
	: public Simulant
{
public:
	explicit AutoSimulant(RTT::PropertyBag const & properties);
	virtual ~AutoSimulant();

	virtual void simulate(flt timeStep);

	virtual void setPosition(Vec3 const & _pos) {
		Simulant::setPosition(_pos);
		model.pos(0) = _pos(0);
		model.pos(1) = _pos(1);
	}

	virtual void setOrientation(Quaternion const & _orientation) {
		Simulant::setOrientation(_orientation);
		model.orientation = _orientation;
	}

	virtual void setVelocity(Vec3 const & _velocity) {
		Simulant::setVelocity(_velocity);
		model.currSpeed = _velocity.norm();
	}

	virtual void setSteer(flt const & _steer) {
		Simulant::setSteer(_steer);
		model.currSteer = _steer;
	}


	flt steerWish;
	flt speedWish;
	flt brakeWish;
	int gearWish;
	std::string infos;

// 	TimedObstacleData_ptr timedObstacleData;

	TimedBaseObstacleBundle_ptr timedObstacleBundle;

	CarModel model;

};

} // namespace modules
} // namespace nav
} // namespace simulator
}
