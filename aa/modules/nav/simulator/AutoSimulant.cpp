#include "AutoSimulant.h"

#include <math/AutoMath.h>

#include "SimulatorEngine.h"
#include <modules/models/carstate/CarState.h>



using namespace aa::modules::nav::simulator;
using namespace modules::models::carstate;
using namespace ::math;

AutoSimulant::AutoSimulant(RTT::PropertyBag const & properties)
	: Simulant(1)
	, steerWish(0.0)
	, speedWish(0.0)
	, brakeWish(0.0)
	, gearWish(CarState::GEAR_DRIVE)
	, model(properties)
{
}

AutoSimulant::~AutoSimulant()
{
}

/**
 * Called by SimulatorEngine
 * Generates new information as a result of a new simulation step
 * @param timeStep
 */
void AutoSimulant::simulate(flt timeStep)
{
	assert(CarState::isValidGear(gearWish));
	model.mGear = gearWish;
// 	std::cout << "before: " << model.currSpeed << std::endl;
	model.control(speedWish - brakeWish, steerWish, timeStep);
// 	std::cout << "after: " << model.currSpeed << std::endl;
	setPosition(model.pos);
	setOrientation(model.orientation);
}

