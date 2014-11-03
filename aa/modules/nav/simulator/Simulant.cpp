#include "Simulant.h"

using namespace aa::modules::nav::simulator;
using namespace ::math;



Simulant::Simulant(int type)
	: immovable(false)
	, dirty(false)
	, typ(type)
	, id(0)
	, size(Vec3::Zero())
	, material(Vec4::Zero())
	, name("")
	, model("")
	, modelscale(Vec3::Zero())

	, mPos(0, 0, 0)
	, mOrientation(1, 0, 0, 0)
	, mVelocity(0, 0, 0)
	, mSteer(0)
	, mAcceleration(0, 0, 0)
	, mAuxDevicesData(AuxDevicesData())
{
}

Simulant::~Simulant()
{
}

/**
 * called by SimulatorEngine when next time step is going to be calculated
 * @param timeStep
 */
void Simulant::simulate(flt timeStep)
{
}
