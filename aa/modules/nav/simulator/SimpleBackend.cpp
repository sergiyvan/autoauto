#include "SimpleBackend.h"
#include <data/VehicleData.h>
#include "AutoSimulant.h"

using namespace aa::modules::nav::simulator;

using namespace RTT;
using namespace boost;
using namespace std;
using namespace ::math;

SimpleBackend::SimpleBackend()
{

}

SimpleBackend::~SimpleBackend()
{

}

Simulant * SimpleBackend::getSimulant()
{
	return new Simulant(0);
}

AutoSimulant * SimpleBackend::getAutoSimulant()
{
    return new AutoSimulant(::data::theVehicleData::instance().rvalue());
}


void SimpleBackend::update(flt currentFrameTime)
{
}

void SimpleBackend::draw()
{}

std::string SimpleBackend::getName() const
{
	return "SimpleBackend";
}
