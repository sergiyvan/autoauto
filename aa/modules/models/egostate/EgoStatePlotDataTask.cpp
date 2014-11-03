/**
 * \file EgoStateFilter.cpp
 * \author Georg Bremer
 */

#include "EgoStatePlotDataTask.h"
#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>

namespace aa
{
namespace modules
{
namespace models
{

namespace egostate
{

REGISTERTASKCONTEXT(EgoStatePlotDataTask);

EgoStatePlotDataTask::EgoStatePlotDataTask(const std::string & name)
	: util::RtTaskContext(name)
	, mEgoStateIn("EgoStateIn")
	, mPlotDataOut("PlotDataOut")
	, mCorrectApplanixAcceleration(false)
	, mPrefix("")
{
	addEventPort(mEgoStateIn);
	addPort(mPlotDataOut);

	addProperty("CorrectApplanixAcceleration",mCorrectApplanixAcceleration).doc("correctedAcc = Vec3(applanixAcc[1], -applanixAcc[0], applanixAcc[2])");
	addProperty("Prefix", mPrefix).doc("prefix for signal names");
}

EgoStatePlotDataTask::~EgoStatePlotDataTask()
{
}

bool EgoStatePlotDataTask::startHook()
{
	REQUIRED_PORT(mEgoStateIn);
	return true;
}

void EgoStatePlotDataTask::stopHook()
{
}

void EgoStatePlotDataTask::updateHook()
{
	if (mPlotDataOut.connected()) {
		TimedEgoState egostate;
		mEgoStateIn.read(egostate);
		math::Vec3 angularRates = egostate.angularRates();
		math::Vec3 velocity = egostate.localToGlobal().rotation().transpose() * egostate.velocity();
		math::Vec3 acceleration = egostate.acceleration();
		if (mCorrectApplanixAcceleration) {
			acceleration = math::Vec3(acceleration[1], -acceleration[0], acceleration[2]);
		}

		if (mStart - TimeStamp() == 0) {
			mStart = egostate;
		}

		::data::plot::SimplePlotDataBundle bundle(getName(), (egostate - mStart)*1e-9);

		bundle.addNamedValue(mPrefix + "omegax", angularRates[0]);
		bundle.addNamedValue(mPrefix + "omegay", angularRates[1]);
		bundle.addNamedValue(mPrefix + "omegaz", angularRates[2]);


		bundle.addNamedValue(mPrefix + "velx", velocity[0]);
		bundle.addNamedValue(mPrefix + "vely", velocity[1]);
		bundle.addNamedValue(mPrefix + "velz", velocity[2]);


		bundle.addNamedValue(mPrefix + "accx", acceleration[0]);
		bundle.addNamedValue(mPrefix + "accy", acceleration[1]);
		bundle.addNamedValue(mPrefix + "accz", acceleration[2]);


		mPlotDataOut.write(bundle);
	}
}

}


}


}


}


