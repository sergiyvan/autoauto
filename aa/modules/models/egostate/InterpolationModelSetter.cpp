#include "InterpolationModelSetter.h"
#include <modules/models/egostate/InterpolationModel.h>

#include <util/TaskContextFactory.h>

namespace aa
{
namespace modules
{
namespace models
{

namespace egostate
{


REGISTERTASKCONTEXT(InterpolationModelSetter);

using namespace std;
using namespace RTT;
using namespace boost;

InterpolationModelSetter::InterpolationModelSetter(string const & name)
	: util::RtTaskContext(name)
	, mEgoStateIn("EgoStateIn")
#ifdef DISPLAYINTERPOL
	, mPlotDataOut("PlotDataOut", 16)
#endif
	, mBufferSize(100)
	, mOldBufferSize(100)
{
	ports()->addEventPort(mEgoStateIn);

	addProperty("BufferSize", mBufferSize).doc("the size of the EgoState buffer");

#ifdef DISPLAYINTERPOL
	ports()->addPort(mPlotDataOut);
#endif
}

InterpolationModelSetter::~InterpolationModelSetter()
{
}

bool InterpolationModelSetter::startHook()
{
	return true;
}

#ifdef DISPLAYINTERPOL
void addEgoData(modules::display::plotview::PlotDataBundle & bundle, TimedEgoState & ego, std::string const & text)
{
	bundle.addNamedValue(text + "pX", ego.position()[0]);
	//bundle.addNamedValue(text + "pY", ego.position()[1]);
	//bundle.addNamedValue(text + "pZ", ego.position()[2]);

	bundle.addNamedValue(text + "vX", ego.velocity()[0]);
	//bundle.addNamedValue(text + "vY", ego.velocity()[1]);
	//bundle.addNamedValue(text + "vZ", ego.velocity()[2]);

	bundle.addNamedValue(text + "aX", ego.acceleration()[0]);
	//bundle.addNamedValue(text + "aY", ego.acceleration()[1]);
	//bundle.addNamedValue(text + "aZ", ego.acceleration()[2]);

	bundle.addNamedValue(text + "rR", ego.rollPitchYaw()[0]);
	//bundle.addNamedValue(text + "rP", ego.rollPitchYaw()[1]);
	//bundle.addNamedValue(text + "rY", ego.rollPitchYaw()[2]);
}

void addEgoData(modules::display::plotview::PlotDataBundle & bundle, TimedEgoState & ego, std::string const & text, TimedEgoState & otherEgo)
{
	bundle.addNamedValue(text + " dp", (ego.position() - otherEgo.position()).norm());
	bundle.addNamedValue(text + " dv", (ego.velocity() - otherEgo.velocity()).norm());
	bundle.addNamedValue(text + " da", (ego.acceleration() - otherEgo.acceleration()).norm());
	math::flt dr = (ego.rollPitchYaw() - otherEgo.rollPitchYaw()).norm();

//	if (dr >= 360) {
//		std::cout << ego.rollPitchYaw() << ":" << otherEgo.rollPitchYaw() << std::endl;
//	}
	dr = std::fabs(dr);
	bundle.addNamedValue(text + " dr", dr);
}
#endif

void InterpolationModelSetter::updateHook()
{
	TimedEgoState ego;

	if (NewData != mEgoStateIn.read(ego)) {
		return;
	}

	if (mOldBufferSize != mBufferSize) {
		theInterpolationModel::instance().setBufferSize(mBufferSize);
		mOldBufferSize = mBufferSize;
	}


#ifndef DISPLAYINTERPOL
	theInterpolationModel::instance().setCurrentState(ego);
#else

	if (mStep) {
		mStep = false;
		InterpolationModel & interpol = theInterpolationModel::instance();
		TimedEgoState extrapolated_linear, extrapolated_cubic;
		TimedEgoState interpolated_linear, interpolated_cubic;
		interpol.getTimedState(extrapolated_linear, mLastEgoState, Linear);
		interpol.getTimedState(extrapolated_cubic, mLastEgoState, Slerp);
		theInterpolationModel::instance().setCurrentState(ego);
		interpol.getTimedState(interpolated_linear, mLastEgoState, Linear);
		interpol.getTimedState(interpolated_cubic, mLastEgoState, Slerp);

		modules::display::plotview::PlotDataBundle bundle("EgoStates");
		addEgoData(bundle, ego, "real", mLastEgoState);
		addEgoData(bundle, extrapolated_linear, "extra_linear", mLastEgoState);
		addEgoData(bundle, extrapolated_cubic, "extra_cubic", mLastEgoState);
		addEgoData(bundle, interpolated_linear, "inter_linear", mLastEgoState);
		addEgoData(bundle, interpolated_cubic, "inter_cubic", mLastEgoState);
		mPlotDataOut.write(bundle);
	}
	else {//interleave this one, so we can interpolate and compare it
		mLastEgoState = ego;
		mStep = true;
	}

#endif
}

void InterpolationModelSetter::stopHook()
{
}
}


}


}


}



