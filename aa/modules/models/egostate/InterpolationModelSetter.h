#pragma once

#include <util/RtTaskContext.h>
#include <core/TimedData.h>
#include <modules/models/egostate/EgoState.h>

//#define DISPLAYINTERPOL
#ifdef DISPLAYINTERPOL
#include <modules/display/plotview/PlotDataBundle.h>
#endif

namespace aa
{
namespace modules
{
namespace models
{

namespace egostate
{

class InterpolationModelSetter
	: public util::RtTaskContext
{
public:
	explicit InterpolationModelSetter(std::string const & name);
	virtual ~InterpolationModelSetter();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

protected:

	RTT::InputPort< TimedEgoState > mEgoStateIn;
	int mBufferSize;
	int mOldBufferSize;

#ifdef DISPLAYINTERPOL
	RTT::OutputPort<modules::display::plotview::PlotDataBundle> mPlotDataOut;

	TimedEgoState mLastEgoState;
	bool mStep;
#endif

};

}


}

 // namespace egostate
}

 // namespace models
}

 // namespace modules
