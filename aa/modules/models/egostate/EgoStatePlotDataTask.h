/**
 * \file EgoStatePlotDataTask.h
 * \brief convert EgoState data to PlotView data
 * \author Tobias Langner
 */

#pragma once
#include <util/RtTaskContext.h>
#include <modules/models/egostate/EgoState.h>
#include <data/plot/SimplePlotDataBundle.h>

namespace aa
{
namespace modules
{
namespace models
{

namespace egostate
{

class EgoStatePlotDataTask : public util::RtTaskContext
{
public:
	EgoStatePlotDataTask(const std::string & name);
	virtual ~EgoStatePlotDataTask();
protected:
	RTT::InputPort<TimedEgoState> mEgoStateIn;
	RTT::OutputPort< ::data::plot::SimplePlotDataBundle> mPlotDataOut;

	bool startHook();
	void updateHook();
	void stopHook();

	bool mCorrectApplanixAcceleration;
	std::string mPrefix;
	TimeStamp mStart;
};

}

}

}

}


