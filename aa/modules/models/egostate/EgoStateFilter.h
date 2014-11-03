/**
 * \file EgoStateFilter.h
 * \brief This file contains the module EgoStateFilter for smoothing the angles from the EgoState
 * \author Georg Bremer
 */

#pragma once
#include <util/RtTaskContext.h>
#include <modules/models/egostate/EgoState.h>
#include <boost/circular_buffer.hpp>

namespace aa
{
namespace modules
{
namespace models
{
namespace egostate
{

/**
 * \class EgoStateFilter
 * \brief This module is an average filter which at the moment only smoothes the angles of the EgoState.
 */
class EgoStateFilter : public util::RtTaskContext
{
public:
	EgoStateFilter(const std::string & name);
	virtual ~EgoStateFilter();
protected:
	RTT::InputPort<TimedEgoState> mEgoStateIn;
	RTT::OutputPort<TimedEgoState> mEgoStateOut;

	RTT::Property<unsigned int> mBufferSize;

	bool startHook();
	void updateHook();
	void stopHook();

private:
	boost::circular_buffer<TimedEgoState> mEgoStateBuffer;
	TimedEgoState mFilteredEgoState;
};

}
}
}
}

