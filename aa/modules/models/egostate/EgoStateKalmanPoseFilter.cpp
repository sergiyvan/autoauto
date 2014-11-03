/*!
 * \file "EgostateKalmanPoseFilter.h"
 * \brief A module, which receives the Egostate (as wrapper for pose) and applys a kalman Filter. Returns the filtered egostate.
 * \author Jan F. Boldt
 */

#include "EgoStateKalmanPoseFilter.h"

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


REGISTERTASKCONTEXT2(EgoStateKalmanPoseFilter, "SequentialActivity");

//////////////////////////////////////////////////////
// Constructor:

EgoStateKalmanPoseFilter::EgoStateKalmanPoseFilter(std::string const & name)
	: util::RtTaskContext(name)
	// TODO constructor
	// Read ports:
	, mEgoStateIn("EgoStateIn")

	// Write ports:
	, mEgoStateOut("EgoStateOut")

	// Properties:

	// Attributes:
{
	// Read ports:
	// Event Ports:
	ports()->addEventPort(mEgoStateIn);
	// Non-event ports:

	// Write ports
	ports()->addPort(mEgoStateOut);

	// Properties:

	// Attributes:

	// Methods:

	// Commands:

}

//////////////////////////////////////////////////////
// Destructor:

EgoStateKalmanPoseFilter::~EgoStateKalmanPoseFilter()
{
	// TODO destructor
}

//////////////////////////////////////////////////////
// TaskContext interface

bool EgoStateKalmanPoseFilter::startHook()
{
	RTT::Logger::In in(this->getName());
	REQUIRED_PORT(mEgoStateIn);

	return TaskContext::startHook();
}

void EgoStateKalmanPoseFilter::updateHook()
{
	RTT::Logger::In in(this->getName());


	TimedEgoState currentEgoState;

	if (RTT::NewData != mEgoStateIn.read(currentEgoState)) {
		return;
	}


	// TODO Filter Implementieren!


	mEgoStateOut.write(currentEgoState);
}

void EgoStateKalmanPoseFilter::stopHook()
{
	RTT::Logger::In in(this->getName());
	// TODO stopHook
}

////////////////////////////////////////////////////////
// Protected functions:

////////////////////////////////////////////////////////
// Private functions:

}

}

}

}

