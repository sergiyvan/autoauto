#pragma once
/*!
 * \file EgostateKalmanPoseFilter.h
 * \brief A module, which receives the Egostate (as wrapper for pose) and applys a kalman Filter. Returns the filtered egostate.
 * \author Jan F. Boldt
 */

#include <util/RtTaskContext.h>

#include <modules/models/egostate/EgoState.h>

// Definitions and forward declarations


namespace aa
{
namespace modules
{
namespace models
{

namespace egostate
{

/*!
* \class EgostateKalmanPoseFilter
* \brief A module, which receives the Egostate (as wrapper for pose) and applys a kalman Filter. Returns the filtered egostate.
* \author Jan F. Boldt
*
* TODO Detailed description of EgostateKalmanPoseFilter
*/
class EgoStateKalmanPoseFilter
	: public util::RtTaskContext
{
public:
	// Definitions:

	// Constructors:
	explicit EgoStateKalmanPoseFilter(std::string const & name);

	// Destructor:
	virtual ~EgoStateKalmanPoseFilter();

protected:
	/** \name Ports: */
	/*! \{ */
	// Read ports:
	RTT::InputPort<TimedEgoState> mEgoStateIn;

	// Write ports:
	RTT::OutputPort<TimedEgoState> mEgoStateOut;

	/*! \} */
	/** \name Properties: */
	/*! \{ */

	/*! \} */


	// Functions:
	virtual bool startHook();

	virtual void updateHook();

	virtual void stopHook();

	// Other attributes:

private:
	// Attributes:

	// Functions:
};

}

}

}

}


