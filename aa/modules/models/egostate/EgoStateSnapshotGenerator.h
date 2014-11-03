#pragma once

#include <util/RtTaskContext.h>
#include <core/TimedData.h>

#include <modules/models/egostate/EgoState.h>
#include <modules/models/egostate/EgoStateSetter.h>
#include <modules/models/egostate/EgoStateModel.h>

namespace aa
{
namespace modules
{
namespace models
{

namespace egostate
{

/*!
	\class EgoStateSnapshotGenerator
 *   	\brief Generates a snapshot of the EgoState at a specific time.
 */


class EgoStateSnapshotGenerator
	: public util::RtTaskContext
{
public:
	explicit EgoStateSnapshotGenerator(std::string const & name);
	virtual ~EgoStateSnapshotGenerator();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	/** @name Registered Methods: */
	//@{

	//@}


protected:

	/** @name Properties: */
	//@{
	//@}

	/** @name OutputPorts: */
	//@{
	RTT::OutputPort< TimedEgoState > mEgoState;
	RTT::OutputPort< ::modules::models::egostate::EgoStateSetter > mEgoStateSetter;
	//@}

	::modules::models::egostate::EgoStateModel & mModel;

};

}

}
 // namespace egostate
}
 // namespace models
}
 // namespace modules
