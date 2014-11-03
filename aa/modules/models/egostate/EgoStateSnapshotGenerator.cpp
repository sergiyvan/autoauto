#include "EgoStateSnapshotGenerator.h"
#include <modules/models/egostate/EgoStateModel.h>

#include <rtt/Logger.hpp>

#include <util/TaskContextFactory.h>

using namespace aa::modules::models::egostate;


REGISTERTASKCONTEXT(EgoStateSnapshotGenerator);

using namespace std;
using namespace RTT;
using namespace boost;

EgoStateSnapshotGenerator::EgoStateSnapshotGenerator(string const & name)
	: util::RtTaskContext(name)
	// Members
	// Properties
	// Ports
	, mEgoState("EgoState")
	, mEgoStateSetter("EgoStateSetter")
	, mModel(theEgoStateModel::instance())
{
	// Method Factory Interface.
	ports()->addPort(mEgoState);
	ports()->addPort(mEgoStateSetter);
}

EgoStateSnapshotGenerator::~EgoStateSnapshotGenerator()
{
	stop();
}

bool EgoStateSnapshotGenerator::startHook()
{
	return true;
}

void EgoStateSnapshotGenerator::updateHook()
{
// 	RTT::Logger::In("EgoStateSnapshotGenerator");
	/// FIXME: use a trigger port to determin the time for the snapshot
	/// get current egostate
	TimedEgoState egoState;
	mModel.getCurrentState(egoState);

	//check egostate consistency
	if (egoState.position().squaredNorm() > 1.6e9f) {
		RTT::Logger::log() << RTT::Logger::Warning << "EgoStateSnapshotGenerator: Egostate corrupted! - IGNORING !!!  " << egoState.position() << RTT::Logger::endl;
	}
	else {
		mEgoState.write(egoState);
	}

	::modules::models::egostate::EgoStateSetter egoStateSetter;
	mEgoStateSetter.write(egoStateSetter);
}

void EgoStateSnapshotGenerator::stopHook()
{
}
