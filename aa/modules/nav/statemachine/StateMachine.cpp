#include "StateMachine.h"

#include <rtt/Logger.hpp>
#include <util/TaskContextFactory.h>
#include <util/Rtti.h>

//#include <GL/gl.h>
//#include <GL/glut.h>
//#include <GL/glext.h>
//#include <gui/GlTools.h>


#define LOG_CONTROL_CHANGES		TRUE
#define LOG_HISTORY_SIZE		10


using namespace aa::modules::nav::statemachine;

REGISTERTASKCONTEXT(StateMachine);

using namespace std;
using namespace boost;
using namespace math;
using namespace util;
using RTT::Logger;

StateMachine::StateMachine(std::string const & name)
	: util::RtTaskContext(name)

	, mSpeedControlMode(CONTROL_CONTROLLER)
	, mSteerControlMode(CONTROL_CONTROLLER)

	, mSpeedManualOverride(false)
	, mSteerManualOverride(false)

	, mReplanWarning(false)
{
	//add states
	mStates[ContextRoot().id] = ContextRoot();

	mStates[ContextDrive().id] = ContextDrive();
	mStates[ContextBrake().id] = ContextBrake();
	mStates[ContextStop().id] = ContextStop();

	mStates[StateDrive().id] = StateDrive();
	mStates[StateLearnedDrive().id] = StateLearnedDrive();
	mStates[StateOperatorDrive().id] = StateOperatorDrive();
	mStates[StateFollow().id] = StateFollow();

	mStates[ContextControlledBrake().id] = ContextControlledBrake();
	mStates[StateControlledBrakeMalfunction().id] = StateControlledBrakeMalfunction();
	mStates[StateControlledBrakePause().id] = StateControlledBrakePause();
	mStates[StateControlledBrakeGoal().id] = StateControlledBrakeGoal();
	mStates[StateControlledBrakeEnd().id] = StateControlledBrakeEnd();
	mStates[StateControlledBrakeTraffigLight().id] = StateControlledBrakeTraffigLight();
	mStates[StateControlledBrakeStopSign().id] = StateControlledBrakeStopSign();
	mStates[StateControlledBrakeGiveWay().id] = StateControlledBrakeGiveWay();
	mStates[StateControlledBrakeReverse().id] = StateControlledBrakeReverse();
	mStates[StateControlledBrakeDecisionPoint().id] = StateControlledBrakeDecisionPoint();
	mStates[StateControlledBrakeObstacle().id] = StateControlledBrakeObstacle();
	mStates[StateReactiveBrake().id] = StateReactiveBrake();

	mStates[StateTimedStop().id] = StateTimedStop();
	mStates[StatePermanentStop().id] = StatePermanentStop();
	mStates[StateWaitForMission().id] = StateWaitForMission();
	mStates[StateWaitForOperator().id] = StateWaitForOperator();
	mStates[StateWaitForRoadToClear().id] = StateWaitForRoadToClear();

	//add transitions
	for (int i = 0; i < MAX_STATE_ID; i++) {
		for (int j = 0; j < MAX_STATE_ID; j++) {
			mTransitions[i][j] = true;
// 				mTransitions[i][j] = false;
		}
	}

// 		mTransitions[StateDrive().id][StateLearnedDrive().id] = true;
// 		mTransitions[StateDrive().id][StateControlledBrake().id] = true;
// 		mTransitions[StateDrive().id][StateTimedStop().id] = true;
// 		mTransitions[StateDrive().id][StatePermanentStop().id] = true;
//
// 		mTransitions[StateLearnedDrive().id][StateDrive().id] = true;
// 		mTransitions[StateLearnedDrive().id][StateControlledBrake().id] = true;
// 		mTransitions[StateLearnedDrive().id][StateTimedStop().id] = true;
// 		mTransitions[StateLearnedDrive().id][StatePermanentStop().id] = true;
//
// 		mTransitions[StateControlledBrake().id][StateTimedStop().id] = true;
// 		mTransitions[StateControlledBrake().id][StatePermanentStop().id] = true;
//
// 		mTransitions[StateTimedStop().id][StateDrive().id] = true;
// 		mTransitions[StatePermanentStop().id][StateDrive().id] = true;



	//default duration in seconds
	mDurationTimedFullStop = 5.0;
	mDurationOperatorDrive = 20.0;

	mCurStateId = CONTEXT_ROOT;



	//create methods
	addOperation("updateStates", &StateMachine::updateStates, this, RTT::ClientThread).doc("updateStates");
	addOperation("manualEnterState", &StateMachine::manualEnterState, this, RTT::ClientThread).doc("manualEnterState").arg("stateName", "stateName").arg("reason", "reason");
	addOperation("enterState", &StateMachine::enterState, this, RTT::ClientThread).doc("enterState").arg("state", "state").arg("initiator", "initiator").arg("reason", "reason");
	addOperation("enterStatef", &StateMachine::enterStatef, this, RTT::ClientThread).doc("enterStatef").arg("state", "state").arg("initiator", "initiator").arg("reason", "reason");
	addOperation("popLastState", &StateMachine::popLastState, this, RTT::ClientThread).doc("popLastState").arg("initiator", "initiator").arg("reason", "reason");
	addOperation("isInState", &StateMachine::isInState, this, RTT::ClientThread).doc("isInState").arg("refState", "refState");
	addOperation("isSubStateOf", &StateMachine::isSubStateOf, this, RTT::ClientThread).doc("isSubStateOf").arg("stateToBeChecked", "stateToBeChecked").arg("refState", "refState");
	addOperation("printState", &StateMachine::printState, this, RTT::ClientThread).doc("printState");

	addOperation("getStateHistory", &StateMachine::getStateHistory, this, RTT::ClientThread).doc("getStateHistory");
	addOperation("getLogHistory", &StateMachine::getLogHistory, this, RTT::ClientThread).doc("getLogHistory");

	addOperation("getCurrentStateId", &StateMachine::getCurrentStateId, this, RTT::ClientThread).doc("getCurrentStateId");
	addOperation("getCurrentState", &StateMachine::getCurrentState, this, RTT::ClientThread).doc("getCurrentState");
	addOperation("getCurrentStateName", &StateMachine::getCurrentStateName, this, RTT::ClientThread).doc("getCurrentStateName");

	addOperation("getState", &StateMachine::getState, this, RTT::ClientThread).doc("getState").arg("state_id", "state_id");
	addOperation("getStateId", &StateMachine::getStateId, this, RTT::ClientThread).doc("getStateId").arg("state_name", "state_name");
	addOperation("getStateName", &StateMachine::getStateName, this, RTT::ClientThread).doc("getStateName").arg("state_id", "state_id");
	addOperation("getStatePath", &StateMachine::getStatePath, this, RTT::ClientThread).doc("getStatePath").arg("state", "state");

	addOperation("setControlMode", &StateMachine::setControlMode, this, RTT::ClientThread).doc("setControlMode").arg("controlMode", "controlMode").arg("value", "value").arg("initiator", "initiator").arg("reason", "reason");
	addOperation("getControlMode", &StateMachine::getControlMode, this, RTT::ClientThread).doc("getControlMode").arg("controlMode", "controlMode");
	addOperation("getControlModeString", &StateMachine::getControlModeString, this, RTT::ClientThread).doc("getControlModeString").arg("controlMode", "controlMode");
	addOperation("printControlMode", &StateMachine::printControlMode, this, RTT::ClientThread).doc("printControlMode");
	addOperation("enableManualOverride", &StateMachine::enableManualOverride, this, RTT::ClientThread).doc("enableManualOverride").arg("controlMode", "controlMode").arg("initiator", "initiator").arg("reason", "reason");
	addOperation("disableManualOverride", &StateMachine::disableManualOverride, this, RTT::ClientThread).doc("disableManualOverride").arg("controlMode", "controlMode").arg("initiator", "initiator").arg("reason", "reason");

	addOperation("setOperatorDecision", &StateMachine::setOperatorDecision, this, RTT::ClientThread).doc("setOperatorDecision").arg("state_id", "state_id").arg("w", "w");
	addOperation("getOperatorDecision", &StateMachine::getOperatorDecision, this, RTT::ClientThread).doc("getOperatorDecision").arg("state_id", "state_id");

	addOperation("setReplanWarning", &StateMachine::setReplanWarning, this, RTT::ClientThread).doc("setReplanWarning").arg("w", "w");
	addOperation("getReplanWarning", &StateMachine::getReplanWarning, this, RTT::ClientThread).doc("getReplanWarning");

}

StateMachine::~StateMachine()
{
}

bool StateMachine::startHook()
{
	Logger::In in("StateMachine");

	enterState(STATE_PERMANENTSTOP, "StateMachine", "State machine initiated");		//default initial state

	return true;
}

void StateMachine::updateHook()
{
	QMutexLocker lock(&mMutex);
	Logger::In in("StateMachine");

//	updateStates();
}

void StateMachine::stopHook()
{
}

void StateMachine::errorHook()
{
}

void StateMachine::updateStates()
{
	TimeStamp now;
	now.stamp();
	flt duration = 1E-9f * RTT::os::TimeService::ticks2nsecs(now - mCurStateStartTime);

	if (isInState(STATE_TIMEDSTOP) && duration >  mDurationTimedFullStop) {
// 		enterStatef(STATE_DRIVE, rtti::typeName(typeid(*this)), format("Timer for TimedStop has expired (%1% s)") % mDurationTimedFullStop);
		enterStatef(STATE_WAITFORROADTOCLEAR, rtti::typeName(typeid(*this)), format("Timer for TimedStop has expired (%1% s)") % mDurationTimedFullStop);
	}
	else if (isInState(STATE_OPERATORDRIVE) && duration >  mDurationOperatorDrive) {
		enterStatef(STATE_DRIVE, rtti::typeName(typeid(*this)), format("Timer for OperatorDrive has expired (%1% s)") % mDurationOperatorDrive);
	}

	if (mOperatorDecision != "" && duration > mDurationOperatorDrive) {
		mOperatorDecision = "";
	}

}



void StateMachine::manualEnterState(std::string stateName, std::string reason)
{
	RTT::Logger::In in("StateMachine");

	int stateId = getStateId(stateName);

	if (stateId > 0)  {
		enterState(stateId, rtti::typeName(typeid(*this)), "resuming drive by manual method call");
	}
	else {
		logWarning() << "state not recognized: " << stateName;
	}
}

void StateMachine::enterStatef(int state, std::string initiator, boost::format reason)
{
	enterState(state, initiator, reason.str());
}

void StateMachine::enterState(int state, std::string initiator, std::string reason)
{
	RTT::Logger::In in("StateMachine");

	assert(state >= 0 && state < MAX_STATE_ID);

	State oldState = getCurrentState();

	if (state == oldState.id) {
		//already in this state
// 		std::cout<<"already in " << mStates[state].name;
		mMutex.lock();

		if (mLogHistory.empty()) {
			return;
		}

		mLogHistory.back().mReason = reason;
		mMutex.unlock();
		return;
	}

	State tmp = mStates[state];

	if (tmp.id == UNKNOWN_STATE) {
		return;
	}

	if (mStateHistory.empty() || mTransitions[oldState.id][state]) {
		//curState = state;
		//go to leaf, update leaf
		while (tmp.last_child_id != UNKNOWN_STATE) {
			tmp = mStates[tmp.last_child_id];
		}

		mCurStateId = tmp.id;
		assert(mCurStateId >= 0 && mCurStateId < MAX_STATE_ID);

		//go to root, update path to root
		while (tmp.parent_id != UNKNOWN_STATE) {
			int cid = tmp.id;
			tmp = mStates[tmp.parent_id];
			mStates[tmp.id].last_child_id = cid;
		}

		//update state history
		if (mStateHistory.empty() || mStateHistory.back().id != mCurStateId) {
			mStateHistory.push_back(mStates[mCurStateId]);
		}

		//update log history
		mMutex.lock();
		State newState = mStateHistory.back();
		std::vector<std::string> statePath = getStatePath(newState);
		StateChangeInfo sci = StateChangeInfo(oldState, newState, statePath, initiator, reason);
		mLogHistory.push_back(sci);

		while (mLogHistory.size() > LOG_HISTORY_SIZE) {
			mLogHistory.pop_front();
		}

		mMutex.unlock();

		mCurStateStartTime.stamp();
	}

	logInfo() << "Enter state " << mStateHistory.back().name;
// 	std::cout<<"Enter state " << mStateHistory.back().name;
	assert(mCurStateId == mStateHistory.back().id);
}

void StateMachine::popLastState(std::string initiator, std::string reason)
{
	RTT::Logger::In in("StateMachine");

	if (mStateHistory.size() >= 2) {
		mStateHistory.pop_back();
		enterState(mStateHistory.back().id, initiator, reason);
	}
	else {
		logWarning() << "Cannot pop last state, because else stack will be empty";
	}
}

bool StateMachine::isInState(int refState) const
{
	return isSubStateOf(mCurStateId, refState);
}

bool StateMachine::isSubStateOf(int stateToBeChecked, int refState) const
{
	int test = stateToBeChecked;

	if (test == refState) {
		return true;
	}

	while (test > 0) {
		test = mStates[test].parent_id;

		if (test == refState) {
			return true;
		}
	}

	return false;
}

void StateMachine::printState() const
{
	std::cout << getCurrentStateId() << " " << getCurrentStateName();
}

StateHistoryType StateMachine::getStateHistory() const
{
	return mStateHistory;
}

LogHistoryType StateMachine::getLogHistory() const
{
	mMutex.lock();
	mLogHistoryCopy = mLogHistory;
	mMutex.unlock();
	return mLogHistoryCopy;
}

int StateMachine::getCurrentStateId() const
{
	return mStates[mCurStateId].id;
}

State StateMachine::getCurrentState() const
{
	return mStates[mCurStateId];
}

std::string StateMachine::getCurrentStateName() const
{
	return getStateName(getCurrentStateId());
}

State StateMachine::getState(int state_id) const
{
	assert(state_id >= 0 && state_id < MAX_STATE_ID);
	return mStates[state_id];
}

int StateMachine::getStateId(std::string state_name) const
{
	for (int i = 0; i < MAX_STATE_ID; i++) {
		if (mStates[i].name == state_name) {
			return mStates[i].id;
		}
	}

	return -1;
}

std::string StateMachine::getStateName(int state_id) const
{
	if (state_id < 0 || state_id >= MAX_STATE_ID) {
		return "STATE_UNKOWN";
	}

	State s = mStates[state_id];

	return s.name;
}


std::vector<std::string> StateMachine::getStatePath(State state)
{
	std::vector<std::string> result;

	State s = state;

	while (s.parent_id != UNKNOWN_STATE) {
		s = getState(s.parent_id);

		if (s.id == CONTEXT_ROOT) {
			break;
		}

		result.push_back(getStateName(s.id));
	}

	return result;
}


void StateMachine::setControlMode(int controlMode, int value, std::string initiator, std::string reason)
{
	assert(value == CONTROL_CONTROLLER ||
		   value == CONTROL_JOYSTICK ||
		   //value == CONTROL_UDPCONTROL ||
		   value == CONTROL_EYECONTROL ||
		   value == CONTROL_REMOTECONTROL ||
		   value == CONTROL_BRAINCONTROL
		  );

	bool oldSpeedManualOverride = mSpeedManualOverride;
	int oldSpeedControlMode = mSpeedControlMode;

	if (controlMode & CONTROL_MODE_SPEED) {
		mSpeedManualOverride = false;
		mSpeedControlMode = value;
	}

	bool oldSteerManualOverride = mSteerManualOverride;
	int oldSteerControlMode = mSteerControlMode;

	if (controlMode & CONTROL_MODE_STEER) {
		mSteerManualOverride = false;
		mSteerControlMode = value;
	}

	if (LOG_CONTROL_CHANGES && ((oldSpeedManualOverride != mSpeedManualOverride) || (oldSteerManualOverride != mSteerManualOverride)
								|| (oldSpeedControlMode != mSpeedControlMode) || (oldSteerControlMode != mSteerControlMode))) {

		mMutex.lock();
		int oldSpeedMode = (oldSpeedManualOverride ? CONTROL_MANUAL : oldSpeedControlMode);
		int oldSteerMode = (oldSteerManualOverride ? CONTROL_MANUAL : oldSteerControlMode);

		std::ostringstream out;
		//	out << "switched control mode from <" << theStateMachine::instance().getControlModeString(mOldSpeedControlMode);
		out << "<" << getControlModeString(oldSpeedMode);
		out << " , " << getControlModeString(oldSteerMode);
		out << "> to <" << getControlModeString(mSpeedControlMode);
		out << " , " << getControlModeString(mSteerControlMode) << ">";

		mLogHistory.push_back(StateChangeInfo(oldSpeedMode, oldSteerMode, mSpeedControlMode, mSteerControlMode, out.str(), initiator, reason));
		mMutex.unlock();
	}
}

int StateMachine::getControlMode(int controlMode) const
{
	if (controlMode == CONTROL_MODE_SPEED) {
		return mSpeedManualOverride ? CONTROL_MANUAL : mSpeedControlMode;
	}
	else if (controlMode == CONTROL_MODE_STEER) {
		return mSteerManualOverride ? CONTROL_MANUAL : mSteerControlMode;
	}

	return CONTROL_UNKNOWN;
}

std::string StateMachine::getControlModeString(int value) const
{
	switch (value) {
	case CONTROL_MANUAL:
		return "MANUAL";

	case CONTROL_CONTROLLER:
		return "CONTROLLER";

	case CONTROL_JOYSTICK:
		return "JOYSTICK";

		//case CONTROL_UDPCONTROL: return "UDPCONTROL";
	case CONTROL_EYECONTROL:
		return "EYECONTROL";

	case CONTROL_REMOTECONTROL:
		return "REMOTECONTROL";

	default:
		return "UNKNOWN";
	}
}

void StateMachine::printControlMode() const
{
	std::cout << "ControlSpeed: " << getControlModeString(getControlMode(CONTROL_MODE_SPEED)) << "\tControlSteer: " << getControlModeString(getControlMode(CONTROL_MODE_STEER));
}

bool StateMachine::enableManualOverride(int controlMode, std::string initiator, std::string reason)
{
	bool oldSpeedManualOverride = mSpeedManualOverride;

	if (controlMode & CONTROL_MODE_SPEED) {
		mSpeedManualOverride = true;
	}

	bool oldSteerManualOverride = mSteerManualOverride;

	if (controlMode & CONTROL_MODE_STEER) {
		mSteerManualOverride = true;
	}

	bool changed = (oldSpeedManualOverride != mSpeedManualOverride) || (oldSteerManualOverride != mSteerManualOverride);

	if (LOG_CONTROL_CHANGES && changed) {

		mMutex.lock();
		int newSpeedMode = (mSpeedManualOverride ? CONTROL_MANUAL : mSpeedControlMode);
		int newSteerMode = (mSteerManualOverride ? CONTROL_MANUAL : mSteerControlMode);

		std::ostringstream out;
		//	out << "switched control mode from <" << theStateMachine::instance().getControlModeString(mOldSpeedControlMode);
		out << "<" << getControlModeString(mSpeedControlMode);
		out << " , " << getControlModeString(mSteerControlMode);
		out << "> to <" << getControlModeString(newSpeedMode);
		out << " , " << getControlModeString(newSteerMode) << ">";

		mLogHistory.push_back(StateChangeInfo(mSpeedControlMode, mSteerControlMode, newSpeedMode, newSteerMode, out.str(), initiator, reason));
		mMutex.unlock();
	}

	return changed;
}

bool StateMachine::disableManualOverride(int controlMode, std::string initiator, std::string reason)
{
	bool oldSpeedManualOverride = mSpeedManualOverride;

	if (controlMode & CONTROL_MODE_SPEED) {
		mSpeedManualOverride = false;
	}

	bool oldSteerManualOverride = mSteerManualOverride;

	if (controlMode & CONTROL_MODE_STEER) {
		mSteerManualOverride = false;
	}

	bool changed = (oldSpeedManualOverride != mSpeedManualOverride) || (oldSteerManualOverride != mSteerManualOverride);

	if (LOG_CONTROL_CHANGES && changed) {

		mMutex.lock();
		int oldSpeedMode = (oldSpeedManualOverride ? CONTROL_MANUAL : mSpeedControlMode);
		int oldSteerMode = (oldSteerManualOverride ? CONTROL_MANUAL : mSteerControlMode);

		std::ostringstream out;
		//	out << "switched control mode from <" << theStateMachine::instance().getControlModeString(mOldSpeedControlMode);
		out << "<" << getControlModeString(oldSpeedMode);
		out << " , " << getControlModeString(oldSteerMode);
		out << "> to <" << getControlModeString(mSpeedControlMode);
		out << " , " << getControlModeString(mSteerControlMode) << ">";

		mLogHistory.push_back(StateChangeInfo(oldSpeedMode, oldSteerMode, mSpeedControlMode, mSteerControlMode, out.str(), initiator, reason));
		mMutex.unlock();
	}

	return changed;
}


void StateMachine::setOperatorDecision(std::string d)
{
	mOperatorDecision = d;
	/*
	assert(state_id >= 0 && state_id < MAX_STATE_ID);

	if(isInState(state_id)) {
		mStates[state_id].setMeta(d);
	}
	*/
}

std::string StateMachine::getOperatorDecision()
{
	return mOperatorDecision;

	/*
	std::string result;
	boost::any meta = mStates[state_id].getMeta();

	try {
		result = any_cast<string>(meta);
	}
	catch (const boost::bad_any_cast &) {
		result = "";
	}

	return result;
	*/
}


void StateMachine::setReplanWarning(bool w)
{
	mReplanWarning = w;
}

bool StateMachine::getReplanWarning()
{
	return mReplanWarning;
}
