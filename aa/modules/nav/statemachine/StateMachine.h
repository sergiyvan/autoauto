#pragma once

#include <util/RtTaskContext.h>
#include <core/TimedData.h>
#include <iostream>
#include <deque>
#include <boost/format.hpp>
#include <QMutex>

#include <math/Types.h>

#include "State.h"
#include "StateChangeInfo.h"


namespace aa
{
namespace modules
{
namespace nav
{

namespace statemachine
{


/**  ids for states */
#define UNKNOWN_STATE						-1				//undefined state

#define CONTEXT_ROOT						0				//root of state machine

#define CONTEXT_DRIVE						10				//context holding states for driving
#define STATE_DRIVE							11				//normal autonomous driving with a controller
#define STATE_LEARNEDDRIVE					12				//autonomous driving with a learned controller
#define STATE_OPERATORDRIVE					13				//autonomous driving after operator has made a choice for mDurationWaitForOperator seconds, this state is volatile
#define STATE_FOLLOW						14				//autonmous following of an object in front

#define CONTEXT_BRAKE						20				//context holding states for braking

#define CONTEXT_CONTROLLEDBRAKE				21				//context holding states for controlled braking
#define STATE_CONTROLLEDBRAKE_MALFUNCTION	22				//braking by the controller because malfunction has been encountered
#define STATE_CONTROLLEDBRAKE_PAUSE			23				//braking by the controller because autonomous run will be paused
#define STATE_CONTROLLEDBRAKE_GOAL			24				//braking by the controller because goal (last checkpoint) will be reached
#define STATE_CONTROLLEDBRAKE_END			25				//braking by the controller before end of spline
#define STATE_CONTROLLEDBRAKE_TRAFFICLIGHT	26				//braking by the controller before red traffic light
#define STATE_CONTROLLEDBRAKE_STOPSIGN		27				//braking by the controller before stop sign
#define STATE_CONTROLLEDBRAKE_GIVEWAY		28				//braking by the controller because we have to give way in crossing
#define STATE_CONTROLLEDBRAKE_REVERSE		29				//braking by the controller before switching to reverse/drive
#define STATE_CONTROLLEDBRAKE_DECISIONPOINT	30				//braking by the controller before decision point
#define STATE_CONTROLLEDBRAKE_OBSTACLE		31				//braking by the controller before obstacle

#define STATE_REACTIVEBRAKE					40				//hard or emergency braking by ReactiveBrake

#define CONTEXT_STOP						50				//context holding states for stopping
#define STATE_TIMEDSTOP						51				//stopping for mDurationTimedFullStop seconds, this state is volatile
#define STATE_PERMANENTSTOP					52				//permanent stop, operator has to leave this state manually
#define STATE_WAITFORMISSION				53				//stop and wait for new mission
#define STATE_WAITFOROPERATOR				54				//stop and wait for operator command
#define STATE_WAITFORROADTOCLEAR			55				//stop and wait for road to clear before continue driving

#define MAX_STATE_ID						60				//max state id, make sure this is bigger than all state ids


/** ids for control modes */
#define CONTROL_MODE_STEER					1
#define CONTROL_MODE_SPEED					2

/** ids for speed and steer control mode */
#define CONTROL_UNKNOWN						-1
#define CONTROL_MANUAL						0
#define CONTROL_CONTROLLER					1
#define CONTROL_JOYSTICK					2
#define CONTROL_REMOTECONTROL				3
#define CONTROL_EYECONTROL					4
#define CONTROL_BRAINCONTROL				5


typedef std::deque< State > StateHistoryType;
typedef std::deque< StateChangeInfo > LogHistoryType;

/** root */
struct UnknownState : State {
	UnknownState() : State(UNKNOWN_STATE, UNKNOWN_STATE, "UNKNOWN_STATE", UNKNOWN_STATE) {}
};

/** root */
struct ContextRoot : State {
	ContextRoot() : State(CONTEXT_ROOT, UNKNOWN_STATE, "STATE_MACHINE", CONTEXT_STOP) {}
};

/** context */
struct ContextDrive : State {
	ContextDrive() : State(CONTEXT_DRIVE, CONTEXT_ROOT, "C_DRIVE", STATE_DRIVE) {}
};
struct ContextBrake : State {
	ContextBrake() : State(CONTEXT_BRAKE, CONTEXT_ROOT, "C_BRAKE", CONTEXT_CONTROLLEDBRAKE) {}
};
struct ContextStop : State {
	ContextStop() : State(CONTEXT_STOP, CONTEXT_ROOT, "C_STOP", STATE_PERMANENTSTOP) {}
};
struct ContextControlledBrake : State {
	ContextControlledBrake() : State(CONTEXT_CONTROLLEDBRAKE, CONTEXT_BRAKE, "C_CONTROLLEDBRAKE", STATE_CONTROLLEDBRAKE_STOPSIGN) {}
};

/** context: drive */
struct StateDrive : State {
	StateDrive() : State(STATE_DRIVE, CONTEXT_DRIVE, "S_DRIVE", UNKNOWN_STATE) {}
};
struct StateLearnedDrive : State {
	StateLearnedDrive() : State(STATE_LEARNEDDRIVE, CONTEXT_DRIVE, "S_LEARNEDDRIVE", UNKNOWN_STATE) {}
};
struct StateOperatorDrive : State {
	StateOperatorDrive() : State(STATE_OPERATORDRIVE, CONTEXT_DRIVE, "S_OPERATORDRIVE", UNKNOWN_STATE) {}
};
struct StateFollow : State {
	StateFollow() : State(STATE_FOLLOW, CONTEXT_DRIVE, "S_FOLLOW", UNKNOWN_STATE) {}
};

/** context: controlled brake and brake */
struct StateControlledBrakeMalfunction : State {
	StateControlledBrakeMalfunction() : State(STATE_CONTROLLEDBRAKE_MALFUNCTION, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_MALFUNCTION", UNKNOWN_STATE) {}
};
struct StateControlledBrakePause : State {
	StateControlledBrakePause() : State(STATE_CONTROLLEDBRAKE_PAUSE, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_PAUSE", UNKNOWN_STATE) {}
};
struct StateControlledBrakeGoal : State {
	StateControlledBrakeGoal() : State(STATE_CONTROLLEDBRAKE_GOAL, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_GOAL", UNKNOWN_STATE) {}
};
struct StateControlledBrakeEnd : State {
	StateControlledBrakeEnd() : State(STATE_CONTROLLEDBRAKE_END, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_END", UNKNOWN_STATE) {}
};
struct StateControlledBrakeTraffigLight : State {
	StateControlledBrakeTraffigLight() : State(STATE_CONTROLLEDBRAKE_TRAFFICLIGHT, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_TRAFFICLIGHT", UNKNOWN_STATE) {}
};
struct StateControlledBrakeStopSign : State {
	StateControlledBrakeStopSign() : State(STATE_CONTROLLEDBRAKE_STOPSIGN, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_STOPSIGN", UNKNOWN_STATE) {}
};
struct StateControlledBrakeGiveWay : State {
	StateControlledBrakeGiveWay() : State(STATE_CONTROLLEDBRAKE_GIVEWAY, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_GIVEWAY", UNKNOWN_STATE) {}
};
struct StateControlledBrakeReverse : State {
	StateControlledBrakeReverse() : State(STATE_CONTROLLEDBRAKE_REVERSE, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_REVERSE", UNKNOWN_STATE) {}
};
struct StateControlledBrakeDecisionPoint : State {
	StateControlledBrakeDecisionPoint() : State(STATE_CONTROLLEDBRAKE_DECISIONPOINT, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_DECISIONPOINT", UNKNOWN_STATE) {}
};
struct StateControlledBrakeObstacle : State {
	StateControlledBrakeObstacle() : State(STATE_CONTROLLEDBRAKE_OBSTACLE, CONTEXT_CONTROLLEDBRAKE, "S_CONTROLLEDBRAKE_OBSTACLE", UNKNOWN_STATE) {}
};
struct StateReactiveBrake : State {
	StateReactiveBrake() : State(STATE_REACTIVEBRAKE, CONTEXT_BRAKE, "S_REACTIVEBRAKE", UNKNOWN_STATE) {}
};

/** context: stop */
struct StateTimedStop : State {
	StateTimedStop(): State(STATE_TIMEDSTOP, CONTEXT_STOP, "S_TIMEDSTOP", UNKNOWN_STATE) {}
};
struct StatePermanentStop : State {
	StatePermanentStop() : State(STATE_PERMANENTSTOP, CONTEXT_STOP, "S_PERMANENTSTOP", UNKNOWN_STATE) {}
};
struct StateWaitForMission : State {
	StateWaitForMission() : State(STATE_WAITFORMISSION, CONTEXT_STOP, "S_WAITFORMISSION", UNKNOWN_STATE) {}
};
struct StateWaitForOperator : State {
	StateWaitForOperator() : State(STATE_WAITFOROPERATOR, CONTEXT_STOP, "S_WAITFOROPERATOR", UNKNOWN_STATE) {}
};
struct StateWaitForRoadToClear : State {
	StateWaitForRoadToClear() : State(STATE_WAITFORROADTOCLEAR, CONTEXT_STOP, "S_WAITFORROADTOCLEAR", UNKNOWN_STATE) {}
};



class StateMachine
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;
	explicit StateMachine(std::string const & name = "StateMachine");
	virtual ~StateMachine();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void errorHook();

	void updateStates();

	void manualEnterState(std::string stateName, std::string reason);
	void enterState(int state, std::string initiator, std::string reason);
	void enterStatef(int state, std::string initiator, boost::format reason);
	void popLastState(std::string initiator, std::string reason);
	bool isInState(int refState) const;
	bool isSubStateOf(int stateToBeChecked, int refState) const;
	void printState() const;

	StateHistoryType getStateHistory() const;
	LogHistoryType getLogHistory() const;

	int getCurrentStateId() const;
	State getCurrentState() const;
	std::string getCurrentStateName() const;

	State getState(int state_id) const;
	int getStateId(std::string state_name) const;
	std::string getStateName(int state_id) const;
	std::vector<std::string> getStatePath(State state);

	void setControlMode(int controlMode, int value, std::string initiator, std::string reason);
	int getControlMode(int controlMode) const;
	std::string getControlModeString(int controlMode) const;
	void printControlMode() const;
	bool enableManualOverride(int controlMode, std::string initiator, std::string reason);
	bool disableManualOverride(int controlMode, std::string initiator, std::string reason);

	void setOperatorDecision(std::string d);
	std::string getOperatorDecision();

	void setReplanWarning(bool w);
	bool getReplanWarning();

protected:


private:
	mutable QMutex mMutex;

	State mStates[MAX_STATE_ID];
	bool mTransitions[MAX_STATE_ID][MAX_STATE_ID];
	TimeStamp mCurStateStartTime;

	int mCurStateId;
	StateHistoryType mStateHistory;
	LogHistoryType mLogHistory;
	mutable LogHistoryType mLogHistoryCopy;

	flt mDurationTimedFullStop;
	flt mDurationOperatorDrive;

	bool mSpeedManualOverride;
	bool mSteerManualOverride;
	int mSpeedControlMode;
	int mSteerControlMode;

	std::string mOperatorDecision;

	bool mReplanWarning;

};


}


}

 // namespace statemachine
}

 // namespace nav
}

 // namespace modules
