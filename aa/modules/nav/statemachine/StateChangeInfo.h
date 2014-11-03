#pragma once

#include <sstream>
#include <vector>
#include <string>
#include <time.h>
#include "State.h"

namespace aa
{
namespace modules
{
namespace nav
{

namespace statemachine
{

class StateChangeInfo
{
public:

	StateChangeInfo(State oldState, State newState, std::vector<std::string> statePath, std::string initiator, std::string reason);
	StateChangeInfo(int oldSpeedControlMode, int oldSteerControlMode, int newSpeedControlMode, int newSteerControlMode, std::string controlChangeString, std::string initiator, std::string reason);
	~StateChangeInfo();

	StateChangeInfo(StateChangeInfo const & rhs);
	StateChangeInfo & operator=(StateChangeInfo const & rhs);

	std::string getTimeString() const;
	std::string getStatePathString() const;
	std::string getStateString() const;
	std::string getReasonString() const;
	std::string getControlChangeString() const;

	std::string toString() const;

	bool mIsControlModeChange;

	State mOldState;
	State mNewState;
	std::vector<std::string> mStatePath;

	int mOldSpeedControlMode;
	int mOldSteerControlMode;
	int mNewSpeedControlMode;
	int mNewSteerControlMode;
	std::string mControlChangeString;

	std::string mInitiator;
	std::string mReason;
	time_t mTime;
};

}

}

}

}

