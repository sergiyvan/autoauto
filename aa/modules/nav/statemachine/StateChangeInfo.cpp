#include "StateChangeInfo.h"

namespace aa
{
namespace modules
{
namespace nav
{

namespace statemachine
{

using namespace std;
using namespace boost;


StateChangeInfo::StateChangeInfo(State oldState, State newState, std::vector<std::string> statePath, std::string initiator, std::string reason)
	: mIsControlModeChange(false)
	, mOldState(oldState)
	, mNewState(newState)
	, mStatePath(statePath)
	, mInitiator(initiator)
	, mReason(reason)
{
	time(&mTime);
}

StateChangeInfo::StateChangeInfo(int oldSpeedControlMode, int oldSteerControlMode, int newSpeedControlMode, int newSteerControlMode, std::string controlChangeString, std::string initiator, std::string reason)
	: mIsControlModeChange(true)
	, mOldSpeedControlMode(oldSpeedControlMode)
	, mOldSteerControlMode(oldSteerControlMode)
	, mNewSpeedControlMode(newSpeedControlMode)
	, mNewSteerControlMode(newSteerControlMode)
	, mControlChangeString(controlChangeString)
	, mInitiator(initiator)
	, mReason(reason)
{
	time(&mTime);
}

StateChangeInfo::~StateChangeInfo()
{
}


StateChangeInfo::StateChangeInfo(StateChangeInfo const & rhs)
	: mIsControlModeChange(rhs.mIsControlModeChange)

	, mOldState(rhs.mOldState)
	, mNewState(rhs.mNewState)
	, mStatePath(rhs.mStatePath)

	, mOldSpeedControlMode(rhs.mOldSpeedControlMode)
	, mOldSteerControlMode(rhs.mOldSteerControlMode)
	, mNewSpeedControlMode(rhs.mNewSpeedControlMode)
	, mNewSteerControlMode(rhs.mNewSteerControlMode)
	, mControlChangeString(rhs.mControlChangeString)

	, mInitiator(rhs.mInitiator)
	, mReason(rhs.mReason)
	, mTime(rhs.mTime)
{

}

StateChangeInfo & StateChangeInfo::operator=(StateChangeInfo const & rhs)
{
	if (&rhs == this) {
		return *this;
	}

	mIsControlModeChange = rhs.mIsControlModeChange;

	mOldState = rhs.mOldState;
	mNewState = rhs.mNewState;
	mStatePath = rhs.mStatePath;

	mOldSpeedControlMode = rhs.mOldSpeedControlMode;
	mOldSteerControlMode = rhs.mOldSteerControlMode;
	mNewSpeedControlMode = rhs.mNewSpeedControlMode;
	mNewSteerControlMode = rhs.mNewSteerControlMode;
	mControlChangeString = rhs.mControlChangeString;

	mInitiator = rhs.mInitiator;
	mReason = rhs.mReason;
	mTime = rhs.mTime;


	return *this;
}

std::string StateChangeInfo::getTimeString() const
{
	ostringstream out;

	struct tm timeinfo;
	localtime_r(&mTime, &timeinfo);

	string h = timeinfo.tm_hour < 10 ? "0" : "";
	string m = timeinfo.tm_min < 10 ? "0" : "";
	string s = timeinfo.tm_sec < 10 ? "0" : "";

	out << "[" << h << timeinfo.tm_hour << ":" << m << timeinfo.tm_min << ":" << s << timeinfo.tm_sec << "]";

	return out.str();
}

std::string StateChangeInfo::getStatePathString() const
{
	if (mIsControlModeChange) {
		return "";
	}

	std::ostringstream out;

	for (int i = mStatePath.size() - 1; i >= 0; i--) {
		out << mStatePath[i] << " > ";
	}

	return out.str();
}

std::string StateChangeInfo::getStateString() const
{
	if (mIsControlModeChange) {
		return "";
	}

	return mNewState.name;
}

std::string StateChangeInfo::getReasonString() const
{
	int ns = mInitiator.find_last_of("::");
	std::ostringstream out;
	out << "[" << mInitiator.substr(ns + 1) << "] ";
	out << mReason;
	return out.str();
}

std::string StateChangeInfo::getControlChangeString() const
{
	if (!mIsControlModeChange) {
		return "";
	}

	return mControlChangeString;
}

std::string StateChangeInfo::toString() const
{
	std::ostringstream out;

	if (mIsControlModeChange) {
		out << getTimeString();
		out << "  ";
		out << getControlChangeString();
		out << "\t\t";
		out << "(" << getReasonString() << ")";
	}
	else {
		out << getTimeString();
		out << "  ";
		out << getStatePathString();
		out << getStateString();
		out << "\t\t";
		out << "(" << getReasonString() << ")";
	}

	return out.str();
}

}


}


}


}


