#pragma once

#include <core/TimedData.h>
#include <iostream>
#include <deque>
#include <boost/format.hpp>
#include <gui/Painter3DTask.h>
#include <QMutex>
#include <math/Types.h>


#include <aa/modules/nav/statemachine/State.h>
#include <aa/modules/nav/statemachine/StateChangeInfo.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace statemachine
{

typedef std::deque< nav::statemachine::State > StateHistoryType;
typedef std::deque< nav::statemachine::StateChangeInfo > LogHistoryType;


class DisplayStateMachine
	: public gui::Painter3DTask
{
public:
	typedef ::math::flt flt;
	explicit DisplayStateMachine(std::string const & name = "StateMachine");
	virtual ~DisplayStateMachine();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	virtual void init3D(SceneNodePtr sceneNode);
	virtual void draw3D(DrawArg);

protected:
	/** @name Properties: general properties */
	RTT::Property<int> mMaxStateHistorySize;

	RTT::TaskContext * mStateMachine;

private:
	void renderReplanWarning(flt left, flt right, flt top, flt bottom) const;
	void renderStateMachineInfo(flt left, flt right, flt top, flt bottom) const;

	std::string mReplanNowKeyBinding;

	QMutex mMutex;


};

}
}
}
}
