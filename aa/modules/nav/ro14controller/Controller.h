#pragma once

#include <util/RtTaskContext.h>
#include <util/Ports.h>
#include <core/TimedData.h>
#include <iostream>
#include <deque>

#include <aa/modules/nav/controller/Plan.h>
#include <aa/modules/nav/controller/data/ControllerData.h>

#include <modules/models/egostate/EgoState.h>


namespace aa
{
namespace modules
{
namespace nav
{
namespace ro14controller
{

class Controller
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;
    typedef TimedData< PlainDataHolder< ::math::flt > > TimedFlt;
	typedef ::math::Vec3 Vec3;

    explicit Controller(std::string const & name);
    virtual ~Controller();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

private:

	/** @name OutputPorts: */
	//@{
	RTT::OutputPort< flt > mSteerOut;
	RTT::OutputPort< TimedFlt > mSpeedOut;
	RTT::OutputPort< int > mGearOut;
    RTT::OutputPort< aa::modules::nav::controller::data::TimedControllerData > mControllerDataOut;
	//@}

	/** @name InputPorts: */
	//@{
    RTT::InputPort< aa::modules::nav::controller::Plan_ptr > mPlanIn;
	RTT::InputPort< TimedEgoState > mEgoStateIn;
    //@}

	/** @name Attributes: general attributes */
    int mCounter;

    /** @name Properties */
    flt mWantedSpeed;
    flt mConstantThrottle;
    flt mConstantSteer;
    flt esum;

	/** members */
    aa::modules::nav::controller::data::TimedControllerData mCurControllerData;
    aa::modules::nav::controller::Plan_ptr mCurPlan;
	TimedEgoState mCurEgoState;

	TimeStamp now;
    TimeStamp mStartTime;

    std::ofstream mOutputStream;


	/** calculate a throttle position given current and wanted speed (uses either linear, pid controller)
	* @param curSpeed current speed of the car in m/s
	* @param wantedSpeed desired speed of the car in m/s
	* @return a value between -1 and 1 indicating gas/brake pedal position
	*/
	flt getThrottleBrakePosition(flt curSpeed, flt wantedSpeed);

	/** calculate a steering position
	* @return a value between -1 and 1 indicating steering position
	*/
    flt getSteeringPosition(aa::modules::nav::controller::Plan_ptr plan);

    /** reset simulator
    */
    void resetSimulator();


};

}
}
}
}
