#include "Controller.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <math/AutoMath.h>
#include <math/PathSpline.h>

#include <aa/modules/nav/simulator/Simulator.h>

namespace aa
{
namespace modules
{
namespace nav
{
namespace ro14controller
{

REGISTERTASKCONTEXT(Controller);


using namespace std;
using namespace RTT;
using namespace boost;
using namespace ::math;
using namespace ::util;
using RTT::Logger;



Controller::Controller(string const & name)
	: ::util::RtTaskContext(name)
	// read ports
	, mPlanIn("PlanIn")
	, mEgoStateIn("EgoStateIn")

	// write ports
	, mSteerOut("SteerOut")
	, mSpeedOut("SpeedOut")
	, mGearOut("GearOut")
	, mControllerDataOut("ControllerDataOut")

	// general properties
    , mWantedSpeed(100 * KMH_2_MS)
    , mConstantThrottle(0.25)
    , mConstantSteer(0.0)
	, esum(0.0)

    //attributes
    , mCounter(0)
{
	ports()->addPort(mPlanIn);
	ports()->addPort(mEgoStateIn);

	ports()->addPort(mSteerOut);
	ports()->addPort(mSpeedOut);
	ports()->addPort(mGearOut);
	ports()->addPort(mControllerDataOut);

    addProperty("WantedSpeed", mWantedSpeed).doc("wanted speed in m/s");
    addProperty("ConstantThrottle", mConstantThrottle).doc("wanted constant throttle to be maintained");
    addProperty("ConstantSteer", mConstantSteer).doc("constant steer value");

    addAttribute("Counter", mCounter);

    addOperation("ResetSimulator", &Controller::resetSimulator, this, RTT::ClientThread).doc("call reset from simulator task");


}


Controller::~Controller()
{}


bool Controller::startHook()
{
	Logger::In in("Controller");

	REQUIRED_PORT(mPlanIn);
	REQUIRED_PORT(mEgoStateIn);

	REQUIRED_PORT(mSteerOut);
	REQUIRED_PORT(mSpeedOut);
    REQUIRED_PORT(mGearOut);
	OPTIONAL_PORT(mControllerDataOut);

    mStartTime.stamp();

    string filename = "controller.csv";
    mOutputStream.open(filename.c_str());

    return true;
}

void Controller::updateHook()
{
	Logger::In in("Controller");


    ///read data
    // read egostate
	mEgoStateIn.read(mCurEgoState);

	if (mCurEgoState == TimeStamp()) {
        Logger::log() << Logger::Warning << "Invalid or no EgoState";
        return;
	}

    // read plan
    mPlanIn.read(mCurPlan);

    if (!mCurPlan) {
        Logger::log() << Logger::Warning << "Got no plan";
        return;
    }


    //update counter
    mCounter++;


    //debug messages
    Logger::log() << Logger::Debug << "=============================" << Logger::endl;

    now.stamp();
    flt timeSinceStart = 1E-9f * RTT::os::TimeService::ticks2nsecs(now - mStartTime);
    Logger::log() << Logger::Debug << "time since start (s):" << timeSinceStart << Logger::endl;

    flt curSpeed = mCurEgoState.vehicleSpeed();
    Logger::log() << Logger::Debug << "car speed:" << curSpeed << Logger::endl;

    Vec3 curPosition = mCurEgoState.position();
    Logger::log() << Logger::Debug << "car position:" << curPosition.transpose() << Logger::endl;

    Vec3 curOrientation = mCurEgoState.forwardDirection();
    Logger::log() << Logger::Debug << "car orientation:" << curOrientation.transpose() << Logger::endl;

    flt closestSqrDistToPlan;
    flt closestParamOnPlan;
    std::pair<flt, flt> const dom((*mCurPlan).domain());
    tie(closestSqrDistToPlan, closestParamOnPlan) = findClosestPoint(*mCurPlan, dom.first, dom.second, (dom.first + dom.second) / 2.0, mCurEgoState.position());
    Logger::log() << Logger::Debug << "dist to closest point on plan:" << sqrt(closestSqrDistToPlan) << Logger::endl;
    Logger::log() << Logger::Debug << "param of closest point on plan:" << closestParamOnPlan << Logger::endl;

    Vec3 closestPointOnPlan = (*mCurPlan)(closestParamOnPlan);
    Logger::log() << Logger::Debug << "closest point on plan:" << closestPointOnPlan.transpose() << Logger::endl;

    Vec3 closestPointOnPlanDir = mCurPlan->firstDerivative(closestParamOnPlan);
    Logger::log() << Logger::Debug << "closest point direction:" << closestPointOnPlanDir.transpose() << Logger::endl;


    //calculate throttle and steering values
    TimedFlt speedCorrection;
    speedCorrection.data = getThrottleBrakePosition(mCurControllerData.curSpeed, mWantedSpeed);
    speedCorrection.stamp();

    flt steerCorrection;
    steerCorrection = getSteeringPosition(mCurPlan);


    //create ControllerData for display module
    mCurControllerData = aa::modules::nav::controller::data::TimedControllerData(now, aa::modules::nav::controller::data::ControllerData(mCurEgoState, ::modules::models::carstate::TimedCarState(), mCurPlan, closestParamOnPlan, closestSqrDistToPlan, NAN, NAN, NAN, NAN, NAN, ""));
    mCurControllerData.wantedSpeed = mWantedSpeed;
    mCurControllerData.speedCorrection = speedCorrection.data;
    mCurControllerData.steerCorrection = steerCorrection;
    mCurControllerData.stamp();


    ///write to OutputPorts
    // write out steer and speed correction
    mSpeedOut.write(speedCorrection);
    mSteerOut.write(steerCorrection);

    //set gear to drive
    mGearOut.write(8);

    // write out controller data
	mControllerDataOut.write(mCurControllerData);


    ///write to file
    mOutputStream << timeSinceStart << "," << curSpeed << std::endl;


}

void Controller::stopHook()
{
    Logger::In in("Controller");

    mOutputStream.close();
}

flt Controller::getThrottleBrakePosition(flt curSpeed, flt wantedSpeed)
{
	Logger::In in("Controller");
	flt K_p = 3.;
	flt K_i = 1.;
	flt result = 0.;
	flt e = wantedSpeed - curSpeed;
	esum = esum + e;
   //insert your code here
	result = K_p* e + K_i * esum;
	if(result > 1.){
		result= 1;
	}
	if(result< -1.){
		result =-1.;
	}
    return result;
}



flt Controller::getSteeringPosition(aa::modules::nav::controller::Plan_ptr plan)
{
    Logger::In in("Controller");

    //insert your code here

    return mConstantSteer;
}

void Controller::resetSimulator()
{
    Logger::In in("Controller");

    RTT::TaskContext* simulator = findPeer(this, "Simulator");
    OperationCaller<void(void)> reset(simulator->getOperation("reset"));

    reset();

}






}
}
}
}
