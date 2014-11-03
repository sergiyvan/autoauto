#include "CarFollowingBehaviour.h"

#include <rtt/Logger.hpp>
#include <patterns/Singleton.h>

#include <util/TaskContextFactory.h>
#include <math/AutoMath.h>
#include <math/Geodetic.h>
#include <math/Rotate.h>

#include <math/PathSpline.h>
#include <util/OrocosHelperFunctions.h>

#include <aa/modules/nav/statemachine/StateMachine.h>


namespace aa
{
namespace modules
{
namespace nav
{

namespace behaviour
{

using namespace std;
using namespace RTT;
using namespace boost;
using namespace ::math;
using namespace util;
using namespace aa::modules::models::rndf;
using RTT::Logger;
using aa::modules::nav::controller::Plan;
using aa::modules::nav::controller::Plan_ptr;

REGISTERTASKCONTEXT(CarFollowingBehaviour);

CarFollowingBehaviour::CarFollowingBehaviour(string const & name)
	: RtTaskContext(name)

	, mEgoStateIn("EgoStateIn")
	, mPlanOut("PlanOut")

	//debug
    , mCounter(0)
	, mFilterDist(3.0)
	, mCheckDist(5.0)

{
	ports()->addPort(mEgoStateIn);
	ports()->addPort(mPlanOut);

    addAttribute("Counter", mCounter);

	addProperty("FilterDist", mFilterDist);
	addProperty("CheckDist", mCheckDist);

	addOperation("ReplanNow", &CarFollowingBehaviour::ReplanNow, this, RTT::ClientThread).doc("replan trajectories");

	mCurEgoState = TimedEgoState();
	mCurEgoState.stamp();
}

CarFollowingBehaviour::~CarFollowingBehaviour()
{
}


bool CarFollowingBehaviour::startHook()
{
	Logger::In in("CarFollowingBehaviour");

	mStateMachine = findPeer(this, "StateMachine");

	if (!mStateMachine) {
		logError() << "missing StateMachine peer";
		return false;
	}

    REQUIRED_PORTS((mEgoStateIn));

	OPTIONAL_PORT(mPlanOut);

	mGpsTrack.clear();

	return true;
}

void CarFollowingBehaviour::updateHook()
{
	Logger::In in("CarFollowingBehaviour");

    mCounter++;


	//get current egoState
	TimedEgoState curEgoState;
	mEgoStateIn.read(curEgoState);

	if (curEgoState == TimeStamp()) {
        if (mCounter % 100 == 0) {
			logWarning() << "Invalid or no EgoState";
		}

		return;
	}

	mCurEgoState = curEgoState;
    const Vec3 vehiclePos = mCurEgoState.position();
    const Vec3 vehicleDir = mCurEgoState.forwardDirection();

	if (mGpsTrack.empty()) {
        mGpsTrack.push_back(pair<Vec3, Vec3>(vehiclePos, vehicleDir));
	}
	else {
            mGpsTrack.push_back(pair<Vec3, Vec3>(mGpsTrack.back().first+(vehicleDir/10), vehicleDir));
	}


	Plan_ptr curPlanPtr = aa::modules::nav::controller::AutoPlan();
	curPlanPtr->clear();


	if (!mGpsTrack.empty()) {
        pair<Vec3, Vec3> anchor = mGpsTrack[0];
		flt param = 0;

		curPlanPtr->push_back(0, anchor.first, anchor.second);

		//    curPlanPtr->push_back(100, anchor.first+anchor.second*60, anchor.second);

        pair<Vec3, Vec3> lastInsertedPair = anchor;

        for (int i = 1; i < mGpsTrack.size(); i++) {
			flt dist = abs((lastInsertedPair.first - mGpsTrack[i].first).norm());
			param += dist;
			curPlanPtr->push_back(param, mGpsTrack[i].first, mGpsTrack[i].second);
			lastInsertedPair = mGpsTrack[i];
		}
	}


	mPlan = curPlanPtr;

	// write out trajectory
	mPlanOut.write(mPlan);
}

void CarFollowingBehaviour::stopHook()
{
}

void CarFollowingBehaviour::errorHook()
{
}


bool CarFollowingBehaviour::ReplanNow()
{
	mGpsTrack.clear();
	return true;
}


}

}

}

}


