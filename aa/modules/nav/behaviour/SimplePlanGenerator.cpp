#include "SimplePlanGenerator.h"

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

REGISTERTASKCONTEXT(SimplePlanGenerator);

SimplePlanGenerator::SimplePlanGenerator(string const & name)
	: RtTaskContext(name)
	, mPlanOut("PlanOut")
{
	ports()->addPort(mPlanOut);

    addOperation("ReplanNow", &SimplePlanGenerator::ReplanNow, this, RTT::ClientThread).doc("replan trajectories");

}

SimplePlanGenerator::~SimplePlanGenerator()
{
}


bool SimplePlanGenerator::startHook()
{
    Logger::In in("SimplePlanGenerator");

	mStateMachine = findPeer(this, "StateMachine");

	if (!mStateMachine) {
		logError() << "missing StateMachine peer";
		return false;
	}

	OPTIONAL_PORT(mPlanOut);


	return true;
}

void SimplePlanGenerator::updateHook()
{
    Logger::In in("SimplePlanGenerator");

    Plan_ptr curPlanPtr = aa::modules::nav::controller::AutoPlan();
    curPlanPtr->clear();

    curPlanPtr->push_back(0, Vec3(0,0,0), Vec3(1,0,0));
    curPlanPtr->push_back(75, Vec3(50,10,0), Vec3(1,0,0));
    curPlanPtr->push_back(150, Vec3(100,-10,0), Vec3(1,0,0));
    curPlanPtr->push_back(225, Vec3(150,10,0), Vec3(1,0,0));
    curPlanPtr->push_back(300, Vec3(200,-10,0), Vec3(1,0,0));
    curPlanPtr->push_back(375, Vec3(250,10,0), Vec3(1,0,0));
    curPlanPtr->push_back(450, Vec3(300,-10,0), Vec3(1,0,0));
    curPlanPtr->push_back(525, Vec3(350,10,0), Vec3(1,0,0));
    curPlanPtr->push_back(600, Vec3(400,-10,0), Vec3(1,0,0));

    mPlan = curPlanPtr;

    // write out plan
	mPlanOut.write(mPlan);
}

void SimplePlanGenerator::stopHook()
{
}

void SimplePlanGenerator::errorHook()
{
}


bool SimplePlanGenerator::ReplanNow()
{
	return true;
}


}

}

}

}


