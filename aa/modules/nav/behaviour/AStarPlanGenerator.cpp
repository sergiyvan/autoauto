#include "AStarPlanGenerator.h"

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

REGISTERTASKCONTEXT(AStarPlanGenerator);

AStarPlanGenerator::AStarPlanGenerator(string const & name)
	: RtTaskContext(name)
	, mPlanOut("PlanOut")
    , mWaypointsOut("WaypointsOut")
    , mTargetPosition(30,15,0)
    , mTargetOrientation(-1,0,0)
    , mDistToDrive(5.0*M_PI/4.0)
    , mMaxTurnRadius(5.0)
    , mMaxDiffPos(2.5)
    , mMaxDiffAngle(M_PI/180.0)
{
    ports()->addPort(mPlanOut);
    ports()->addPort(mWaypointsOut);

    addProperty("TargetPosition", mTargetPosition);
    addProperty("TargetOrientation", mTargetOrientation);
    addProperty("DistToDrive", mDistToDrive).doc("distance the car drives each AStar step (in m)");
    addProperty("MaxTurnRadius", mMaxTurnRadius).doc("max turning radius of the car (in m)");
    addProperty("MaxDiffPos", mMaxDiffPos).doc("max dist to target for target reached check (in m)");
    addProperty("MaxDiffAngle", mMaxDiffAngle).doc("max angle difference to target orientation for target reached check (in rad)");

    addOperation("ReplanNow", &AStarPlanGenerator::ReplanNow, this, RTT::ClientThread).doc("replan trajectories");

}

AStarPlanGenerator::~AStarPlanGenerator()
{
}


bool AStarPlanGenerator::startHook()
{
    Logger::In in("AStarPlanGenerator");

	OPTIONAL_PORT(mPlanOut);

    ReplanNow();

	return true;
}

void AStarPlanGenerator::updateHook()
{
    Logger::In in("AStarPlanGenerator");

    // write out plan
	mPlanOut.write(mPlan);

    //write out open list for display module
    mWaypointsOut.write(mOpenList);

}

void AStarPlanGenerator::stopHook()
{
}

void AStarPlanGenerator::errorHook()
{
}



void AStarPlanGenerator::generatePlanFromWaypoint(AStarWaypointPtr waypoint)
{
    std::vector<AStarWaypoint> waypoints;
    waypoints.push_back(*waypoint.get());
    AStarWaypointPtr prev = waypoint->prevWaypoint;
    while (prev) {
        waypoints.push_back(*prev.get());
        prev = prev->prevWaypoint;
    }


    Plan_ptr curPlanPtr = aa::modules::nav::controller::AutoPlan();
    curPlanPtr->clear();

    for(std::vector<AStarWaypoint>::reverse_iterator rit = waypoints.rbegin(); rit != waypoints.rend(); ++rit)
    {
        curPlanPtr->push_back((*rit).costFromStart, (*rit).position, (*rit).orientation);
    }

    mPlan = curPlanPtr;

}

std::vector<AStarWaypointPtr> AStarPlanGenerator::generateChildren(AStarWaypointPtr waypoint)
{
    std::vector<AStarWaypointPtr> children;

    flt distToDrive = mDistToDrive;
    flt radius = mMaxTurnRadius; //Wendekreis des Autos

    flt angle = distToDrive/radius; //Das Auto rotiert um diesen Winkel wenn es distToDrive m auf dem Kreisbegen mit r=radius fährt.

    flt waypointAngle = atan2(waypoint->orientation[1],waypoint->orientation[0]);
    Eigen::AngleAxis<double> rotateToWaypointOrientation(waypointAngle, Vec3(0,0,1));

    Eigen::AngleAxis<double> rotateLeft(angle, Vec3(0,0,1));
    Vec3 relativePosChangeLeft(sin(angle)*radius, (-cos(angle)+1.0)*radius, 0);
    Vec3 posChangeLeft = rotateToWaypointOrientation*relativePosChangeLeft;
    AStarWaypointPtr left(new AStarWaypoint(waypoint->position+posChangeLeft, rotateLeft*waypoint->orientation, waypoint->costFromStart+distToDrive));
    calculateCostToTarget(left);
    left->prevWaypoint = waypoint;
    children.push_back(left);

    AStarWaypointPtr center(new AStarWaypoint(waypoint->position+waypoint->orientation*distToDrive, waypoint->orientation, waypoint->costFromStart+distToDrive));
    calculateCostToTarget(center);
    center->prevWaypoint = waypoint;
    children.push_back(center);

    Eigen::AngleAxis<double> rotateRight(-angle, Vec3(0,0,1));
    Vec3 relativePosChangeRight(sin(angle)*radius, -(-cos(angle)+1.0)*radius, 0);
    Vec3 posChangeRight = rotateToWaypointOrientation*relativePosChangeRight;
    AStarWaypointPtr right(new AStarWaypoint(waypoint->position+posChangeRight, rotateRight*waypoint->orientation, waypoint->costFromStart+distToDrive));
    calculateCostToTarget(right);
    right->prevWaypoint = waypoint;
    children.push_back(right);

    return children;
}

bool AStarPlanGenerator::checkTargetReached(AStarWaypointPtr waypoint)
{
    flt waypointAngle = atan2(waypoint->orientation[1],waypoint->orientation[0]);
    flt targetAngle = atan2(mTargetOrientation[1],mTargetOrientation[0]);

    flt orientationDiffAngle = waypointAngle-targetAngle;

    //normalize angle
    if (orientationDiffAngle >= M_PI) {
        orientationDiffAngle -= 2*M_PI;
    }
    if (orientationDiffAngle < -M_PI) {
        orientationDiffAngle += 2*M_PI;
    }

    flt distToTarget = (waypoint->position - mTargetPosition).norm();

    return (distToTarget < mMaxDiffPos && abs(orientationDiffAngle)<mMaxDiffAngle);
}

void AStarPlanGenerator::calculateCostToTarget(AStarWaypointPtr waypoint)
{
    //insert your code/heuristic here
    waypoint->costToTarget = 0;
    waypoint->costTotal = waypoint->costFromStart+waypoint->costToTarget;
}

bool AStarPlanGenerator::ReplanNow()
{
    //implement AStar here. The following code is just here to give you some examples how to use the data structures.

    mOpenList.clear();

    AStarWaypointPtr start(new AStarWaypoint(Vec3(0,0,0), Vec3(1,0,0), 0.0));
    calculateCostToTarget(start);

    //insert waypoint in open list (which is a vector of waypoint pointers)
    mOpenList.push_back(start);

    //delete waypoint in open list
    int indexToDelete = 0;
    mOpenList.erase(mOpenList.begin()+indexToDelete);

    //generate children of waypoint
    std::vector<AStarWaypointPtr> children = generateChildren(start);

    //insert vector of waypoints to open list
    mOpenList.insert(mOpenList.end(), children.begin(), children.end());

    //Log size of open list
    Logger::log() << Logger::Debug << "open list size: " << mOpenList.size() << std::endl;

    //Log total cost of all waypoints in open list
    for (int i=0;i<mOpenList.size();i++) {
        Logger::log() << Logger::Debug << "total cost of waypoint " << i << ": " << mOpenList[i]->costToTarget << std::endl;
    }

    //check if first Waypoint in open list has reached target
    if (checkTargetReached(mOpenList.front())) {
        Logger::log() << Logger::Debug << "Reached Goal" << Logger::endl;
    }


    //generate a plan for the controller from a waypoint
    //insert only your final waypoint (the one which has reached the target) here.
    //(Previous waypoints are added recursively with the prevWaypoint pointer. You dont have to care about that)
    generatePlanFromWaypoint(mOpenList[2]);

    return true;
}



}

}

}

}


