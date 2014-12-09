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
    , mEgoStateIn("EgoStateIn")
    , mObstaclesIn("ObstaclesIn")
    , mPlanOut("PlanOut")
    , mWaypointsOut("WaypointsOut")
    , mTargetPosition(30,15,0)
    , mTargetOrientation(-1,0,0)
    , mDistToDrive(5.0*M_PI/4.0)
    , mMaxTurnRadius(5.0)
    , mMaxDiffPos(2.5)
    , mMaxDiffAngle(M_PI/180)
    , mEpsilon(2.5)
{
    ports()->addPort(mEgoStateIn);
    ports()->addPort(mObstaclesIn);

    ports()->addPort(mPlanOut);
    ports()->addPort(mWaypointsOut);

    addProperty("TargetPosition", mTargetPosition);
    addProperty("TargetOrientation", mTargetOrientation);
    addProperty("DistToDrive", mDistToDrive).doc("distance the car drives each AStar step (in m)");
    addProperty("MaxTurnRadius", mMaxTurnRadius).doc("max turning radius of the car (in m)");
    addProperty("MaxDiffPos", mMaxDiffPos).doc("max dist to target for target reached check (in m)");
    addProperty("MaxDiffAngle", mMaxDiffAngle).doc("max angle difference to target orientation for target reached check (in rad)");
    addProperty("Epsilon", mEpsilon).doc("factor for greedyy heuristic");

    addOperation("ReplanNow", &AStarPlanGenerator::ReplanNow, this, RTT::ClientThread).doc("replan trajectories");

    mObstacles = AutoBaseObstacleBundle();
    mObstacles->clear();

}

AStarPlanGenerator::~AStarPlanGenerator()
{
}


bool AStarPlanGenerator::startHook()
{
    Logger::In in("AStarPlanGenerator");

    REQUIRED_PORT(mObstaclesIn);

    OPTIONAL_PORT(mEgoStateIn);
    OPTIONAL_PORT(mPlanOut);

    ReplanNow();

    return true;
}

void AStarPlanGenerator::updateHook()
{
    Logger::In in("AStarPlanGenerator");

    //read from ports
    mEgoStateIn.read(mEgoState);
    mObstaclesIn.read(mObstacles);

    ReplanNow();

    // write out plan and open list for display module
    mPlanOut.write(mPlan);
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

    ///change this calculation for assignment 8 excercise 1
    flt leftCostWithCurvature = waypoint->costWithCurvature+distToDrive;
    ///

    AStarWaypointPtr left(new AStarWaypoint(waypoint->position+posChangeLeft, rotateLeft*waypoint->orientation, waypoint->costFromStart+distToDrive, leftCostWithCurvature));
    calculateCostToTarget(left);
    left->prevWaypoint = waypoint;
    children.push_back(left);

    ///change this calculation for assignment 8 excercise 1
    flt centerCostWithCurvature = waypoint->costWithCurvature+distToDrive;
    ///

    AStarWaypointPtr center(new AStarWaypoint(waypoint->position+waypoint->orientation*distToDrive, waypoint->orientation, waypoint->costFromStart+distToDrive,centerCostWithCurvature));
    calculateCostToTarget(center);
    center->prevWaypoint = waypoint;
    children.push_back(center);

    Eigen::AngleAxis<double> rotateRight(-angle, Vec3(0,0,1));
    Vec3 relativePosChangeRight(sin(angle)*radius, -(-cos(angle)+1.0)*radius, 0);
    Vec3 posChangeRight = rotateToWaypointOrientation*relativePosChangeRight;

    ///change this calculation for assignment 8 excercise 1
    flt rightCostWithCurvature = waypoint->costWithCurvature+distToDrive;
    ///

    AStarWaypointPtr right(new AStarWaypoint(waypoint->position+posChangeRight, rotateRight*waypoint->orientation, waypoint->costFromStart+distToDrive, rightCostWithCurvature));
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

bool AStarPlanGenerator::collisionWithObstacle(AStarWaypointPtr wp, TimedBaseObstacleBundle_ptr obstacles)
{
    ///This is dummy code! Insert your code here!

    //2d position of the waypoint
    Vec2 pos2d = head(wp->position);

    //iterate through obstacles
    for (TimedBaseObstacleBundle_ptr::element_type::const_iterator ito=obstacles->begin(); ito != obstacles->end(); ++ito) {
        const aa::data::obstacle::BaseObstacle & obstacle = *ito;
        aa::data::obstacle::util::Contour contour = obstacle.contour();
        //iterate through contour points
        for (int i = 1; i < contour.size(); i++){

            Vec2 c = contour[i]; //2d point on the obstacle contour

            if (false) {
                //found collision
                return true;
            }


        }
    }
    return false;
}


void AStarPlanGenerator::calculateCostToTarget(AStarWaypointPtr waypoint)
{
    waypoint->costToTarget = (mTargetPosition-waypoint->position).norm()*mEpsilon;
    waypoint->costTotal = waypoint->costWithCurvature+waypoint->costToTarget;
}

bool AStarPlanGenerator::ReplanNow()
{
    //stamp mTimeStamp for time limit check
    mTimeStamp.stamp();    

    //init open list, reachedTarget and min
    mOpenList.clear();
    bool reachedTarget= false;
    AStarWaypointPtr min;

    //insert start waypoint
    AStarWaypointPtr start(new AStarWaypoint(Vec3(0,0,0), Vec3(1,0,0), 0.0,0.0));
    calculateCostToTarget(start);
    mOpenList.push_back(start);



    //expand path while target not reached or no more options (or time limit reached)
    while (!reachedTarget && !mOpenList.empty()) {
        //check for time limit
        TimeStamp now;
        now.stamp();
        if (1E-9f * RTT::os::TimeService::ticks2nsecs(now - mTimeStamp) > 0.9) {
            //break if we took longer than 0.03 s
            break;
        }

        //search for waypoint with minimum total cost in open list
        min = mOpenList[0];
        int minIndex = 0;
        for (int i=0; i < mOpenList.size(); i++) {
            if (mOpenList[i]->costTotal < min->costTotal) {
                min = mOpenList[i];
                minIndex=i;
            }
        }

        //check if we reached the target
        if (checkTargetReached(min)) {
            reachedTarget=true;
        }

        //generate children of min
        std::vector<AStarWaypointPtr> children = generateChildren(min);

        //check children for collision (insert your code in collisionWithObstacle!)
        std::vector<AStarWaypointPtr> collisionFreeChildren;
        for (std::vector<AStarWaypointPtr>::iterator it = children.begin(); it != children.end(); it++) {
            if (!collisionWithObstacle(*it,mObstacles)) {
                collisionFreeChildren.push_back(*it);
            }
        }

        //erase min from open list and insert (collision free) children
        mOpenList.erase(mOpenList.begin()+minIndex);
        mOpenList.insert(mOpenList.end(), collisionFreeChildren.begin(), collisionFreeChildren.end());

    }

    //generate a plan from min (which has reached the target or is the closest if time limit was reached)
    generatePlanFromWaypoint(min);

    if (!reachedTarget) {
        //at the moment it does not matter what ReplanNow returns but could be used to check if the plan reaches the target
        return false;
    }

    return true;
}



}
}
}
}


