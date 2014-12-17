#include "RRTPlanGenerator.h"

#include <rtt/Logger.hpp>
#include <patterns/Singleton.h>

#include <util/TaskContextFactory.h>
#include <math/AutoMath.h>

#include <math/PathSpline.h>
#include <util/OrocosHelperFunctions.h>

#include <aa/modules/nav/statemachine/StateMachine.h>

#include <gui/View3D.h>



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
using RTT::Logger;
using aa::modules::nav::controller::Plan;
using aa::modules::nav::controller::Plan_ptr;

REGISTERTASKCONTEXT(RRTPlanGenerator);

RRTPlanGenerator::RRTPlanGenerator(string const & name)
    : RtTaskContext(name)
    , mObstaclesIn("ObstaclesIn")
    , mPlanOut("PlanOut")
    , mWaypointsOut("WaypointsOut")
    , mTargetPosition(25,10,0)
    , mDistToDrive(1)
    , mMaxDiffPos(1)
    , mCapture(new osgViewer::ScreenCaptureHandler)
{
    ports()->addPort(mObstaclesIn);

    ports()->addPort(mPlanOut);
    ports()->addPort(mWaypointsOut);

    addProperty("TargetPosition", mTargetPosition);
    addProperty("DistToDrive", mDistToDrive).doc("max distance the car drives each RTT step (in m)");
    addProperty("MaxDiffPos", mMaxDiffPos).doc("max dist to target for target reached check (in m)");

    addOperation("ReplanNow", &RRTPlanGenerator::ReplanNow, this, RTT::ClientThread).doc("replan trajectories");

    mObstacles = AutoBaseObstacleBundle();
    mObstacles->clear();

}

RRTPlanGenerator::~RRTPlanGenerator()
{
}


bool RRTPlanGenerator::startHook()
{
    Logger::In in("RRTPlanGenerator");

    REQUIRED_PORT(mObstaclesIn);

    OPTIONAL_PORT(mPlanOut);

    gui::View3D * view = gui::View3D::getView3D(0);
    view->addEventHandler(mCapture);

    mCapture->setCaptureOperation(new osgViewer::ScreenCaptureHandler::WriteToFile("screenshot", "png", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER));
    mCapture->setFramesToCapture(1);


    ReplanNow();

    return true;
}

void RRTPlanGenerator::updateHook()
{
    Logger::In in("RRTPlanGenerator");

    //read from ports
    mObstaclesIn.read(mObstacles);

    // write out plan and open list for display module
    mPlanOut.write(mPlan);
    mWaypointsOut.write(mNodes);

}

void RRTPlanGenerator::stopHook()
{
    gui::View3D * view = gui::View3D::getView3D(0);
    view->removeEventHandler(mCapture);
}


void RRTPlanGenerator::generatePlanFromWaypoint(RRTWaypointPtr waypoint)
{
    std::vector<RRTWaypoint> waypoints;
    waypoints.push_back(*waypoint.get());
    RRTWaypointPtr prev = waypoint->prevWaypoint;
    while (prev) {
        waypoints.push_back(*prev.get());
        prev = prev->prevWaypoint;
    }


    if (waypoints.size() > 1) {
        Plan_ptr curPlanPtr = aa::modules::nav::controller::AutoPlan();
        curPlanPtr->clear();

        flt distanceTravelled = 0;
        RRTWaypoint prev = waypoints.back();
        curPlanPtr->push_back(0, prev.position, Vec3(1,0,0));

        for(std::vector<RRTWaypoint>::reverse_iterator rit = waypoints.rbegin()+1; rit != waypoints.rend(); ++rit)
        {
            distanceTravelled += ((*rit).position-prev.position).norm();
            curPlanPtr->push_back(distanceTravelled, (*rit).position, ((*rit).position-prev.position).normalized());
            prev = *rit;
        }

        mPlan = curPlanPtr;

    }

}


bool RRTPlanGenerator::checkTargetReached(RRTWaypointPtr waypoint)
{
    flt distToTarget = (waypoint->position - mTargetPosition).norm();

    return (distToTarget < mMaxDiffPos);
}

bool RRTPlanGenerator::collisionWithObstacle(RRTWaypointPtr wp, TimedBaseObstacleBundle_ptr obstacles)
{
    //intersection check done like in http://www.tonypa.pri.ee/vectors/tut05.html

    if (!obstacles)
        return false;

    Vec2 ab = head(wp->position) - head(wp->prevWaypoint->position); //2d vector from a to b

    for (TimedBaseObstacleBundle_ptr::element_type::const_iterator ito=obstacles->begin(); ito != obstacles->end(); ++ito) {
        //iterate through contour points
        const aa::data::obstacle::BaseObstacle & obstacle = *ito;
        aa::data::obstacle::util::Contour contour = obstacle.contour();
        for (aa::data::obstacle::util::Contour::iterator it= contour.begin()+1; it!=contour.end(); it++){

            Vec2 c = (*it)-*(it-1); //vector on the obstacle contour

            Vec2 v = head(wp->prevWaypoint->position) - *(it-1); //vector from origin of contour vector to origin of ab
            // t=perP(v, ab)/perP(c, ab)  (perP = perp product);
            flt t=(v[0]*ab[1]-v[1]*ab[0]) / (c[0]*ab[1]-c[1]*ab[0]);

            Vec2 v2 = -v; //vector from origin of ab to origin of contour vector
            // t2=perP(v2, c)/perP(ab, c)  (perP = perp product);
            flt t2=(v2[0]*c[1]-v2[1]*c[0]) / (ab[0]*c[1]-ab[1]*c[0]);

            if (t >= 0 && t <= 1 && t2 >= 0 && t2 <= 1) {
                //found intersection
                return true;
            }


        }
    }
    return false;
}

Vec3 RRTPlanGenerator::generateRandomPosition(flt xmin, flt xmax, flt ymin, flt ymax) {
    flt randx = (rand() * (xmax-xmin) / RAND_MAX)+xmin;
    flt randy = (rand() * (ymax-ymin) / RAND_MAX)+ymin;

    return Vec3(randx, randy, 0);
}


bool RRTPlanGenerator::ReplanNow()
{
    //init open list, reachedTarget and q_new
    mNodes.clear();
    bool reachedTarget= false;
    RRTWaypointPtr q_new;

    //insert start waypoint
    RRTWaypointPtr start(new RRTWaypoint(Vec3(0,0,0)));
    mNodes.push_back(start);

    //expand tree while target not reached
    while (!reachedTarget) {
        //generate new random position in search region
        Vec3 q_rand_pos = generateRandomPosition(0,30,-15,15);

        //TODO insert your code here

        //dummy code
        q_new = start;
        reachedTarget = true;

    }

    generatePlanFromWaypoint(q_new);

    if (!reachedTarget) {
        //at the moment it does not matter what ReplanNow returns
        //but could be used to check if the plan reaches the target
        return false;
    }

    return true;
}


void RRTPlanGenerator::screenshot()
{
    mCapture->setFramesToCapture(1);
    mCapture->startCapture();
}


}
}
}
}


