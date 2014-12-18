#include "DubinsRRTPlanGenerator.h"

#include <rtt/Logger.hpp>
#include <patterns/Singleton.h>

#include <util/TaskContextFactory.h>
#include <math/AutoMath.h>
#include <math/Geodetic.h>
#include <math/Rotate.h>

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
using namespace aa::modules::models::rndf;
using RTT::Logger;
using aa::modules::nav::controller::Plan;
using aa::modules::nav::controller::Plan_ptr;

REGISTERTASKCONTEXT(DubinsRRTPlanGenerator);

DubinsRRTPlanGenerator::DubinsRRTPlanGenerator(string const & name)
    : RtTaskContext(name)
    , mObstaclesIn("ObstaclesIn")
    , mPlanOut("PlanOut")
    , mWaypointsOut("WaypointsOut")
    , mTargetPosition(25,10,0)
    , mDistToDrive(1)
    , mMaxTurnRadius(5.0)
    , mMaxDiffPos(1)
    , mCapture(new osgViewer::ScreenCaptureHandler)
{
    ports()->addPort(mObstaclesIn);

    ports()->addPort(mPlanOut);
    ports()->addPort(mWaypointsOut);

    addProperty("TargetPosition", mTargetPosition);
    addProperty("DistToDrive", mDistToDrive).doc("distance the car drives each AStar step (in m)");
    addProperty("MaxTurnRadius", mMaxTurnRadius).doc("max turning radius of the car (in m)");
    addProperty("MaxDiffPos", mMaxDiffPos).doc("max dist to target for target reached check (in m)");

    addOperation("ReplanNow", &DubinsRRTPlanGenerator::ReplanNow, this, RTT::ClientThread).doc("replan trajectories");

    mObstacles = AutoBaseObstacleBundle();
    mObstacles->clear();

}

DubinsRRTPlanGenerator::~DubinsRRTPlanGenerator()
{
}


bool DubinsRRTPlanGenerator::startHook()
{
    Logger::In in("DubinsRRTPlanGenerator");

    REQUIRED_PORT(mObstaclesIn);

    OPTIONAL_PORT(mPlanOut);

    gui::View3D * view = gui::View3D::getView3D(0);
    view->addEventHandler(mCapture);

    mCapture->setCaptureOperation(new osgViewer::ScreenCaptureHandler::WriteToFile("screenshot", "png", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER));
    mCapture->setFramesToCapture(1);

    ReplanNow();

    return true;
}

void DubinsRRTPlanGenerator::updateHook()
{
    Logger::In in("DubinsRRTPlanGenerator");

    //read from ports
    mObstaclesIn.read(mObstacles);

    // write out plan and open list for display module
    mPlanOut.write(mPlan);
    mWaypointsOut.write(mNodes);

}

void DubinsRRTPlanGenerator::stopHook()
{
    gui::View3D * view = gui::View3D::getView3D(0);
    view->removeEventHandler(mCapture);
}

void DubinsRRTPlanGenerator::errorHook()
{
}



void DubinsRRTPlanGenerator::generatePlanFromWaypoint(DubinsRRTWaypointPtr waypoint)
{
    std::vector<DubinsRRTWaypoint> waypoints;
    waypoints.push_back(*waypoint.get());
    DubinsRRTWaypointPtr prev = waypoint->prevWaypoint;
    while (prev) {
        waypoints.push_back(*prev.get());
        prev = prev->prevWaypoint;
    }


    if (waypoints.size() > 1) {
        Plan_ptr curPlanPtr = aa::modules::nav::controller::AutoPlan();
        curPlanPtr->clear();

        flt distanceTravelled = 0;
        for(std::vector<DubinsRRTWaypoint>::reverse_iterator rit = waypoints.rbegin(); rit != waypoints.rend(); ++rit)
        {
            DubinsRRTWaypoint wp = (*rit);
            if (wp.prevWaypoint) {
                //we have an prev waypoint so we have a dc
                if (wp.distToPrev > wp.dubinsCurve.mD1+wp.dubinsCurve.mD2) {
                    //we are on the 3rd segment -> insert both intersection point in the plan
                    distanceTravelled+=wp.dubinsCurve.mD1;
                    curPlanPtr->push_back(distanceTravelled, wp.dubinsCurve.mI1Pos, wp.dubinsCurve.mI1Dir);

                    distanceTravelled+=wp.dubinsCurve.mD2;
                    curPlanPtr->push_back(distanceTravelled, wp.dubinsCurve.mI2Pos, wp.dubinsCurve.mI2Dir);

                    distanceTravelled+= wp.distToPrev-(wp.dubinsCurve.mD1+wp.dubinsCurve.mD2);
                    curPlanPtr->push_back(distanceTravelled, wp.position, wp.orientation);
                } else if (wp.distToPrev > wp.dubinsCurve.mD1) {
                    //we are on the 2nd segment -> insert first intersection point in the plan
                    distanceTravelled+=wp.dubinsCurve.mD1;
                    curPlanPtr->push_back(distanceTravelled, wp.dubinsCurve.mI1Pos, wp.dubinsCurve.mI1Dir);

                    distanceTravelled+= wp.distToPrev-(wp.dubinsCurve.mD1);
                    curPlanPtr->push_back(distanceTravelled, wp.position, wp.orientation);
                } else {
                    //we are on first segment -> just insert wp
                    distanceTravelled+= wp.distToPrev;
                    curPlanPtr->push_back(distanceTravelled, wp.position, wp.orientation);
                }
            } else {
                //this should happen only with the first wp
                distanceTravelled += (*rit).distToPrev;
                curPlanPtr->push_back(distanceTravelled, (*rit).position, (*rit).orientation);
            }

        }

        mPlan = curPlanPtr;

    }

}


bool DubinsRRTPlanGenerator::checkTargetReached(DubinsRRTWaypointPtr waypoint)
{
    flt distToTarget = (waypoint->position - mTargetPosition).norm();

    return (distToTarget < mMaxDiffPos);
}

bool DubinsRRTPlanGenerator::collisionWithObstacle(DubinsRRTWaypointPtr wp, TimedBaseObstacleBundle_ptr obstacles)
{
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

Vec3 DubinsRRTPlanGenerator::generateRandomPosition(flt xmin, flt xmax, flt ymin, flt ymax) {
    flt randx = (rand() * (xmax-xmin) / RAND_MAX)+xmin;
    flt randy = (rand() * (ymax-ymin) / RAND_MAX)+ymin;

    return Vec3(randx, randy, 0);
}

Vec3 DubinsRRTPlanGenerator::generateRandomDir() {
    flt phi = (rand() * 2*M_PI / RAND_MAX);
    return Vec3(cos(phi), sin(phi), 0);
}

void DubinsRRTPlanGenerator::walkOnDubinsCurve(Vec3 &resultPos, Vec3 &resultDir, flt dist, DubinsCurve dc) {

    //TODO this is dummy code. Insert your code here
    resultPos=dc.mEndPos;
    resultDir=dc.mEndDir;

}

bool DubinsRRTPlanGenerator::ReplanNow()
{
    //init open list, reachedTarget and min
    mNodes.clear();
    bool reachedTarget= false;
    DubinsRRTWaypointPtr q_new;

    //insert start waypoint
    DubinsRRTWaypointPtr start(new DubinsRRTWaypoint(Vec3(0,0,0),Vec3(1,0,0),0));
    mNodes.push_back(start);



    //expand path while target not reached or no more options (or time limit reached)
    while (!reachedTarget) {
        //generate new random position in search region
        Vec3 q_rand_pos = generateRandomPosition(-10,30,-20,20);
        Vec3 q_rand_dir = generateRandomDir();

        //TODO search for closest waypoint and dubins curve
        DubinsRRTWaypointPtr closestWP = mNodes[0];
        DubinsCurve min(DubinsType::RSL, mMaxTurnRadius, mNodes[0]->position, mNodes[0]->orientation, q_rand_pos, q_rand_dir);

        //create new waypoint
        Vec3 q_new_pos;
        Vec3 q_new_dir;
        walkOnDubinsCurve(q_new_pos, q_new_dir,mDistToDrive,min);
        q_new = DubinsRRTWaypointPtr(new DubinsRRTWaypoint(q_new_pos,q_new_dir, std::min(mDistToDrive,min.mTotalDist)));
        q_new->prevWaypoint = closestWP;
        q_new->dubinsCurve = min;

        //check for collision
        if (!collisionWithObstacle(q_new, mObstacles)) {

            //check if we reached the target
            if (checkTargetReached(q_new)) {
                reachedTarget=true;
            }

            mNodes.push_back(q_new);

        }

    }

    generatePlanFromWaypoint(q_new);

    if (!reachedTarget) {
        //at the moment it does not matter what ReplanNow returns but could be used to check if the plan reaches the target
        return false;
    }

    return true;
}


void DubinsRRTPlanGenerator::screenshot()
{
    mCapture->setFramesToCapture(1);
    mCapture->startCapture();
}


}
}
}
}


