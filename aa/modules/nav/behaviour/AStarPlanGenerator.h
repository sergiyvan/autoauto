#pragma once

#include <util/RtTaskContext.h>

#include <aa/modules/nav/controller/Plan.h>
#include <modules/models/egostate/EgoState.h>
#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <QMutex>
#include <core/TimeStamp.h>

#include <osgViewer/ViewerEventHandlers>



namespace aa
{
namespace modules
{
namespace nav
{

namespace behaviour
{


struct AStarWaypoint {

    AStarWaypoint();

    AStarWaypoint( math::Vec3 pos, math::Vec3 dir, math::flt cfs, math::flt cwc) {
        position = pos;
        orientation = dir;
        costFromStart = cfs;
        costWithCurvature = cwc;
    }

    math::Vec3 position;
    math::Vec3 orientation;
    math::flt costTotal;
    math::flt prevCurvature;
    math::flt costFromStart;
    math::flt costWithCurvature;
    math::flt costToTarget;
    boost::shared_ptr<AStarWaypoint> prevWaypoint;
};

typedef boost::shared_ptr<AStarWaypoint> AStarWaypointPtr;


/*!
* \brief  AStarPlanGenerator module
*
*/
class AStarPlanGenerator
    : public util::RtTaskContext
{
public:

    explicit AStarPlanGenerator(std::string const & name);
    virtual ~AStarPlanGenerator();

    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void errorHook();

private:

    /** InputPorts: */
    RTT::InputPort< TimedEgoState > mEgoStateIn;
    RTT::InputPort< TimedBaseObstacleBundle_ptr > mObstaclesIn;

    /** OutputPorts: */
    RTT::OutputPort< aa::modules::nav::controller::Plan_ptr > mPlanOut;
    RTT::OutputPort< std::vector<AStarWaypointPtr> > mWaypointsOut;

    /** Methods: */
    void generatePlanFromWaypoint(AStarWaypointPtr waypoint);
    std::vector< AStarWaypointPtr > generateChildren(AStarWaypointPtr waypoint);
    bool checkTargetReached(AStarWaypointPtr waypoint);
    void calculateCostToTarget(AStarWaypointPtr waypoint);
    bool ReplanNow();
    bool collisionWithObstacle(AStarWaypointPtr wp, TimedBaseObstacleBundle_ptr obstacles);
    void screenshot();


    /** Properties */
    math::Vec3 mTargetPosition;
    math::Vec3 mTargetOrientation;
    math::flt mDistToDrive;
    math::flt mMaxTurnRadius;
    math::flt mMaxDiffPos;
    math::flt mMaxDiffAngle;
    math::flt mEpsilon;

    /** Member Variables */
    controller::Plan_ptr mPlan;
    std::vector<AStarWaypointPtr> mOpenList;
    TimedEgoState mEgoState;
    TimedBaseObstacleBundle_ptr mObstacles;
    QMutex mMutex;
    TimeStamp mTimeStamp;
    osgViewer::ScreenCaptureHandler * mCapture;


};

}


}

 // modules
}

 // nav
}

 // behaviour
