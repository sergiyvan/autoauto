#pragma once

#include <util/RtTaskContext.h>

#include <aa/modules/nav/controller/Plan.h>
#include <aa/data/obstacle/BaseObstacleBundle.h>

#include <osgViewer/ViewerEventHandlers>



namespace aa
{
namespace modules
{
namespace nav
{

namespace behaviour
{


struct RRTWaypoint {

    RRTWaypoint();

    RRTWaypoint( math::Vec3 pos) {
        position = pos;
    }

    math::Vec3 position;
    boost::shared_ptr<RRTWaypoint> prevWaypoint;
};

typedef boost::shared_ptr<RRTWaypoint> RRTWaypointPtr;


/*!
* \brief  RRTPlanGenerator module
*
*/
class RRTPlanGenerator
    : public util::RtTaskContext
{
public:

    explicit RRTPlanGenerator(std::string const & name);
    virtual ~RRTPlanGenerator();

    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void errorHook();

private:

    /** InputPorts: */
    RTT::InputPort< TimedBaseObstacleBundle_ptr > mObstaclesIn;

    /** OutputPorts: */
    RTT::OutputPort< aa::modules::nav::controller::Plan_ptr > mPlanOut;
    RTT::OutputPort< std::vector<RRTWaypointPtr> > mWaypointsOut;

    /** Methods: */
    void generatePlanFromWaypoint(RRTWaypointPtr waypoint);
    bool checkTargetReached(RRTWaypointPtr waypoint);
    math::Vec3 generateRandomPosition(math::flt xmin, math::flt xmax, math::flt ymin, math::flt ymax);
    bool ReplanNow();
    bool collisionWithObstacle(RRTWaypointPtr wp, TimedBaseObstacleBundle_ptr obstacles);
    void screenshot();


    /** Properties */
    math::Vec3 mTargetPosition;
    math::flt mDistToDrive;
    math::flt mMaxDiffPos;

    /** Member Variables */
    controller::Plan_ptr mPlan;
    std::vector<RRTWaypointPtr> mNodes;
    TimedBaseObstacleBundle_ptr mObstacles;
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
