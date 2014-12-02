#pragma once

#include <util/RtTaskContext.h>

#include <aa/modules/nav/controller/Plan.h>

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

    AStarWaypoint( math::Vec3 pos, math::Vec3 dir, math::flt cfs) {
        position = pos;
        orientation = dir;
        costFromStart = cfs;
    }

    math::Vec3 position;
    math::Vec3 orientation;
    math::flt costTotal;
    math::flt costFromStart;
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

    /** OutputPorts: */
	RTT::OutputPort< aa::modules::nav::controller::Plan_ptr > mPlanOut;							/** write out a selected plan */

    /** Methods: */
    void generatePlanFromWaypoint(AStarWaypointPtr waypoint);
    std::vector< AStarWaypointPtr > generateChildren(AStarWaypointPtr waypoint);
    bool checkTargetReached(AStarWaypointPtr waypoint);
    void calculateCostToTarget(AStarWaypointPtr waypoint);
    bool ReplanNow();

    /** Properties */
    math::Vec3 mTargetPosition;
    math::Vec3 mTargetOrientation;
    math::flt mDistToDrive;
    math::flt mMaxTurnRadius;
    math::flt mMaxDiffPos;
    math::flt mMaxDiffAngle;

    /** Member Variables */
    controller::Plan_ptr mPlan;
    std::vector<AStarWaypointPtr> mOpenList;

};

}


}

 // modules
}

 // nav
}

 // behaviour
