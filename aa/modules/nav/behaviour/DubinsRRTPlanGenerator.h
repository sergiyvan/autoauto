#pragma once

#include <util/RtTaskContext.h>

#include <aa/modules/nav/controller/Plan.h>
#include <aa/data/obstacle/BaseObstacleBundle.h>
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

enum DubinsType {LSL, LSR, RSR, RSL, LRL, RLR};

struct DubinsCurve {

    DubinsCurve(){

    }

    DubinsCurve( DubinsType type, math::flt radius, math::Vec3 startPos, math::Vec3 startDir, math::Vec3 endPos, math::Vec3 endDir) {
        mType = type;
        mRadius = radius;
        mStartPos = startPos;
        mStartDir = startDir;
        mEndPos = endPos;
        mEndDir = endDir;

        //calc values depending on type
        math::Vec3 v;
        math::flt c;
        math::flt nx;
        math::flt ny;
        math::Vec3 n;
        math::Vec3 v1;
        math::Vec3 v2;
        math::Vec3 v3;
        math::Vec3 v4;
        math::flt phi;
        math::flt phi2;
        math::flt phi3;

        switch (mType) {
        case DubinsType::RSL:
            mP1Pos = mStartPos+(math::Vec3(mStartDir[1],-mStartDir[0],0)*mRadius);
            mP2Pos = mEndPos+(math::Vec3(-mEndDir[1],mEndDir[0],0)*mRadius);
            v = mP2Pos-mP1Pos;
            if (v.norm()<2*mRadius) {
                mIsValid = false;
            } else {
                c = (mRadius + mRadius)/v.norm();
                nx = v[0]*c - v[1]*sqrt(1-c*c);
                ny = v[0]*sqrt(1-c*c) + v[1]*c;
                n = math::Vec3(nx,ny,0);
                mI1Pos = mP1Pos+n.normalized()*mRadius;
                mI2Pos = mP2Pos-n.normalized()*mRadius;
                mI1Dir = (mI2Pos-mI1Pos).normalized();
                mI2Dir = (mI2Pos-mI1Pos).normalized();

                v1 = (mStartPos-mP1Pos);
                v2 = (mI1Pos-mP1Pos);
                phi = atan2(v2[1],v2[0])-atan2(v1[1],v1[0]);
                if (phi>0) {
                    phi = phi-2*M_PI;
                }
                mD1=fabs(mRadius*phi);

                mD2 = (mI2Pos-mI1Pos).norm();

                v3 = (mI2Pos-mP2Pos);
                v4 = (mEndPos-mP2Pos);
                phi2 = atan2(v4[1],v4[0])-atan2(v3[1],v3[0]);
                if (phi2<0) {
                    phi2 = phi2+2*M_PI;
                }
                mD3=fabs(mRadius*phi2);

                mTotalDist=mD1+mD2+mD3;
                mIsValid=true;
            }
            break;
        case DubinsType::LSR:
            //TODO
            mIsValid = false;
            break;
        case DubinsType::LSL:
            //TODO
            mIsValid = false;
            break;
        case DubinsType::RSR:
            //TODO
            mIsValid = false;
            break;
        case DubinsType::LRL:
            //TODO
            mIsValid = false;
            break;
        case DubinsType::RLR:
            //TODO
            mIsValid = false;
            break;
        default:
            break;
        }
    }

    DubinsType mType;
    math::flt mRadius;
    bool mIsValid; //true if the curve can reach the end pos
    math::Vec3 mStartPos;
    math::Vec3 mStartDir;
    math::Vec3 mP1Pos; //center of start circle
    math::Vec3 mP2Pos; //center of end circle
    math::Vec3 mP3Pos; //center of center circle (only set when type == LRL or RLR)
    math::Vec3 mI1Pos; //pos of the first intersection (the intersection between segment 1 and 2)
    math::Vec3 mI1Dir; //dir of the first intersection
    math::Vec3 mI2Pos; //pos of the second intersection (the intersection between segment 2 and 3)
    math::Vec3 mI2Dir; //dir of the second intersection
    math::Vec3 mEndPos;
    math::Vec3 mEndDir;
    math::flt mD1; // distance (on the circle) between start and first intersection
    math::flt mD2; // distance (on the circle for LRL and RLR) between first and second intersection
    math::flt mD3; // distance (on the circle) between second intersection and end
    math::flt mTotalDist;

};

struct DubinsRRTWaypoint {

    DubinsRRTWaypoint();

    DubinsRRTWaypoint( math::Vec3 pos, math::Vec3 dir, math::flt dist) {
        position = pos;
        orientation = dir;
        distToPrev = dist;
    }

    math::Vec3 position;
    math::Vec3 orientation;
    math::flt distToPrev;
    DubinsCurve dubinsCurve;
    boost::shared_ptr<DubinsRRTWaypoint> prevWaypoint;
};

typedef boost::shared_ptr<DubinsRRTWaypoint> DubinsRRTWaypointPtr;


/*!
* \brief  DubinsRRTPlanGenerator module
*
*/
class DubinsRRTPlanGenerator
    : public util::RtTaskContext
{
public:

    explicit DubinsRRTPlanGenerator(std::string const & name);
    virtual ~DubinsRRTPlanGenerator();

    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void errorHook();

private:

    /** InputPorts: */
    RTT::InputPort< TimedBaseObstacleBundle_ptr > mObstaclesIn;

    /** OutputPorts: */
    RTT::OutputPort< aa::modules::nav::controller::Plan_ptr > mPlanOut;
    RTT::OutputPort< std::vector<DubinsRRTWaypointPtr> > mWaypointsOut;

    /** Methods: */
    void generatePlanFromWaypoint(DubinsRRTWaypointPtr waypoint);
    bool checkTargetReached(DubinsRRTWaypointPtr waypoint);
    math::Vec3 generateRandomPosition(math::flt xmin, math::flt xmax, math::flt ymin, math::flt ymax);
    math::Vec3 generateRandomDir();
    void walkOnDubinsCurve(math::Vec3& resultPos, math::Vec3& resultDir, math::flt dist, DubinsCurve dc);
    bool ReplanNow();
    bool collisionWithObstacle(DubinsRRTWaypointPtr wp, TimedBaseObstacleBundle_ptr obstacles);
    void screenshot();


    /** Properties */
    math::Vec3 mTargetPosition;
    math::flt mDistToDrive;
    math::flt mMaxTurnRadius;
    math::flt mMaxDiffPos;

    /** Member Variables */
    controller::Plan_ptr mPlan;
    std::vector<DubinsRRTWaypointPtr> mNodes;
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
