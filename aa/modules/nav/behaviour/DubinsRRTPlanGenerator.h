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
            mP1Pos = mStartPos+(math::Vec3(-mStartDir[1],mStartDir[0],0)*mRadius);
            mP2Pos = mEndPos+(math::Vec3(mEndDir[1],-mEndDir[0],0)*mRadius);
            v = mP2Pos-mP1Pos;
            if (v.norm()<2*mRadius) {
                mIsValid = false;
            } else {
                c = (mRadius + mRadius)/v.norm();
                nx = v[0]*c + v[1]*sqrt(1-c*c);
                ny = -v[0]*sqrt(1-c*c) + v[1]*c;
                n = math::Vec3(nx,ny,0);
                mI1Pos = mP1Pos+n.normalized()*mRadius;
                mI2Pos = mP2Pos-n.normalized()*mRadius;
                mI1Dir = (mI2Pos-mI1Pos).normalized();
                mI2Dir = (mI2Pos-mI1Pos).normalized();

                v1 = (mStartPos-mP1Pos);
                v2 = (mI1Pos-mP1Pos);
                phi = atan2(v2[1],v2[0])-atan2(v1[1],v1[0]);
                if (phi<0) {
                    phi = phi+2*M_PI;
                }
                mD1=fabs(mRadius*phi);

                mD2 = (mI2Pos-mI1Pos).norm();

                v3 = (mI2Pos-mP2Pos);
                v4 = (mEndPos-mP2Pos);
                phi2 = atan2(v4[1],v4[0])-atan2(v3[1],v3[0]);
                if (phi2>0) {
                    phi2 = phi2-2*M_PI;
                }
                mD3=fabs(mRadius*phi2);

                mTotalDist=mD1+mD2+mD3;
                mIsValid=true;

            }
            break;
        case DubinsType::LSL:
            mP1Pos = mStartPos+(math::Vec3(-mStartDir[1],mStartDir[0],0)*mRadius);
            mP2Pos = mEndPos+(math::Vec3(-mEndDir[1],mEndDir[0],0)*mRadius);
            v = mP2Pos-mP1Pos;
            nx = v[1];
            ny = -v[0];
            n = math::Vec3(nx,ny,0);
            mI1Pos = mP1Pos+n.normalized()*mRadius;
            mI2Pos = mP2Pos+n.normalized()*mRadius;
            mI1Dir = (mI2Pos-mI1Pos).normalized();
            mI2Dir = (mI2Pos-mI1Pos).normalized();

            v1 = (mStartPos-mP1Pos);
            v2 = (mI1Pos-mP1Pos);
            phi = atan2(v2[1],v2[0])-atan2(v1[1],v1[0]);
            if (phi<0) {
                phi = phi+2*M_PI;
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

            break;
        case DubinsType::RSR:
            mP1Pos = mStartPos+(math::Vec3(mStartDir[1],-mStartDir[0],0)*mRadius);
            mP2Pos = mEndPos+(math::Vec3(mEndDir[1],-mEndDir[0],0)*mRadius);
            v = mP2Pos-mP1Pos;
            nx = -v[1];
            ny = v[0];
            n = math::Vec3(nx,ny,0);
            mI1Pos = mP1Pos+n.normalized()*mRadius;
            mI2Pos = mP2Pos+n.normalized()*mRadius;
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
            if (phi2>0) {
                phi2 = phi2-2*M_PI;
            }
            mD3=fabs(mRadius*phi2);

            mTotalDist=mD1+mD2+mD3;

            mIsValid=true;

            break;
        case DubinsType::LRL:
            mP1Pos = mStartPos+(math::Vec3(-mStartDir[1],mStartDir[0],0)*mRadius);
            mP2Pos = mEndPos+(math::Vec3(-mEndDir[1],mEndDir[0],0)*mRadius);
            v = mP2Pos-mP1Pos;
            if (v.norm()>4*mRadius) {
                mIsValid = false;
            } else {
                phi = atan2(v[1],v[0]) + acos(v.norm()/(4*mRadius));

                mP3Pos = mP1Pos + math::Vec3(2*mRadius*cos(phi),2*mRadius*sin(phi),0);
                mI1Pos = mP1Pos+(mP3Pos-mP1Pos)/2;
                mI2Pos = mP2Pos+(mP3Pos-mP2Pos)/2;
                mI2Dir = (mI2Pos-mI1Pos).normalized();

                v1 = (mStartPos-mP1Pos);
                v2 = (mI1Pos-mP1Pos);
                mI1Dir = math::Vec3(-v2[1],v2[0],0).normalized();
                phi = atan2(v2[1],v2[0])-atan2(v1[1],v1[0]);
                if (phi<0) {
                    phi = phi+2*M_PI;
                }
                mD1=fabs(mRadius*phi);

                v3 = (mI2Pos-mP2Pos);
                phi3 = atan2(v3[1],v3[0])-atan2(v2[1],v2[0]);
                if (phi3>0) {
                    phi3 = phi3-2*M_PI;
                }
                mD2=fabs(mRadius*phi3);

                v4 = (mEndPos-mP2Pos);
                mI2Dir = math::Vec3(-v3[1],v3[0],0).normalized();
                phi2 = atan2(v4[1],v4[0])-atan2(v3[1],v3[0]);
                if (phi2<0) {
                    phi2 = phi2+2*M_PI;
                }
                mD3=fabs(mRadius*phi2);

                mTotalDist=mD1+mD2+mD3;

                mIsValid=true;
            }
            break;
        case DubinsType::RLR:
            mP1Pos = mStartPos+(math::Vec3(mStartDir[1],-mStartDir[0],0)*mRadius);
            mP2Pos = mEndPos+(math::Vec3(mEndDir[1],-mEndDir[0],0)*mRadius);
            v = mP2Pos-mP1Pos;

            if (v.norm()>4*mRadius) {
                mIsValid = false;
            } else {
                phi = atan2(v[1],v[0]) - acos(v.norm()/(4*mRadius));

                mP3Pos = mP1Pos + math::Vec3(2*mRadius*cos(phi),2*mRadius*sin(phi),0);
                mI1Pos = mP1Pos+(mP3Pos-mP1Pos)/2;
                mI2Pos = mP2Pos+(mP3Pos-mP2Pos)/2;
                mI2Dir = (mI2Pos-mI1Pos).normalized();

                v1 = (mStartPos-mP1Pos);
                v2 = (mI1Pos-mP1Pos);
                mI1Dir = math::Vec3(v2[1],-v2[0],0).normalized();
                phi = atan2(v2[1],v2[0])-atan2(v1[1],v1[0]);
                if (phi>0) {
                    phi = phi-2*M_PI;
                }
                mD1=fabs(mRadius*phi);

                v3 = (mI2Pos-mP2Pos);
                mI2Dir = math::Vec3(v3[1],-v3[0],0).normalized();
                phi3 = atan2(v3[1],v3[0])-atan2(v2[1],v2[0]);
                if (phi3<0) {
                    phi3 = phi3+2*M_PI;
                }
                mD2=fabs(mRadius*phi3);

                v4 = (mEndPos-mP2Pos);
                phi2 = atan2(v4[1],v4[0])-atan2(v3[1],v3[0]);
                if (phi2>0) {
                    phi2 = phi2-2*M_PI;
                }
                mD3=fabs(mRadius*phi2);

                mTotalDist=mD1+mD2+mD3;

                mIsValid = true;
            }
            break;
        default:
            break;
        }
    }

    DubinsType mType;
    math::flt mRadius;
    bool mIsValid;
    math::Vec3 mStartPos;
    math::Vec3 mStartDir;
    math::Vec3 mI1Pos;
    math::Vec3 mP1Pos; //center of start circle
    math::Vec3 mP2Pos; //center of end circle
    math::Vec3 mP3Pos; //center of center circle (only set when type == LRL or RLR)
    math::Vec3 mI1Dir;
    math::Vec3 mI2Pos;
    math::Vec3 mI2Dir;
    math::Vec3 mEndPos;
    math::Vec3 mEndDir;
    math::flt mD1;
    math::flt mD2;
    math::flt mD3;
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
    void travelOnDubinsCurve(math::Vec3& resultPos, math::Vec3& resultDir, math::flt dist, DubinsCurve dc);
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
