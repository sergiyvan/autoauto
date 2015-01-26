#pragma once

#include <util/RtTaskContext.h>

#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <modules/models/egostate/EgoState.h>

#include <aa/data/OdometricData.h>
#include <core/TimeStamp.h>
#include <fstream>



namespace aa
{
namespace modules
{
namespace nav
{

namespace localisation
{


/*!
* \brief  ParticleFilterLocalisation module
*
*/
class ParticleFilterLocalisation
    : public util::RtTaskContext
{
public:

    explicit ParticleFilterLocalisation(std::string const & name);
    virtual ~ParticleFilterLocalisation();

    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();

private:

    /** InputPorts: */
    RTT::InputPort< TimedBaseObstacleBundle_ptr > mObstaclesIn;
    RTT::InputPort< TimedEgoState > mEgoStateIn;
    RTT::InputPort< data::TimedOdometricData > mOdometricDataIn;

    /** OutputPorts: */
    RTT::OutputPort< std::vector<math::Vec3> > mParticlesOut;

    /** Methods: */
    void updateStep();
    void predictionStep();

    /** Properties */

    /** Member Variables */
    TimedBaseObstacleBundle_ptr mObstacles;
    TimedEgoState mCurEgoState;
    data::TimedOdometricData mOdometricData;

    std::vector<math::Vec3> mParticles;
    std::vector<math::flt> mParticleWeights;

    std::vector<math::Vec2> mMap;
    std::vector<math::Vec2> mMeasurements;

    math::Vec2 mGroundTruth;
    math::Vec3 mOdometryOnly;


    TimeStamp mStartTime;
    TimeStamp mNow;
    std::ofstream mOutputStream;

    int mNumParticles;


};

}


}

 // modules
}

 // nav
}

 // behaviour
