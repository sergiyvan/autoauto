#include "ParticleFilterLocalisation.h"

#include <rtt/Logger.hpp>

#include <util/TaskContextFactory.h>

#include <util/OrocosHelperFunctions.h>


namespace aa
{
namespace modules
{
namespace nav
{

namespace localisation
{

using RTT::Logger;

REGISTERTASKCONTEXT(ParticleFilterLocalisation);

ParticleFilterLocalisation::ParticleFilterLocalisation(std::string const & name)
    : RtTaskContext(name)
    , mObstaclesIn("ObstaclesIn")
    , mEgoStateIn("EgoStateIn")
    , mOdometricDataIn("OdometricDataIn")
    , mParticlesOut("ParticlesOut")
{
    ports()->addPort(mObstaclesIn);
    ports()->addPort(mEgoStateIn);
    ports()->addEventPort(mOdometricDataIn);

    ports()->addPort(mParticlesOut);

    mObstacles = AutoBaseObstacleBundle();
    mObstacles->clear();

}

ParticleFilterLocalisation::~ParticleFilterLocalisation()
{
}


bool ParticleFilterLocalisation::startHook()
{
    Logger::In in("ParticleFilterLocalisation");

    REQUIRED_PORTS((mEgoStateIn)(mObstaclesIn)(mOdometricDataIn));


    mStartTime.stamp();

    std::string filename = "u12.csv";
    mOutputStream.open(filename.c_str());

    //init particles
    mParticles.clear();
    math::flt xmin = -10;
    math::flt xmax = 10;
    math::flt ymin = -10;
    math::flt ymax = 10;
    math::flt thetamin = -M_PI;
    math::flt thetamax = M_PI;
    int mNumParticles = 100;
    for (int i=0;i<mNumParticles; i++) {
        math::flt randx = (rand() * (xmax-xmin) / RAND_MAX)+xmin;
        math::flt randy = (rand() * (ymax-ymin) / RAND_MAX)+ymin;
        math::flt randtheta = (rand() * (thetamax-thetamin) / RAND_MAX)+thetamin;

        mParticles.push_back(math::Vec3(randx, randy, randtheta));
        mParticleWeights.push_back(1.0/mNumParticles);

    }

    //init map
    mMap.clear();
    mMap.push_back(math::Vec2(10,15));
    mMap.push_back(math::Vec2(10,-15));

    //init ground truth and mOdometryOnly vectors
    mGroundTruth = math::Vec2(0,0);
    mOdometryOnly = math::Vec3(0,0,0);

    return true;
}

void ParticleFilterLocalisation::updateHook()
{
    Logger::In in("ParticleFilterLocalisation");

    //read data from ports
    mOdometricDataIn.read(mOdometricData);
    mEgoStateIn.read(mCurEgoState);
    mObstaclesIn.read(mObstacles);

    //get ground truth from EgoState
    mGroundTruth = math::head(mCurEgoState.position());

    //sum up odometry only position
    math::flt odometryOnlyAngle = mOdometryOnly[2]+mOdometricData.mAngularDisplacement;
    math::flt odometryOnlyXOffset = mOdometricData.mTravelledDistance*cos(odometryOnlyAngle);
    math::flt odometryOnlyYOffset = mOdometricData.mTravelledDistance*sin(odometryOnlyAngle);
    mOdometryOnly = math::Vec3(mOdometryOnly[0]+odometryOnlyXOffset,mOdometryOnly[1]+odometryOnlyYOffset, odometryOnlyAngle);

    //calculate mMeasurements from EgoState and mObstacles
    mMeasurements.clear();
    for (TimedBaseObstacleBundle_ptr::element_type::const_iterator ito=mObstacles->begin(); ito != mObstacles->end(); ++ito) {
        mMeasurements.push_back(math::head(mCurEgoState.localToGlobal().inverse()*math::Vec3(ito->boundingCircleCentre()[0], ito->boundingCircleCentre()[1], 0)));
    }


    ///do the prediction step
    predictionStep();

    ///do the update step
    updateStep();


    //write out particles for display module
    mParticlesOut.write(mParticles);

    ///write to file
    mNow.stamp();
    math::flt timeSinceStart = 1E-9f * RTT::os::TimeService::ticks2nsecs(mNow - mStartTime);
    mOutputStream << timeSinceStart << "," << mGroundTruth[0] << "," << mGroundTruth[1] << "," << mOdometryOnly[0] << "," << mOdometryOnly[1] << std::endl;


}

void ParticleFilterLocalisation::stopHook()
{
    mOutputStream.close();

}

void ParticleFilterLocalisation::predictionStep()
{
    //generate new particles
    // TODO...

    //predict particle movement
    //TODO ...
}

void ParticleFilterLocalisation::updateStep()
{
    //update weights
    //TODO ...


    //normalize weights
    //TODO ...
}



}
}
}
}


