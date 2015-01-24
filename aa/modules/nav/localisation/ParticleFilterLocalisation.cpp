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
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.1);
    double x_norm = 0.0;
    double y_norm = 0.0;
    double theta_norm = 0.0;

    //generate new particles
    // TODO...
    int i =0;
    while(mParticles.size() < 100){
        i = rand() % mParticles.size();
        math::flt randx = mParticles[i][0];
        math::flt randy = mParticles[i][1];
        math::flt randtheta = mParticles[i][2];
        mParticles.push_back(math::Vec3(randx, randy, randtheta));

    }


    //predict particle movement
    //TODO ...
     for (int j=0;j<mParticles.size(); j++) {
         x_norm = distribution(generator);
         y_norm = distribution(generator);
         theta_norm = distribution(generator);
         mParticles[j] = mParticles[j] +mOdometryOnly + (math::Vec3(x_norm,y_norm,theta_norm));
     }
}

void ParticleFilterLocalisation::updateStep()
{

    float dist_l_z;
    float dist_l_x;
    float theta_l_z;
    float theta_l_x;

    //update weights
    //TODO ...
   // float obst_1_x = mMeasurements[0][0];
   //float obst_1_y = mMeasurements[0][1];
   // float obst_2_x = mMeasurements[1][0];
   // float obst_2_y = mMeasurements[0][1];

   // std::cout << "!!!!!!!!!!!!!!!!!!!" << std::endl;
   // std::cout << "mMeasurements: "<< mMeasurements[0][0]<< ";" << mMeasurements[0][1] << std::endl;

    //loop over all weights
    for (int i=0;i<mParticleWeights.size(); i++) {
        //distanz = sqrt(x²+y²)
        mParticleWeights[i]= 1.; //vllt falsch

        //loop over all obstacles
        for(int j =0; j<mMap.size(); j++){
            dist_l_z = sqrt( (mMeasurements[j][0]^2) + (mMeasurements[j][1])^2);
            dist_l_x = sqrt( ((abs(mParticles[i][0]-mMap[j][0]))^2)  + ((abs(mParticles[i][1]-mMap[j][1]))^2));
        mParticleWeights[i] *=  exp(-((dist_l_z-dist_l_x)^2));
        }
    }

    //normalize weights
    //TODO ...
}



}
}
}
}


