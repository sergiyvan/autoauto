#include "KalmanObstacleTracker.h"

#include <rtt/Logger.hpp>

#include <util/TaskContextFactory.h>

#include <util/OrocosHelperFunctions.h>


namespace aa
{
namespace modules
{
namespace nav
{

namespace obstacles
{

using RTT::Logger;

REGISTERTASKCONTEXT(KalmanObstacleTracker);

KalmanObstacleTracker::KalmanObstacleTracker(std::string const & name)
    : RtTaskContext(name)
    , mObstaclesIn("ObstaclesIn")
{
    ports()->addEventPort(mObstaclesIn);

    mObstacles = AutoBaseObstacleBundle();
    mObstacles->clear();

}

KalmanObstacleTracker::~KalmanObstacleTracker()
{
}


bool KalmanObstacleTracker::startHook()
{
    Logger::In in("KalmanObstacleTracker");

    REQUIRED_PORT(mObstaclesIn);

    //init kalman filter matrices
    mA << 1, 0.02,
          0, 1;

    mH = math::Mat2x2::Identity();

    mQ << 0.01*0.01*0.02, 0,
          0, 0.01*0.01;

    mR << 1.0, 0.,
          0., 1.0;

    mP_posterior << 1000.0, 0,
          0, 1000.0;

    mX_posterior = math::Vec2(0,0);

    maxSize = 25;

    //max size rows , 2 columns
    tempQ.resize(maxSize,2);

    counter = 0;


    mStartTime.stamp();

    std::string filename = "u11.csv";
    mOutputStream.open(filename.c_str());


    return true;
}

void KalmanObstacleTracker::updateHook()
{
    Logger::In in("KalmanObstacleTracker");


    //read obstacle data from ports
    mObstaclesIn.read(mObstacles);

    predictionStep();

    //get obstacle with specific id
    int id = 2;
    aa::data::obstacle::BaseObstacle obstacle;
    TimedBaseObstacleBundle_ptr::element_type::const_iterator ito;
    for (ito=mObstacles->begin(); ito != mObstacles->end(); ++ito) {
        obstacle = *ito;
        if (obstacle.id() == id) {
            //if we have found the id, break the loop
            break;
        }
    }
    if (ito == mObstacles->end()) {
        //if we found no obstacle with the specific id -> step out of this method
        Logger::log() << Logger::Error << "found no obstacle with id " << id << "! Skip tracking" << Logger::endl;
        return;
    }

    //get obstacle position
    math::Vec2 pos2d = obstacle.boundingCircleCentre();
    mZ[0]=pos2d[0];
    //get obstacle velocity (in x direction)
    math::Vec3 vel3d = obstacle.velocity();
    mZ[1]=vel3d[0];

    if(counter < maxSize){
        if(counter == 0){
            tempQ(counter,0) = mZ[0];
            tempQ(counter,1) = mZ[1];
        }
        else{ // speicher die differenz
            tempQ(counter,0) = mZ[0]-tempQ(counter-1,0);
            tempQ(counter,1) = mZ[1]-tempQ(counter-1,1);
        }
        counter++;
    }else{ //Berechne mQ covarianz
        std::cout << " "<< std::endl;
        std::cout << "================"<< std::endl;
        Eigen::Matrix2d k = tempQ.transpose() *tempQ;
        Eigen::MatrixXd x_mean = tempQ.colwise().mean();
        mQ = (1./(maxSize-1))*(k-(maxSize*(x_mean.transpose()*x_mean)));
        std::cout<< "mQ: " << mQ << std::endl;
        std::cout << "================"<< std::endl;
        std::cout << "================"<< std::endl;
        counter = 0;

    }

    //do the update step
    updateStep();

    //std::cout << mZ.transpose() << std::endl;

    if (mValues.empty()) {
        mValues2.push_back(mZ);
    } else {
        mValues2.push_back(mZ-math::Vec2(mValues.back()[0],0));
    }
    mValues.push_back(mZ);

    Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> > mat(&(mValues2[0][0]), mValues2.size(), 2);
    Eigen::MatrixXd centered = mat.rowwise() - mat.colwise().mean();
    Eigen::MatrixXd cov = (centered.transpose() * centered) / double(mat.rows() - 1);

    //std::cout << "!!!!!!!!!!!!!!!!!!!" << std::endl;
    //std::cout << cov << std::endl;
    //std::cout << "!!!!!!!!!!!!!!!!!!!" << std::endl;


    ///write to file
    mNow.stamp();
    math::flt timeSinceStart = 1E-9f * RTT::os::TimeService::ticks2nsecs(mNow - mStartTime);
    mOutputStream << timeSinceStart << "," << mZ[0] << "," << mZ[1] << "," << mX_posterior[0] << "," << mX_posterior[1] << std::endl;


}

void KalmanObstacleTracker::stopHook()
{
    mOutputStream.close();

}

void KalmanObstacleTracker::predictionStep()
{
    // estimate the new state
    mX_prior = mA * mX_posterior;

    // estimate the new error covariance matrix
    mP_prior = mA * mP_posterior * mA.transpose() + mQ;

}

void KalmanObstacleTracker::updateStep()
{
    // calculate S
    math::Mat2x2 S = mR + mH * mP_prior *mH.transpose();

    // calculate the Kalman gain
    math::Mat2x2 K = mP_prior * mH.transpose() * S.inverse();

    // update the signal estimate using the previous estimate plus the measurement
    mX_posterior = mX_prior + K * (mZ - mH * mX_prior);

    // update the error covariance
    mP_posterior = mP_prior - K * S * K.transpose();
}



}
}
}
}


