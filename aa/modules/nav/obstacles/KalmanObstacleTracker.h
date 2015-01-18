#pragma once

#include <util/RtTaskContext.h>

#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <core/TimeStamp.h>
#include <fstream>
#include <Eigen/Dense>

using Eigen::MatrixXd;

namespace aa
{
namespace modules
{
namespace nav
{

namespace obstacles
{


/*!
* \brief  KalmanObstacleTracker module
*
*/
class KalmanObstacleTracker
    : public util::RtTaskContext
{
public:

    explicit KalmanObstacleTracker(std::string const & name);
    virtual ~KalmanObstacleTracker();

    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();

private:

    /** InputPorts: */
    RTT::InputPort< TimedBaseObstacleBundle_ptr > mObstaclesIn;

    /** OutputPorts: */

    /** Methods: */
    void updateStep();
    void predictionStep();

    /** Properties */

    /** Member Variables */
    TimedBaseObstacleBundle_ptr mObstacles;

    // The propagation matrix (relation between one state and the next)
    math::Mat2x2 mA;
    // The sensory data transformation matrix
    math::Mat2x2 mH;
    // The process error covariance matrix
    math::Mat2x2 mQ;
    // The process error covariance matrix
    math::Mat2x2d mR;

    //temporary matrix to calculate r
    MatrixXd tempQ;

    int counter;

    int maxSize;

    // The error covariance matrix
    math::Mat2x2 mP_prior;
    math::Mat2x2 mP_posterior;

    // a priori state estimate
    math::Vec2 mX_prior;
    // a posteriori state estimate
    math::Vec2 mX_posterior;

    // The measurement vector
    math::Vec2 mZ;


    TimeStamp mStartTime;
    TimeStamp mNow;
    std::ofstream mOutputStream;

    std::vector<math::Vec2> mValues;
    std::vector<math::Vec2> mValues2;

};

}


}

 // modules
}

 // nav
}

 // behaviour
