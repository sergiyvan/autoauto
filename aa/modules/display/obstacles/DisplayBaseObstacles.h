#pragma once
/*!
 * \file DisplayBaseObstacles.h
 * \brief Definition of module class DisplayBaseObstacle.
 * \author Michael Schnürmacher
 */

#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <gui/Painter3DTask.h>
#include <qmutex.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace obstacles
{

/*!
* \class DisplayBaseObstacles
* \brief Definition of module class DisplayBaseObstacle.
* \author Michael Schnürmacher
*
* TODO Detailed description of DisplayBaseObstacles
*/
class DisplayBaseObstacles
        : public gui::Painter3DTask
{

public:

    // Definitions:
    
    // Constructors:
    explicit DisplayBaseObstacles(std::string const & name);
    
    // Destructor:
    virtual ~DisplayBaseObstacles();
    

protected:

    /** \name Ports: */
    /*! \{ */
    // Read ports:
    RTT::InputPort<TimedBaseObstacleBundle_ptr> mBaseObstaclesIn;
    

    /*! \} */
    /** \name Properties: */
    /*! \{ */
    
    /*! \} */
    
    // Inherited
    virtual void updateHook();
    virtual void draw3D(DrawArg);

    
private:

    TimedBaseObstacleBundle_ptr mObstacleBundlePtr;

    QMutex mMutex;

};

}
}
}
}
