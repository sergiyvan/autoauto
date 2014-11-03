/*!
 * \file "DisplayBaseObstacles.h"
 * \brief Definition of module class DisplayBaseObstacle.
 * \author Michael Schnürmacher
 */

#include "DisplayBaseObstacles.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <gui/GlTools.h>
#include <math/IbeoMath.h>


using namespace aa::modules::display::obstacles;
using namespace aa::data::obstacle;

REGISTERTASKCONTEXT(DisplayBaseObstacles);



DisplayBaseObstacles::DisplayBaseObstacles(std::string const & name)
    : gui::Painter3DTask(name)
    // Read ports:
    , mBaseObstaclesIn("BaseObstaclesIn")
    // Properties:

{
    addEventPort(mBaseObstaclesIn);
    
}


DisplayBaseObstacles::~DisplayBaseObstacles()
{

}


void DisplayBaseObstacles::updateHook()
{
    mMutex.lock();
    mBaseObstaclesIn.read(mObstacleBundlePtr);
    mMutex.unlock();
}


void DisplayBaseObstacles::draw3D(DrawArg)
{
    RTT::Logger::In in("DisplayBaseObstacles");

    mMutex.lock();
    if (!mObstacleBundlePtr) {
        mMutex.unlock();
        return;
    }

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushMatrix();
    glEnable(GL_BLEND);
    glDisable(GL_LIGHTING);

    for (uint i = 0; i < mObstacleBundlePtr->size(); ++i) {
        BaseObstacle const & obstacle = mObstacleBundlePtr->at(i);

        std::string idStr = "";
        math::ibeomath::append(idStr, obstacle.id());
        gui::renderString(&idStr[0], (math::zeroExtend(obstacle.boundingCircleCentre()) + math::Vec3(0.0, 0.0, 0.5)), 1.0, 0.0, 0.0);
    }

    glPopMatrix();
    glPopAttrib();

    mMutex.unlock();
}



