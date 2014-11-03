/*!
 * \file "DisplayCartesianSystem.h"
 * \brief Displays a cartesian coordinate system.
 * \author Michael Schnürmacher
 */

#include "DisplayRangeCircles.h"

#include "Util.h"

#include <util/TaskContextFactory.h>
#include <math/AutoMath.h>
#include <gui/GlTools.h>
#include <util/OrocosHelperFunctions.h>


using namespace aa::modules::display::utilities;
using namespace gui;


REGISTERTASKCONTEXT(DisplayRangeCircles);

DisplayRangeCircles::DisplayRangeCircles(std::string const & name)
	: Painter3DTask(name)
	// Inports
	, mEgoStateIn("EgoStateIn")
	// Properties
	, mResolution(10.0)
	, mRange(200.0)
	// Members
{
	addEventPort(mEgoStateIn);

	addProperty("Resolution", mResolution);
	addProperty("Range", mRange);

}


DisplayRangeCircles::~DisplayRangeCircles()
{

}

void DisplayRangeCircles::updateHook()
{
	mMutex.lock();
	mEgoStateIn.read(mEgoState);
	mMutex.unlock();
}


void DisplayRangeCircles::draw3D(DrawArg)
{
	mMutex.lock();

	if (mResolution == 0) {
		mMutex.unlock();
		return;
	}

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPushMatrix();


	for (uint i = 0; i < int(mRange / mResolution); ++i) {
		renderCircle(mEgoState.position(), flt(i + 1) * mResolution, Color::BLACK, 0.1);
	}


	glPopMatrix();
	glPopAttrib();

	mMutex.unlock();
}







