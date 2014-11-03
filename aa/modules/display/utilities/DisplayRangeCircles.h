#pragma once
/*!
 * \file DisplayRangeCircles.h
 * \author Michael Schnürmacher
 */

#include <boost/smart_ptr.hpp>
#include <core/TimedData.h>
#include <gui/Painter3DTask.h>
#include <math/AutoMath.h>
#include <modules/models/egostate/EgoState.h>
#include <qmutex.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace utilities
{

class DisplayRangeCircles
	: public gui::Painter3DTask
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	explicit DisplayRangeCircles(std::string const & name);
	~DisplayRangeCircles();

	void updateHook();
	void draw3D(DrawArg);


protected:

	/** \name Ports: */
	/** { */
	RTT::InputPort<TimedEgoState> mEgoStateIn;
	/** } */

	/** \name Properties: */
	/** { */
	flt mResolution;
	flt mRange;
	/** } */

private:

	QMutex mMutex;
	TimedEgoState mEgoState;

};

}
}
}
}