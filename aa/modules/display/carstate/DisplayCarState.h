#pragma once

#include <gui/Painter3DTask.h>
#include <core/TimedData.h>
#include <qmutex.h>

#include <modules/models/carstate/CarState.h>
#include <aa/modules/models/carstate/PassatCarState.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace carstate
{

/*!	\ingroup DisplayModules
	@{
	\class DisplayCarState
 *   	\brief Display RNDF module.
 *	Display
 */
class DisplayCarState
	: public gui::Painter3DTask
{
public:
	typedef ::math::flt flt;
	explicit DisplayCarState(std::string const & name);
	virtual ~DisplayCarState();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	virtual void draw3D(DrawArg);
	void init3D(SceneNodePtr viewer);

protected:

	QMutex m_mutex;

	/** @name Properties: */
	//@{
	//@}

	/** @name InputPorts: */
	//@{
	RTT::InputPort< ::modules::models::carstate::TimedCarState > mCarStateIn;
	RTT::InputPort< aa::modules::models::carstate::TimedPassatCarState > mPassatCarStateIn;
	//@}

private:
	void renderCarState(flt left, flt right, flt top, flt bottom);
	void renderPassatCarState(flt left, flt right, flt top, flt bottom);

	void drawMeter(flt left, flt right, flt top, flt bottom, std::string text, flt value, flt minVal = -1, flt maxVal = 1);
	void drawToggle(flt x, flt y, flt textOffset, std::string text, bool on);


	::modules::models::carstate::TimedCarState mCurCarState;
	aa::modules::models::carstate::TimedPassatCarState mCurPassatCarState;

};

}
}
}
}
