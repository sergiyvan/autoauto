#pragma once
#include <gui/Painter3DTask.h>
#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <osg/ShapeDrawable>

#include <core/TimedData.h>
#include <qmutex.h>

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class DisplaySimulator
	: public gui::Painter3DTask
{
public:
	explicit DisplaySimulator(std::string const & name);
	virtual ~DisplaySimulator();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	virtual void init3D(SceneNodePtr);
	virtual void draw3D(DrawArg);

protected:
	RTT::Property<bool> mDrawPhysics;
	RTT::Property<bool> mDrawWorld;
	RTT::Property<bool> mDrawScanner;

private:
	class impl;
	boost::shared_ptr<impl> pimpl;
};

} // namespace modules
} // namespace nav
} // namespace simulator
}
