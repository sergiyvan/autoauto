#pragma once

#include <gui/Painter3DTask.h>

#include <core/TimedData.h>
#include <boost/optional.hpp>
#include <core/TimedData.h>
#include <aa/data/ObstacleData.h>
#include <qmutex.h>

/*!	\ingroup DisplayModules
	@{
	\class DisplaySimulatorCtrl
 *   	\brief Display RNDF module.
 *	Display
 */

namespace aa
{
namespace modules
{
namespace nav
{

namespace simulator
{

class DisplaySimulatorCtrl
	: public gui::Painter3DTask
	, public osgGA::GUIEventHandler
{
public:
	typedef ::math::Vec2 Vec2;
	explicit DisplaySimulatorCtrl(std::string const & name);
	virtual ~DisplaySimulatorCtrl();

	// Keymapping.xml helper
	virtual void saveSimScene(std::string const & fname);
	virtual void chooseModel();
	virtual void previousObstacleInScenery();
	virtual void nextObstacleInScenery();
	virtual void deleteSelectedObstacle();
	virtual void obsManipulLift();
	virtual void obsManipulIncreaseSizeFront();
	virtual void obsManipulDecreaseSize();
	virtual void obsManipulIncreaseSizeSideways();
	virtual void obsManipulDecreaseSizeSideways();
	virtual void obsManipulLower();
	virtual void obsManipulPushBackwards();
	virtual void obsManipulPushForward();
	virtual void obsManipulPushRight();
	virtual void obsManipulPushLeft();
	virtual void obsManipulRotateZAxis();
	virtual void obsManipulRotateZAxisN();
	// END Keymapping.xml helper

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	virtual osgGA::GUIEventHandler * getEventHandler() const {
		return const_cast<DisplaySimulatorCtrl *>(this);
	}

	virtual bool handle(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object *, osg::NodeVisitor *);
	virtual bool mouseMoveEvent(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object *, osg::NodeVisitor *);
	virtual bool mousePressEvent(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object *, osg::NodeVisitor *);
	virtual bool mouseReleaseEvent(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object *, osg::NodeVisitor *);

	bool pick(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, Vec2 & point);

	/** @name Registered Methods: */
	//@{

	//@}

	virtual void init3D(SceneNodePtr);
	virtual void draw3D(DrawArg);

protected:
	QMutex mMutex;
	boost::optional<Vec2> mMousePos;

	/** @name Properties: */
	//@{
    RTT::Property<bool> mEnableDisplay;
	//@}

	/** @name InputPorts: */
	//@{
	//@}

	RTT::TaskContext * mSimulator;

private:
	class impl;
	boost::shared_ptr<impl> pimpl;
};

}

}
 // namespace modules
}
 // namespace nav
}
 // namespace simulator
