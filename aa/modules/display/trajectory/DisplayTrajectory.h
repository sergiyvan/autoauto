#pragma once

#include <gui/Painter3DTask.h>
#include <modules/models/egostate/EgoState.h>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Property.hpp>

/*!
 * \file DisplayTrajectory.h
 * \brief TODO Brief description for DisplayTrajectory.h
 */

// Definitions and forward declarations


namespace aa
{
namespace modules
{
namespace display
{
namespace trajectory
{
/*!
* \class DisplayTrajectory
* \brief Display a trajectory of one or two EgoState positions
*
* TODO Detailed description of DisplayTrajectory.
*/
class DisplayTrajectory
	: public gui::Painter3DTask, public osg::NodeCallback
{
public:
	// Definitions:

	// Constructors:
	explicit DisplayTrajectory(std::string const & name);

	// Destructor:
	virtual ~DisplayTrajectory();

protected:
	/** \name Ports: */
	/*! \{ */
	// Read ports:
	RTT::InputPort<TimedEgoState> mEgoStateIn;
	// Write ports:

	/*! \} */
	/** \name Properties: */
	/*! \{ */
	RTT::Property< ::math::Vec3> mColor;
	RTT::Property<float> mTransparency;
	/*! \} */


	// Functions:
	virtual bool startHook();

	virtual void updateHook();

	virtual void stopHook();

	void init3D(SceneNodePtr);

	virtual void operator()(osg::Node * node, osg::NodeVisitor * nv);

	// Other attributes:

private:
	// Attributes:
	TimeStamp mLast;
	osg::ref_ptr<osg::DrawArrays> mDrawArrayLines;
	osg::ref_ptr<osg::Geometry> mGeometry;
	osg::ref_ptr<osg::Vec3Array> mVertices;
    bool reset;
	// Functions:
    void resetTrajectory();
};

}
}
}
}
