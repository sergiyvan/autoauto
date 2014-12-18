#pragma once

#include <gui/Painter3DTask.h>
#include <modules/models/egostate/EgoState.h>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Property.hpp>
#include "DubinsRRTPlanGenerator.h"

/*!
 * \file DisplayDubinsRRTWaypoints.h
 * \brief TODO Brief description for DisplayDubinsRRTWaypoints.h
 */

// Definitions and forward declarations


namespace aa
{
namespace modules
{
namespace nav
{
namespace behaviour
{
/*!
* \class DisplayDubinsRRTWaypoints
* \brief Display a trajectory of one or two EgoState positions
*
* TODO Detailed description of DisplayDubinsRRTWaypoints.
*/
class DisplayDubinsRRTWaypoints
	: public gui::Painter3DTask, public osg::NodeCallback
{
public:
	// Definitions:

	// Constructors:
    explicit DisplayDubinsRRTWaypoints(std::string const & name);

	// Destructor:
    virtual ~DisplayDubinsRRTWaypoints();

protected:
	/** \name Ports: */
	/*! \{ */
	// Read ports:
    RTT::InputPort<std::vector<DubinsRRTWaypointPtr> > mWaypointsIn;
	// Write ports:

	/*! \} */
	/** \name Properties: */
	/*! \{ */
    ::math::Vec3 mPointColor;
    ::math::Vec3 mLineColor;
    ::math::Vec3 mLineColor2;
    ::math::Vec3 mLineColor3;
    float mTransparency;
    bool mDrawLines;
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

    osg::ref_ptr<osg::Geode> mPointGeode;
    osg::ref_ptr<osg::Geode> mLineGeode;
    osg::ref_ptr<osg::Geode> mLineGeode2;
    osg::ref_ptr<osg::Geode> mLineGeode3;

    // Functions:
};

}
}
}
}
