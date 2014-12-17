#pragma once

#include <gui/Painter3DTask.h>
#include <modules/models/egostate/EgoState.h>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Property.hpp>
#include "RRTPlanGenerator.h"

/*!
 * \file DisplayRRTWaypoints.h
 * \brief TODO Brief description for DisplayRRTWaypoints.h
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
* \class DisplayRRTWaypoints
* \brief Display a trajectory of one or two EgoState positions
*
* TODO Detailed description of DisplayRRTWaypoints.
*/
class DisplayRRTWaypoints
	: public gui::Painter3DTask, public osg::NodeCallback
{
public:
	// Definitions:

	// Constructors:
    explicit DisplayRRTWaypoints(std::string const & name);

	// Destructor:
    virtual ~DisplayRRTWaypoints();

protected:
	/** \name Ports: */
	/*! \{ */
	// Read ports:
    RTT::InputPort<std::vector<RRTWaypointPtr> > mWaypointsIn;
	// Write ports:

	/*! \} */
	/** \name Properties: */
	/*! \{ */
    ::math::Vec3 mPointColor;
    ::math::Vec3 mLineColor;
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

    osg::ref_ptr<osg::Geode> mLineGeode;

    // Functions:
};

}
}
}
}
