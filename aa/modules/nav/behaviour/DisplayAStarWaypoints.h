#pragma once

#include <gui/Painter3DTask.h>
#include <modules/models/egostate/EgoState.h>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Property.hpp>
#include "AStarPlanGenerator.h"

/*!
 * \file DisplayAStarWaypoints.h
 * \brief TODO Brief description for DisplayAStarWaypoints.h
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
* \class DisplayAStarWaypoints
* \brief Display a trajectory of one or two EgoState positions
*
* TODO Detailed description of DisplayAStarWaypoints.
*/
class DisplayAStarWaypoints
	: public gui::Painter3DTask, public osg::NodeCallback
{
public:
	// Definitions:

	// Constructors:
    explicit DisplayAStarWaypoints(std::string const & name);

	// Destructor:
    virtual ~DisplayAStarWaypoints();

protected:
	/** \name Ports: */
	/*! \{ */
	// Read ports:
    RTT::InputPort<std::vector<AStarWaypointPtr> > mWaypointsIn;
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
