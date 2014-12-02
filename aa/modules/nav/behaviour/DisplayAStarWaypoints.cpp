/*!
 * \file DisplayAStarWaypoints.cpp
 * \brief TODO Brief description of DisplayAStarWaypoints.
 */

#include "DisplayAStarWaypoints.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <osg/Geode>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/Point>


namespace aa
{
namespace modules
{
namespace nav
{
namespace behaviour
{

REGISTERTASKCONTEXT(DisplayAStarWaypoints);

//////////////////////////////////////////////////////
// Constructor:

DisplayAStarWaypoints::DisplayAStarWaypoints(std::string const & name)
	: gui::Painter3DTask(name)
	// TODO constructor
	// Read ports:
    , mWaypointsIn("WaypointsIn")
	// Write ports:

	// Properties:
	, mColor("Color", "trajectory color", ::math::Vec3(1.0, 0.0, 0.0))
	, mTransparency("Transparency", "transparency of trajectory", 0.5f)
	// Attributes:
	, mVertices(new osg::Vec3Array)
{
	// Read ports:

	// Event Ports:
    ports()->addPort(mWaypointsIn);
	// Non-event ports:

	// Write ports

	// Properties:
	addProperty(mColor);
	addProperty(mTransparency);
	// Attributes:

	// Methods:

	// Commands:

}

//////////////////////////////////////////////////////
// Destructor:

DisplayAStarWaypoints::~DisplayAStarWaypoints()
{
}

//////////////////////////////////////////////////////
// TaskContext interface

bool DisplayAStarWaypoints::startHook()
{
	return true;
}

void DisplayAStarWaypoints::updateHook()
{
}

void DisplayAStarWaypoints::stopHook()
{
}


////////////////////////////////////////////////////////
// Protected functions:

////////////////////////////////////////////////////////
// Private functions:


void DisplayAStarWaypoints::init3D(SceneNodePtr root)
{
	osg::ref_ptr<osg::Geode> trajectoryGeode = new osg::Geode;
	root->addChild(trajectoryGeode);
	trajectoryGeode->setUpdateCallback(this);
	mGeometry = new osg::Geometry();
	trajectoryGeode->addDrawable(mGeometry);
	mGeometry->setVertexArray(mVertices);
    mDrawArrayLines = new osg::DrawArrays(osg::PrimitiveSet::POINTS);
	mGeometry->addPrimitiveSet(mDrawArrayLines);
	osg::ref_ptr<osg::Vec4Array> colourArray = new osg::Vec4Array();
	::math::Vec3 color = mColor.rvalue();
	colourArray->push_back(osg::Vec4(color(0), color(1), color(2), mTransparency.rvalue()));
	mGeometry->setColorArray(colourArray.get());
	mGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	osg::StateSet * stateSet = trajectoryGeode->getOrCreateStateSet();
	stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
	stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setAttribute( new osg::Point( 3.0f ), osg::StateAttribute::ON );
}

void DisplayAStarWaypoints::operator()(osg::Node * node, osg::NodeVisitor * nv)
{
    std::vector<AStarWaypointPtr> waypoints;
    mWaypointsIn.read(waypoints);

    mVertices->clear();

    for (int i=0;i<waypoints.size();i++) {
        math::Vec3 pos = waypoints[i]->position;
        mVertices->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
    }

    mDrawArrayLines->setFirst(0);
    mDrawArrayLines->setCount(mVertices->size());
    mGeometry->setVertexArray(mVertices);

}

}
}
}
}
