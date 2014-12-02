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
    , mPointColor(::math::Vec3(1.0, 0.0, 0.0))
    , mLineColor(::math::Vec3(0.0, 1.0, 0.0))
    , mTransparency(0.5f)
    , mDrawLines(true)
	// Attributes:
	, mVertices(new osg::Vec3Array)
{
	// Read ports:

	// Event Ports:
    ports()->addPort(mWaypointsIn);
	// Non-event ports:

	// Write ports

	// Properties:
    addProperty("PointColor", mPointColor);
    addProperty("LineColor", mLineColor);
    addProperty("Transparency", mTransparency);
    addProperty("DrawLines", mDrawLines);

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
    mLineGeode = new osg::Geode;
    root->addChild(mLineGeode);
    osg::StateSet * stateSet2 = mLineGeode->getOrCreateStateSet();
    stateSet2->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet2->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateSet2->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::Geode> trajectoryGeode = new osg::Geode;
	root->addChild(trajectoryGeode);
	trajectoryGeode->setUpdateCallback(this);
	mGeometry = new osg::Geometry();
	trajectoryGeode->addDrawable(mGeometry);
	mGeometry->setVertexArray(mVertices);
    mDrawArrayLines = new osg::DrawArrays(osg::PrimitiveSet::POINTS);
	mGeometry->addPrimitiveSet(mDrawArrayLines);
	osg::ref_ptr<osg::Vec4Array> colourArray = new osg::Vec4Array();
    colourArray->push_back(osg::Vec4(mPointColor(0), mPointColor(1), mPointColor(2), mTransparency));
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



    if (mDrawLines) {

        mLineGeode->removeDrawables(0,mLineGeode->getNumDrawables());

        for (int i=0;i<waypoints.size();i++) {
            if (waypoints[i]) {
                AStarWaypointPtr wp = waypoints[i];
                osg::ref_ptr<osg::Vec3Array> vertices(new osg::Vec3Array());

                while (wp) {
                    math::Vec3 pos = wp->position;
                    vertices->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
                    wp = wp->prevWaypoint;

                }

                osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
                geometry->setVertexArray(vertices);
                geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
                osg::ref_ptr<osg::Vec4Array> colourArray = new osg::Vec4Array();
                colourArray->push_back(osg::Vec4(mLineColor(0), mLineColor(1), mLineColor(2), mTransparency));
                geometry->setColorArray(colourArray.get());
                geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

                mLineGeode->addDrawable(geometry);

            }
        }

    }


    mVertices->clear();

    for (int i=0;i<waypoints.size();i++) {
        if (waypoints[i]) {
            math::Vec3 pos = waypoints[i]->position;
            mVertices->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
        }
    }

    mDrawArrayLines->setFirst(0);
    mDrawArrayLines->setCount(mVertices->size());
    mGeometry->setVertexArray(mVertices);



}

}
}
}
}
