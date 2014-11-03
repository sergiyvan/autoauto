/*!
 * \file DisplayTrajectory.cpp
 * \brief TODO Brief description of DisplayTrajectory.
 */

#include "DisplayTrajectory.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <osg/Geode>
#include <osg/Array>
#include <osg/Geometry>

namespace aa
{
namespace modules
{
namespace display
{
namespace trajectory
{

REGISTERTASKCONTEXT(DisplayTrajectory);

//////////////////////////////////////////////////////
// Constructor:

DisplayTrajectory::DisplayTrajectory(std::string const & name)
	: gui::Painter3DTask(name)
	// TODO constructor
	// Read ports:
	, mEgoStateIn("EgoStateIn")
	// Write ports:

	// Properties:
	, mColor("Color", "trajectory color", ::math::Vec3(1.0, 0.0, 0.0))
	, mTransparency("Transparency", "transparency of trajectory", 0.5f)
	// Attributes:
	, mVertices(new osg::Vec3Array)
{
	// Read ports:

	// Event Ports:
	ports()->addPort(mEgoStateIn);
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

DisplayTrajectory::~DisplayTrajectory()
{
}

//////////////////////////////////////////////////////
// TaskContext interface

bool DisplayTrajectory::startHook()
{
	return true;
}

void DisplayTrajectory::updateHook()
{
}

void DisplayTrajectory::stopHook()
{
}


////////////////////////////////////////////////////////
// Protected functions:

////////////////////////////////////////////////////////
// Private functions:


void DisplayTrajectory::init3D(SceneNodePtr root)
{
	osg::ref_ptr<osg::Geode> trajectoryGeode = new osg::Geode;
	root->addChild(trajectoryGeode);
	trajectoryGeode->setUpdateCallback(this);
	mGeometry = new osg::Geometry();
	trajectoryGeode->addDrawable(mGeometry);
	mGeometry->setVertexArray(mVertices);
	mDrawArrayLines = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);
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
}

void DisplayTrajectory::operator()(osg::Node * node, osg::NodeVisitor * nv)
{
	if (mDrawArrayLines.valid()) {
		TimedEgoState ego;
		mEgoStateIn.read(ego);


		if (ego.after(mLast)) {
			mLast = ego;
			::math::Vec3 pos = ego.position();
			mVertices->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
			mDrawArrayLines->setFirst(0);
			mDrawArrayLines->setCount(mVertices->size());
			mGeometry->setVertexArray(mVertices);
		}
	}

	traverse(node, nv);
}

}
}
}
}