/*!
 * \file DisplayParticles.cpp
 * \brief TODO Brief description of DisplayParticles.
 */

#include "DisplayParticles.h"

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

REGISTERTASKCONTEXT(DisplayParticles);

//////////////////////////////////////////////////////
// Constructor:

DisplayParticles::DisplayParticles(std::string const & name)
	: gui::Painter3DTask(name)
	// TODO constructor
	// Read ports:
    , mParticlesIn("ParticlesIn")
	// Write ports:

	// Properties:
    , mColor(::math::Vec3(1.0, 0.0, 0.0))
    , mTransparency(0.5f)
    , mVertices(new osg::Vec3Array)
{
	// Read ports:

	// Event Ports:
    ports()->addPort(mParticlesIn);
	// Non-event ports:

	// Write ports

	// Properties:
    addProperty("Color", mColor);
    addProperty("Transparency", mTransparency);

	// Attributes:

	// Methods:

	// Commands:

}

//////////////////////////////////////////////////////
// Destructor:

DisplayParticles::~DisplayParticles()
{
}

//////////////////////////////////////////////////////
// TaskContext interface

bool DisplayParticles::startHook()
{
	return true;
}

void DisplayParticles::updateHook()
{
}

void DisplayParticles::stopHook()
{
}


////////////////////////////////////////////////////////
// Protected functions:

////////////////////////////////////////////////////////
// Private functions:


void DisplayParticles::init3D(SceneNodePtr root)
{

    mPointGeode = new osg::Geode;
    root->addChild(mPointGeode);
    mPointGeode->setUpdateCallback(this);
	mGeometry = new osg::Geometry();
    mPointGeode->addDrawable(mGeometry);
	mGeometry->setVertexArray(mVertices);
    mDrawArrayLines = new osg::DrawArrays(osg::PrimitiveSet::POINTS);
	mGeometry->addPrimitiveSet(mDrawArrayLines);
	osg::ref_ptr<osg::Vec4Array> colourArray = new osg::Vec4Array();
    colourArray->push_back(osg::Vec4(mColor(0), mColor(1), mColor(2), mTransparency));
	mGeometry->setColorArray(colourArray.get());
	mGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    osg::StateSet * stateSet = mPointGeode->getOrCreateStateSet();
	stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setAttribute( new osg::Point( 3.0f ), osg::StateAttribute::ON );



}

void DisplayParticles::operator()(osg::Node * node, osg::NodeVisitor * nv)
{
    std::vector<math::Vec3> particles;
    mParticlesIn.read(particles);




    //draw short line from point in direction of orientation
    mPointGeode->removeDrawables(0,mPointGeode->getNumDrawables());

    osg::ref_ptr<osg::Vec3Array> verticesP(new osg::Vec3Array());
    for (int i=0;i<particles.size();i++) {
        verticesP->push_back(osg::Vec3(particles[i][0], particles[i][1], 0));
        verticesP->push_back(osg::Vec3(particles[i][0]+cos(particles[i][2]), particles[i][1]+sin(particles[i][2]), 0));
    }
    osg::ref_ptr<osg::Geometry> geometryP = new osg::Geometry();
    geometryP->setVertexArray(verticesP);
    geometryP->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, verticesP->size()));
    osg::ref_ptr<osg::Vec4Array> colourArrayP = new osg::Vec4Array();
    colourArrayP->push_back(osg::Vec4(mColor(0), mColor(1), mColor(2), mTransparency));
    geometryP->setColorArray(colourArrayP.get());
    geometryP->setColorBinding(osg::Geometry::BIND_OVERALL);
    mPointGeode->addDrawable(geometryP);


    //draw points
    mVertices->clear();
    for (int i=0;i<particles.size();i++) {
        mVertices->push_back(osg::Vec3(particles[i][0], particles[i][1], 0));
    }
    mDrawArrayLines->setFirst(0);
    mDrawArrayLines->setCount(mVertices->size());
    mGeometry->setVertexArray(mVertices);

    mPointGeode->addDrawable(mGeometry);



}

}
}
}
}
