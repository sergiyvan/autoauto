#pragma once

#include <gui/Painter3DTask.h>
#include <modules/models/egostate/EgoState.h>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Property.hpp>

/*!
 * \file DisplayParticles.h
 * \brief TODO Brief description for DisplayParticles.h
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
* \class DisplayParticles
* \brief Display a trajectory of one or two EgoState positions
*
* TODO Detailed description of DisplayParticles.
*/
class DisplayParticles
	: public gui::Painter3DTask, public osg::NodeCallback
{
public:
	// Definitions:

	// Constructors:
    explicit DisplayParticles(std::string const & name);

	// Destructor:
    virtual ~DisplayParticles();

protected:
	/** \name Ports: */
	/*! \{ */
	// Read ports:
    RTT::InputPort<std::vector< math::Vec3 > > mParticlesIn;
	// Write ports:

	/*! \} */
	/** \name Properties: */
	/*! \{ */
    ::math::Vec3 mColor;
    float mTransparency;
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
