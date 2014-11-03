#pragma once
/*!
 * \file DisplayVirtualRangeScan.h
 * \brief Display virtual range scan.
 * \author Michael Schnürmacher
 */

#include <boost/smart_ptr.hpp>
#include <core/TimedData.h>
#include <gui/Painter3DTask.h>
#include <math/AutoMath.h>
#include <qmutex.h>
#include <aa/data/obstacle/BaseObjectBundle.h>
#include <data/geometry/BoundingBox2D.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace obstacles
{


/*!
* \class DisplayVirtualRangeScan
* \brief Display virtual range scan.
* \author Michael Schnürmacher
*
* TODO Detailed description of DisplayVirtualRangeScan.
*/
class DisplayObjects
	: public gui::Painter3DTask
{
public:
	// Definitions:
	typedef ::math::Vec3 Vec3;
	typedef ::math::Vec2 Vec2;
	typedef ::math::flt flt;
	typedef boost::shared_ptr<DisplayObjects> Ptr;
	typedef TimedData<DisplayObjects> Timed;
	typedef boost::shared_ptr<Timed> TimedPtr;
	typedef TimedData<Ptr> PtrTimed;

	// Constructors:
	explicit DisplayObjects(std::string const & name);

	// Destructor:
	virtual ~ DisplayObjects();

	/** \name Ports: */
	/** { */
	// Inports
	RTT::InputPort<TimedBaseObjectBundle_ptr> mBaseObjectsIn;
	/** } */


	/** \name Properties: */
	/** { */
	// Display Properties
	bool mDisplayBoundingBoxes;
	bool mDisplayPositionCovariances;
	bool mDisplayVelocityCovariances;
	bool mDisplayVelocityVectors;
	flt mPosColorR;
	flt mPosColorG;
	flt mPosColorB;
	flt mVelColorR;
	flt mVelColorG;
	flt mVelColorB;
	// Port Counter
	uint mBaseObjectsInCounter;
	/** } */

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void draw3D(gui::Painter3D::DrawArg);



protected:

	TimedBaseObjectBundle_ptr mBaseObjectsPtr;

	std::vector<Vec2> mPosCovVecs1;
	std::vector<Vec2> mPosCovVecs2;
	std::vector< std::vector<Vec2> > mPosCovEllipse;

	std::vector<Vec2> mVelCovVecs1;
	std::vector<Vec2> mVelCovVecs2;
	std::vector< std::vector<Vec2> > mVelCovEllipse;


private:

	QMutex mMutex;

};

}
}
}
}
