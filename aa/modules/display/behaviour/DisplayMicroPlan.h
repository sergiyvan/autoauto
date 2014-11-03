#pragma once

#include <rtt/base/PortInterface.hpp>
#include <gui/Painter3DTask.h>
#include <QMutex>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math/Quad2d.h>

#include <aa/modules/nav/controller/Plan.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace behaviour
{

/*!	\ingroup SourceModules
	@{
    \class DisplayMicroPlan
 *   	\brief DisplayMicroPlan module.
 *	Source module which displays the trajectory and some information gathered by all checkers .
 */
class DisplayMicroPlan
	: public gui::Painter3DTask
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;




    explicit DisplayMicroPlan(std::string const & name);
    virtual ~DisplayMicroPlan();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void draw3D(DrawArg);
	virtual void init3D(SceneNodePtr scene);

protected:
	void displaySelectedPlan() const;

	/** @name InputPorts: */
	//@{
	RTT::InputPort< aa::modules::nav::controller::Plan_ptr > mPlanIn;
	//@}

	/** @name Properties: */
	//@{
	RTT::Property<int> mNumSamples;
	RTT::Property<int> mMinSamples;
	RTT::Property<int> mMaxSamples;
	RTT::Property<bool> mDrawDottedLines;
	RTT::Property<bool> mDrawGLFlatMode;
	//@}


	void plotCurvature();



private:

	std::pair<flt, flt> mPlanDomain;															//domain of plan
    std::vector< Vec3 > mPlanSampled;															//vector of sampled points
	std::vector< Vec3 > mPlanColorSampled;														//vector of sampled colors

    QMutex mMutex;

    std::vector<Vec3> samplePlan(aa::modules::nav::controller::Plan_ptr plan) const;
	std::vector<Vec3> colorsamplePlan(aa::modules::nav::controller::Plan_ptr plan) const;


};


}
}
}
}
