#pragma once

#include <rtt/base/PortInterface.hpp>
#include <gui/Painter3DTask.h>
#include <QMutex>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math/AutoMath.h>

#include <aa/modules/nav/controller/Plan.h>
#include <aa/modules/nav/controller/data/ControllerData.h>

#include <aa/modules/nav/simulator/Simulator.h>
#include <aa/modules/models/rndf/RndfGraph.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace controller
{


/*!	\ingroup SourceModules
	@{
	\class DisplayController
 *   	\brief DisplayController module.
 *	Source module which displays the information about the controller
 */
class DisplayControllerData
	: public gui::Painter3DTask
{
	typedef TimedData<aa::modules::nav::controller::data::ControllerData> TimedControllerData;

public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;
	explicit DisplayControllerData(std::string const & name);
	virtual ~DisplayControllerData();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	virtual void draw3D(DrawArg);
	virtual void init3D(SceneNodePtr scene);

protected:

	/** @name InputPorts: */
	//@{
	RTT::InputPort<TimedControllerData> mControllerDataIn;
	//@}

	/** @name Attributes: general attributes */
	RTT::Attribute<int> mCounter;


	/** @name Properties: general properties */
	RTT::Property<unsigned int> mNumControllerDataSamples;

	RTT::Property<bool> mDisplayModelPredictionDataSet;
	RTT::Property<bool> mDisplayModelPredictionSupervisedSeries;

	RTT::Property<bool> mDisplayProjectedPositions;
	RTT::Property<bool> mDisplayNumericValues;


private:
	void renderThrottleMeter(flt left, flt right, flt top, flt bottom) const;
	void renderThrottleDiagram(flt left, flt right, flt top, flt bottom) const;
	void renderSteerMeter(flt left, flt right, flt top, flt bottom) const;
	void renderSteerDiagram(flt left, flt right, flt top, flt bottom) const;
	void renderLateralError(flt left, flt right, flt top, flt bottom) const;
	void renderHeadingError(flt left, flt right, flt top, flt bottom) const;
	void renderSpeedInfo(flt left, flt right, flt top, flt bottom) const;

	void renderProjectedPositions() const;

	TimedControllerData mControllerData;

	int mNumControllerData;

	std::vector<flt> mLastSpeedCorrections;
	std::vector< std::pair<flt, flt> > mLastIdleZones;
	std::vector< std::pair<flt, flt> > mLastComfortZones;
	std::vector< std::pair<flt, flt> > mLastThrottleWindows;

	std::vector<flt> mLastSteerCorrections;
	std::vector< std::pair<flt, flt> > mLastSteerWindows;

	std::vector<flt> mLastLateralErrors;
	std::vector<flt> mLastHeadingErrors;


	QMutex mMutex;
};
}
}
}
}
