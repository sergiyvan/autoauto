#include "DisplayControllerData.h"

#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glext.h>

#include <osg/Vec3>
#include <osg/Geode>
#include <osg/Texture2D>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/BlendFunc>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>

#include <data/VehicleData.h>
#include <math/AutoMath.h>
#include <util/OrocosHelperFunctions.h>
#include <util/TaskContextFactory.h>
#include <gui/GlTools.h>
#include <aa/modules/models/rndf/RndfGraph.h>
#include <aa/modules/nav/statemachine/StateMachine.h>

#define VERBOSE
#if defined(VERBOSE)
#define COUT(X)		std::cout << X << std::endl
#define DEBUG(X)	RTT::Logger::log() << RTT::Logger::Debug << X << RTT::Logger::endl
#define INFO(X)		RTT::Logger::log() << RTT::Logger::Info << X << RTT::Logger::endl
#define WARNING(X)	RTT::Logger::log() << RTT::Logger::Warning << X << RTT::Logger::endl
#define ERROR(X)	RTT::Logger::log() << RTT::Logger::Error << X << RTT::Logger::endl
#define CRITICAL(X)	RTT::Logger::log() << RTT::Logger::Critical << X << RTT::Logger::endl
#define FATAL(X)	RTT::Logger::log() << RTT::Logger::Fatal << X << RTT::Logger::endl
#else
#define COUT(X)		{}
#define DEBUG(X)	{}
#define INFO(X)		{}
#define WARNING(X)	{}
#define ERROR(X)	{}
#define CRITICAL(X)	{}
#define FATAL(X)	{}
#endif

namespace aa
{
namespace modules
{
namespace display
{
namespace controller
{

REGISTERTASKCONTEXT(DisplayControllerData);


static math::flt const MIN_Z_OFFSET = 0.005;


DisplayControllerData::DisplayControllerData(std::string const & name)
	: Painter3DTask(name)
	, mControllerDataIn("ControllerDataIn")

	, mNumControllerDataSamples("NumControllerDataSamples", "number of ControllerData samples to take for visualisation", 100)

	, mDisplayModelPredictionDataSet("DisplayModelPredictionDataSet", "display model prediction data", true)
	, mDisplayModelPredictionSupervisedSeries("DisplayModelPredictionSupervisedSeries", "display supervised series of model prediction data", true)

	, mDisplayProjectedPositions("DisplayProjectedPositions", "display projected vehicle positions", false)
	, mDisplayNumericValues("DisplayNumericValues", "display numeric values", false)

	, mCounter("Counter", 0)

	, mNumControllerData(0)
	, mControllerData()
{
	ports()->addEventPort(mControllerDataIn);

	addAttribute(mCounter);

	addProperty(mNumControllerDataSamples);

	addProperty(mDisplayModelPredictionDataSet);
	addProperty(mDisplayModelPredictionSupervisedSeries);

	addProperty(mDisplayProjectedPositions);
	addProperty(mDisplayNumericValues);
}

DisplayControllerData::~DisplayControllerData()
{
	stop();
}

bool DisplayControllerData::startHook()
{
	RTT::Logger::In in("DisplayControllerData");

	OPTIONAL_PORT(mControllerDataIn);

	return true;
}

void DisplayControllerData::updateHook()
{
	QMutexLocker lock(&mMutex);
	RTT::Logger::In in("DisplayControllerData");

	mCounter.set(mCounter.get() + 1);


	//read data
	RTT::FlowStatus status = mControllerDataIn.read(mControllerData);

	if (status == RTT::NewData) {
		//throttle diagram
		if (!std::isnan(mControllerData.speedCorrection)) {
			mLastSpeedCorrections.push_back(mControllerData.speedCorrection);
		}

		if (mLastSpeedCorrections.size() > mNumControllerDataSamples.get()) {
			mLastSpeedCorrections.erase(mLastSpeedCorrections.begin(), mLastSpeedCorrections.begin() + mLastSpeedCorrections.size() - mNumControllerDataSamples.get());
		}

		if (!std::isnan(mControllerData.idleZone.first) && !std::isnan(mControllerData.idleZone.second)) {
			mLastIdleZones.push_back(mControllerData.idleZone);
		}

		if (mLastIdleZones.size() > mNumControllerDataSamples.get()) {
			mLastIdleZones.erase(mLastIdleZones.begin(), mLastIdleZones.begin() + mLastIdleZones.size() - mNumControllerDataSamples.get());
		}

		if (!std::isnan(mControllerData.comfortZone.first) && !std::isnan(mControllerData.comfortZone.second)) {
			mLastComfortZones.push_back(mControllerData.comfortZone);
		}

		if (mLastComfortZones.size() > mNumControllerDataSamples.get()) {
			mLastComfortZones.erase(mLastComfortZones.begin(), mLastComfortZones.begin() + mLastComfortZones.size() - mNumControllerDataSamples.get());
		}

		if (!std::isnan(mControllerData.throttleWindow.first) && !std::isnan(mControllerData.throttleWindow.second)) {
			mLastThrottleWindows.push_back(mControllerData.throttleWindow);
		}

		if (mLastThrottleWindows.size() > mNumControllerDataSamples.get()) {
			mLastThrottleWindows.erase(mLastThrottleWindows.begin(), mLastThrottleWindows.begin() + mLastThrottleWindows.size() - mNumControllerDataSamples.get());
		}

		//steer diagram
		if (!std::isnan(mControllerData.steerCorrection)) {
			mLastSteerCorrections.push_back(mControllerData.steerCorrection);
		}

		if (mLastSteerCorrections.size() > mNumControllerDataSamples.get()) {
			mLastSteerCorrections.erase(mLastSteerCorrections.begin(), mLastSteerCorrections.begin() + mLastSteerCorrections.size() - mNumControllerDataSamples.get());
		}

		if (!std::isnan(mControllerData.steerWindow.first) && !std::isnan(mControllerData.steerWindow.second)) {
			mLastSteerWindows.push_back(mControllerData.steerWindow);
		}

		if (mLastSteerWindows.size() > mNumControllerDataSamples.get()) {
			mLastSteerWindows.erase(mLastSteerWindows.begin(), mLastSteerWindows.begin() + mLastSteerWindows.size() - mNumControllerDataSamples.get());
		}



		//lateral errors
		if (!std::isnan(mControllerData.lateralError)) {
			mLastLateralErrors.push_back(mControllerData.lateralError);
		}

		if (mLastLateralErrors.size() > mNumControllerDataSamples.get()) {
			mLastLateralErrors.erase(mLastLateralErrors.begin(), mLastLateralErrors.begin() + mLastLateralErrors.size() - mNumControllerDataSamples.get());
		}

		//heading error
		if (!std::isnan(mControllerData.headingError)) {
			mLastHeadingErrors.push_back(mControllerData.headingError);
		}

		if (mLastHeadingErrors.size() > mNumControllerDataSamples.get()) {
			mLastHeadingErrors.erase(mLastHeadingErrors.begin(), mLastHeadingErrors.begin() + mLastHeadingErrors.size() - mNumControllerDataSamples.get());
		}

		mNumControllerData++;
	}


}

void DisplayControllerData::stopHook()
{
	stop();
}

void DisplayControllerData::init3D(SceneNodePtr sceneNode)
{
	Painter3DTask::init3D(sceneNode);
// 	root = osg::ref_ptr<osg::Group>(new osg::Group());
// 	sceneNode->addChild(root.get());
	sceneNode->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	sceneNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF);
	setSupportsDisplayList(false);

// 	pyramidGeode = new osg::Geode();
// 	pyramidGeometry = new osg::Geometry();
//
// 	pyramidGeode->addDrawable(pyramidGeometry);
// 	root->addChild(pyramidGeode);
//
// 	srand(time(NULL));
}

void DisplayControllerData::draw3D(DrawArg viewer)
{
	QMutexLocker lock(&mMutex);

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	if (mNumControllerData > 1) {
		static math::flt const bottom = 0.70f;
		static math::flt const top = 0.99f;

		renderThrottleMeter(-0.99f, -0.72f, top, bottom);
		renderThrottleDiagram(-0.71f, -0.44f, top, bottom);
		renderSteerMeter(-0.43f, -0.16f, top, bottom);
		renderSteerDiagram(-0.15f, 0.12f, top, bottom);
		renderLateralError(0.13f, 0.40f, top, bottom);
		renderHeadingError(0.41f, 0.68f, top, bottom);
		renderSpeedInfo(0.69f, 0.99f, top, bottom);
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();


	if (mDisplayProjectedPositions.get()) {
		renderProjectedPositions();
	}

	glPopAttrib();
}

void DisplayControllerData::renderThrottleMeter(math::flt left, math::flt right, math::flt top, math::flt bottom) const
{
	glColor4f(0.0, 0.0, 0.0, 0.3);

	math::flt width = right - left;
	math::flt mid = (left + right) / 2;

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	math::flt r = width / 2.2f;

	glLineWidth(1);
	glColor4f(1.0, 1.0, 1.0, 0.5);
	glBegin(GL_LINE_LOOP);

	for (int i = 0; i <= 100; i++) {
		math::flt theta = math::PI * math::flt(i) / 100;
		math::flt x = r * cos(theta);
		math::flt y = r * sin(theta);
		glVertex3f(x + mid, y + bottom + 0.05, 0.0);
	}

	glEnd();

	glBegin(GL_LINES);
	glVertex3f(mid, bottom + 0.05, 0.0);
	glVertex3f(mid, bottom + 0.05 + r, 0.0);
	glEnd();

	glLineWidth(5);

	std::ostringstream o;
	o.precision(3);

	if (!std::isnan(mControllerData.throttlePositionMeasured)) {
		math::flt gamma = math::PI / 2 - mControllerData.throttlePositionMeasured * math::PI / 2;
		glColor4f(0.0, 1.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(mid, bottom + 0.05, 0.0);
		glVertex3f(r * cos(gamma) + mid, r * sin(gamma) + bottom + 0.05, 0.0);
		glEnd();

		if (mDisplayNumericValues.get()) {
			o.str("");
			o << mControllerData.throttlePositionMeasured;
			gui::renderString(o.str(), left + 0.01, bottom + 0.2, 0.0, 0.0, 1.0, 0.0, 1.0, GLUT_BITMAP_HELVETICA_10);
		}
	}

	if (!std::isnan(mControllerData.speedCorrection)) {
		glLineWidth(5);
		glColor4f(1.0, 1.0, 1.0, 1.0);
		math::flt alpha = math::PI / 2 - mControllerData.speedCorrection * math::PI / 2;
		glBegin(GL_LINES);
		glVertex3f(mid, bottom + 0.05, 0.0);
		glVertex3f(r * cos(alpha) + mid, r * sin(alpha) + bottom + 0.05, 0.0);
		glEnd();

		if (mDisplayNumericValues.get()) {
			o.str("");
			o << mControllerData.speedCorrection;
			gui::renderString(o.str(), right - 0.065f, bottom + 0.2, 0.0, 1.0, 1.0, 1.0, 1.0, GLUT_BITMAP_HELVETICA_10);
		}
	}

	gui::renderString("Throttle:", left + 0.01, top - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("-1.0", left + 0.01, bottom + 0.02, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("+1.0", right - 0.05f, bottom + 0.02, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);

}

void DisplayControllerData::renderThrottleDiagram(math::flt left, math::flt right, math::flt top, math::flt bottom) const
{
	glColor4f(0.0, 0.0, 0.0, 0.3);

	math::flt width = right - left;
	math::flt height = top - bottom;

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	math::flt min = -1.0f;
	math::flt max = 1.0f;
	math::flt hOffset = 0.05;
	math::flt wOffset = 0.05;
	math::flt midH = (top + bottom - hOffset) / 2;

	math::flt padding = 0.005;

	glLineWidth(1.0);

	glColor4f(1.0, 1.0, 1.0, 0.5);
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, top - 0.01 - hOffset, 0.0);
	glVertex3f(left + wOffset + padding, bottom + 0.01, 0.0);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, midH, 0.0);
	glVertex3f(right - padding, midH, 0.0);
	glEnd();

	glColor4f(0.5, 0.5, 0.5, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastIdleZones.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastIdleZones.size()), midH + math::rangeCut(min, mLastIdleZones[i].first, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();
	glBegin(GL_LINES);

	for (int i = 0; i < mLastIdleZones.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastIdleZones.size()), midH + math::rangeCut(min, mLastIdleZones[i].second, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();

	glColor4f(1.0, 1.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastComfortZones.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastComfortZones.size()), midH + math::rangeCut(min, mLastComfortZones[i].first, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();
	glBegin(GL_LINES);

	for (int i = 0; i < mLastComfortZones.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastComfortZones.size()), midH + math::rangeCut(min, mLastComfortZones[i].second, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();

	glColor4f(1.0, 0.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastThrottleWindows.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastThrottleWindows.size()), midH + math::rangeCut(min, mLastThrottleWindows[i].first, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();
	glBegin(GL_LINES);

	for (int i = 0; i < mLastThrottleWindows.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastThrottleWindows.size()), midH + math::rangeCut(min, mLastThrottleWindows[i].second, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();

	glColor4f(0.0, 1.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastSpeedCorrections.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastSpeedCorrections.size()), midH + math::rangeCut(min, mLastSpeedCorrections[i], max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();

	gui::renderString("Throttle:", left + 0.01, top - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("+1.0", left + 0.01, top - hOffset - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("-1.0", left + 0.01, bottom + 0.01, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
}

void DisplayControllerData::renderSteerMeter(math::flt left, math::flt right, math::flt top, math::flt bottom) const
{
	glColor4f(0.0, 0.0, 0.0, 0.3);

	math::flt width = right - left;
	math::flt mid = (left + right) / 2;

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	math::flt r = width / 2.2f;

	glLineWidth(1);
	glColor4f(1.0, 1.0, 1.0, 0.5);
	glBegin(GL_LINE_LOOP);

	for (int i = 0; i <= 100; i++) {
		math::flt theta = math::PI * math::flt(i) / 100;
		math::flt x = r * cos(theta);
		math::flt y = r * sin(theta);
		glVertex3f(x + mid, y + bottom + 0.05, 0.0);
	}

	glEnd();

	glBegin(GL_LINES);
	glVertex3f(mid, bottom + 0.05, 0.0);
	glVertex3f(mid, bottom + 0.05 + r, 0.0);
	glEnd();

	glLineWidth(5);

	std::ostringstream o;
	o.precision(3);

	if (!std::isnan(mControllerData.steeringAngleMeasured)) {
		math::flt const maxSteer = ::data::theVehicleData::instance().getPropertyType<math::flt>("maxSteer")->value();		//maximal angle of wheel in radian

		math::flt gamma = math::PI / 2 - mControllerData.steeringAngleMeasured / maxSteer * math::PI / 2;
		glColor4f(0.0, 1.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(mid, bottom + 0.05, 0.0);
		glVertex3f(r * cos(gamma) + mid, r * sin(gamma) + bottom + 0.05, 0.0);
		glEnd();


		if (mDisplayNumericValues.get()) {
			o.str("");
			o << (mControllerData.steeringAngleMeasured  / maxSteer);
			gui::renderString(o.str(), left + 0.01, bottom + 0.2, 0.0, 0.0, 1.0, 0.0, 1.0, GLUT_BITMAP_HELVETICA_10);
		}
	}

	if (!std::isnan(mControllerData.steerCorrection)) {
		math::flt beta = math::PI / 2 - mControllerData.steerCorrection * math::PI / 2;
		glColor4f(1.0, 1.0, 1.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(mid, bottom + 0.05, 0.0);
		glVertex3f(r * cos(beta) + mid, r * sin(beta) + bottom + 0.05, 0.0);
		glEnd();

		if (mDisplayNumericValues.get()) {
			o.str("");
			o << mControllerData.steerCorrection;
			gui::renderString(o.str(), right - 0.08, bottom + 0.2, 0.0, 1.0, 1.0, 1.0, 1.0, GLUT_BITMAP_HELVETICA_10);
		}
	}

	gui::renderString("Steering:", left + 0.01, top - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("-1.0", left + 0.01, bottom + 0.02, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("+1.0", right - 0.06f, bottom + 0.02, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
}


void DisplayControllerData::renderSteerDiagram(math::flt left, math::flt right, math::flt top, math::flt bottom) const
{
	glColor4f(0.0, 0.0, 0.0, 0.3);

	math::flt width = right - left;
	math::flt height = top - bottom;

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	math::flt min = -1.0;
	math::flt max = 1.0;
	math::flt hOffset = 0.05;
	math::flt wOffset = 0.05;
	math::flt midH = (top + bottom - hOffset) / 2;

	math::flt padding = 0.005;

	glLineWidth(1.0);

	glColor4f(1.0, 1.0, 1.0, 0.5);
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, top - 0.01 - hOffset, 0.0);
	glVertex3f(left + wOffset + padding, bottom + 0.01, 0.0);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, midH, 0.0);
	glVertex3f(right - padding, midH, 0.0);
	glEnd();


	glColor4f(1.0, 0.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastSteerWindows.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastSteerWindows.size()), midH + math::rangeCut(min, mLastSteerWindows[i].first, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();
	glBegin(GL_LINES);

	for (int i = 0; i < mLastSteerWindows.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastSteerWindows.size()), midH + math::rangeCut(min, mLastSteerWindows[i].second, max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();

	glColor4f(0.0, 1.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastSteerCorrections.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastSteerCorrections.size()), midH + math::rangeCut(min, mLastSteerCorrections[i], max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();

	gui::renderString("Steering:", left + 0.01, top - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("+1.0", left + 0.01, top - hOffset - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("-1.0", left + 0.01, bottom + 0.01, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
}

void DisplayControllerData::renderLateralError(math::flt left, math::flt right, math::flt top, math::flt bottom) const
{
	glColor4f(0.0, 0.0, 0.0, 0.3);

	math::flt width = right - left;
	math::flt height = top - bottom;

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	math::flt min = -5.0f;
	math::flt max = 5.0f;
	math::flt hOffset = 0.05;
	math::flt wOffset = 0.05;
	math::flt midH = (top + bottom - hOffset) / 2;

	math::flt padding = 0.005;

	glLineWidth(1.0);

	glColor4f(1.0, 1.0, 1.0, 0.5);
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, top - 0.01 - hOffset, 0.0);
	glVertex3f(left + wOffset + padding, bottom + 0.01, 0.0);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, midH, 0.0);
	glVertex3f(right - padding, midH, 0.0);
	glEnd();

	glColor4f(0.0, 1.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastLateralErrors.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastLateralErrors.size()), midH + math::rangeCut(min, mLastLateralErrors[i], max) / (max - min) * (height - hOffset - 0.02), 0.0);
	}

	glEnd();

	gui::renderString("Lateral Error:", left + 0.01, top - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("+5.0", left + 0.01, top - hOffset - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("-5.0", left + 0.01, bottom + 0.01, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
}

void DisplayControllerData::renderHeadingError(math::flt left, math::flt right, math::flt top, math::flt bottom) const
{
	glColor4f(0.0, 0.0, 0.0, 0.3);

	math::flt width = right - left;
	math::flt height = top - bottom;

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	math::flt min = -math::PI / 2;
	math::flt max = math::PI / 2;
	math::flt hOffset = 0.05;
	math::flt wOffset = 0.055;
	math::flt midH = (top + bottom - hOffset) / 2;

	math::flt padding = 0.005;

	glLineWidth(1.0);

	glColor4f(1.0, 1.0, 1.0, 0.5);
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, top - 0.01 - hOffset, 0.0);
	glVertex3f(left + wOffset + padding, bottom + 0.01, 0.0);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(left + wOffset + padding, midH, 0.0);
	glVertex3f(right - padding, midH, 0.0);
	glEnd();

	glColor4f(0.0, 1.0, 0.0, 1.0);
	glBegin(GL_LINES);

	for (int i = 0; i < mLastHeadingErrors.size(); i++) {
		glVertex3f(left + wOffset + padding + (width - wOffset - 2 * padding) * ((math::flt)i / (math::flt)mLastHeadingErrors.size()), midH + math::rangeCut(min, mLastHeadingErrors[i], max) / (max - min) * (height - hOffset - padding), 0.0);
	}

	glEnd();

	gui::renderString("Heading Error:", left + 0.01, top - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("+PI/2", left + 0.01, top - hOffset - 0.03, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
	gui::renderString("-PI/2", left + 0.01, bottom + 0.01, 0.0, 1.0, 1.0, 1.0, 0.6, GLUT_BITMAP_HELVETICA_10);
}

void DisplayControllerData::renderSpeedInfo(math::flt left, math::flt right, math::flt top, math::flt bottom) const
{
	glColor4f(0.0, 0.0, 0.0, 0.3);

	math::flt width = right - left;
	math::flt mid = (left + right) / 2;

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	std::ostringstream o;
	o.precision(3);

	//draw wanted
	o.str("");

	if (std::isnan(mControllerData.wantedSpeed)) {
		o << "want: -";
	}
	else if (mControllerData.wantedSpeed < math::epsilon) {
		o << "want: " << 0;
	}
	else {
		o << "want: " << std::max(0.0, mControllerData.wantedSpeed * math::MS_2_KMH);
	}

	o << " (" << ::modules::models::carstate::CarState::gearString(mControllerData.gearCorrection) << ")";
	gui::renderString(o.str(), left + 0.02, top - 0.08f, 0.0, 1.0, 1.0, 1.0, 1.0, GLUT_BITMAP_HELVETICA_18);


	//draw current
	o.str("");

	if (std::isnan(mControllerData.curSpeed)) {
		o << "cur: -";
	}
	else if (mControllerData.curSpeed < math::epsilon) {
		o << "cur: " << 0;
	}
	else {
		o << "cur: " << std::max(0.0, mControllerData.curSpeed * math::MS_2_KMH);
	}

	o << " (" << ::modules::models::carstate::CarState::gearString(mControllerData.gearPosition) << ")";
	gui::renderString(o.str(), left + 0.02, top - 0.16f, 0.0, 0.0, 1.0, 0.0, 1.0, GLUT_BITMAP_HELVETICA_18);



	//draw limit
	o.str("");
	o.precision(3);

	if (std::isnan(mControllerData.maxSpeedLimit)) {
		o << "limit: -";
	}
	else {
		o << "limit: " << std::max(0.0, round(mControllerData.maxSpeedLimit * math::MS_2_KMH));
	}

	gui::renderString(o.str(), left + 0.02, top - 0.24f, 0.0, 1.0, 0.0, 0.0, 1.0, GLUT_BITMAP_HELVETICA_18);
}


void DisplayControllerData::renderProjectedPositions() const
{
	gui::renderSolidCircle(Vec3(mControllerData.projectedFrontAxlePos[0], mControllerData.projectedFrontAxlePos[1], 0.0), 0.15, 0.15, Vec3(1, 1, 1), 0.1);       //front axle white (control point)
	gui::renderSolidCircle(Vec3(mControllerData.projectedRearAxlePos[0], mControllerData.projectedRearAxlePos[1], 0.0), 0.15, 0.15, Vec3(0, 0, 0), 0.1);         //rear axle black
	gui::renderSolidCircle(Vec3(mControllerData.projectedFrontTipPos[0], mControllerData.projectedFrontTipPos[1], 0.0), 0.15, 0.15, Vec3(0, 1, 1), 0.1);         //front teal
	gui::renderSolidCircle(Vec3(mControllerData.projectedBackTipPos[0], mControllerData.projectedBackTipPos[1], 0.0), 0.15, 0.15, Vec3(1, 1, 0), 0.1);           //back gelb

}
}
}
}
}
