#include "DisplayMicroPlan.h"

#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/range/size.hpp>

#include <gui/GlTools.h>
#include <gui/GlDispatch.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/BlendFunc>

#include <util/OrocosHelperFunctions.h>
#include <util/TaskContextFactory.h>
#include <math/DiffGeom.h>
#include <math/AutoMath.h>
#include <data/VehicleData.h>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>


#include <GL/gl.h>
#include <GL/glu.h>

#include <osg/Point>
#include <osg/Projection>
#include <osg/MatrixTransform>

#include <gui/GlTools.h>


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

#define MIN_Z_OFFSET	0.005

namespace aa
{
namespace modules
{
namespace display
{
namespace behaviour
{


using namespace std;
using namespace RTT;
using namespace boost;
using namespace gui;
using namespace ::math;
using RTT::Logger;


REGISTERTASKCONTEXT(DisplayMicroPlan);

typedef patterns::Singleton< ::aa::modules::models::rndf::RNDFGraph> theRNDFGraph;



DisplayMicroPlan::DisplayMicroPlan(string const & name)
	: Painter3DTask(name)

	, mPlanIn("PlanIn")

	, mNumSamples("NumSamples", "number of samples", 200)
	, mMinSamples("MinSamples", "minimum number of samples", 100)
	, mMaxSamples("MaxSamples", "maximum number of samples", 5000)
	, mDrawDottedLines("DrawDottedLines", "draw dotted lines", true)
	, mDrawGLFlatMode("DrawGLFlatMode", "draw in OpenGL flat shade mode", false)

{
	ports()->addEventPort(mPlanIn);

	addProperty(mNumSamples);
	addProperty(mMinSamples);
	addProperty(mMaxSamples);
	addProperty(mDrawDottedLines);
	addProperty(mDrawGLFlatMode);

}

DisplayMicroPlan::~DisplayMicroPlan()
{

	stop();

}

void DisplayMicroPlan::init3D(SceneNodePtr sceneNode)
{
	Painter3DTask::init3D(sceneNode);
	sceneNode->getOrCreateStateSet()->setRenderBinDetails(1000, "RenderBin");
	sceneNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF);
	setSupportsDisplayList(false);

}

void DisplayMicroPlan::draw3D(DrawArg)
{
	QMutexLocker lock(&mMutex);

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();


	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
// 	glTranslatef(0, 0, 0.05);

	if (mDrawGLFlatMode.get()) {
		glShadeModel(GL_FLAT);
	}
	else {
		glShadeModel(GL_SMOOTH);
	}

	glDisable(GL_DEPTH_TEST);


    if (!mPlanSampled.empty()) {
		displaySelectedPlan();
	}


	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glDisable(GL_DEPTH_TEST);


	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glPopAttrib();
}


void DisplayMicroPlan::displaySelectedPlan() const
{
	if (mPlanSampled.size() > 0) {
		glLineWidth(6.f);

		glBegin(GL_LINE_STRIP);

		for (uint i = 0; i < mPlanSampled.size(); i++) {
			if (mDrawDottedLines.get() && i % 2 == 0) {
				glColor4f(mPlanColorSampled[i][0] + 0.5f, mPlanColorSampled[i][1] + 0.5f, mPlanColorSampled[i][2] + 0.5f, 1.f);
			}
			else {
				glColor4f(mPlanColorSampled[i][0], mPlanColorSampled[i][1], mPlanColorSampled[i][2], 0.8f);
			}

			glVertex2f(mPlanSampled[i][0], mPlanSampled[i][1]);
		}

		glEnd();

//		if (mDisplayParam.get()) {
//			char buffer[100];
//			sprintf(buffer, "%0.1f", mPlanDomain.first);
//			renderString(buffer, mPlanSampled.front()[0] + 4.f, mPlanSampled.front()[1] - 1.f, 0.05f, GLUT_BITMAP_HELVETICA_10);

//			sprintf(buffer, "%0.1f", mPlanDomain.second);
//			renderString(buffer, mPlanSampled.back()[0] + 4.f, mPlanSampled.back()[1] - 1.f, 0.05f, GLUT_BITMAP_HELVETICA_10);
//		}


		//display first sampled point
		renderSolidCircle(Vec3(mPlanSampled.front()[0], mPlanSampled.front()[1], 0.f), 0.15f, 0.15f, Vec3(1, 1, 0), 0.1);

	}
}





bool DisplayMicroPlan::startHook()
{
    Logger::In in("DisplayMicroPlan");

    REQUIRED_PORT(mPlanIn);

	return true;
}


void DisplayMicroPlan::updateHook()
{
	QMutexLocker lock(&mMutex);
    Logger::In in("DisplayMicroPlan");


	//got something from mPlanIn?
    // sample Trajectory
    aa::modules::nav::controller::Plan_ptr timedPlan;
    RTT::FlowStatus status = mPlanIn.read(timedPlan);

    if (status == RTT::NewData) {
        if (timedPlan) {
            aa::modules::nav::controller::Plan const & tmPlan = *timedPlan;

            if (!tmPlan.empty()) {
                std::pair<flt, flt> const dom(tmPlan.domain());

//						INFO("got Trajectory " << dom.first << " " << dom.second);

                // clear old sample
                mPlanSampled.clear();
                mPlanColorSampled.clear();
                mPlanSampled = samplePlan(timedPlan);
                mPlanColorSampled = colorsamplePlan(timedPlan);

                mPlanDomain = dom;

            }
        }
    }


}


void DisplayMicroPlan::stopHook()
{
	stop();
}


vector<Vec3> DisplayMicroPlan::samplePlan(aa::modules::nav::controller::Plan_ptr plan) const
{
    vector<Vec3> result;

	if (!plan) {
		return result;
	}

	aa::modules::nav::controller::Plan const & curPlan = *plan;

	if (curPlan.empty()) {
		return result;
	}

	std::pair<flt, flt> const dom(curPlan.domain());
	flt const startParam = dom.first;
	flt const endParam = dom.second;

    flt const samples = rangeCut<flt>(mMinSamples.get(), mNumSamples.get(), mMaxSamples.get());
	flt const sampleStep = (endParam - startParam) / (samples - 1);

	for (uint t = 0; t < samples; t++) {
		flt const curParam = startParam + sampleStep * t;

		if (startParam <= curParam && curParam <= endParam) {
			result.push_back(curPlan(curParam));
		}
	}

	return result;
}

vector<Vec3> DisplayMicroPlan::colorsamplePlan(aa::modules::nav::controller::Plan_ptr plan) const
{
	vector<Vec3> result;

	if (!plan) {
		return result;
	}

	aa::modules::nav::controller::Plan const & curPlan = *plan;

	if (curPlan.empty()) {
		return result;
	}

	std::pair<flt, flt> const dom(curPlan.domain());
	flt const startParam = dom.first;
	flt const endParam = dom.second;

    flt const samples = rangeCut<flt>(mMinSamples.get(), mNumSamples.get(), mMaxSamples.get());
	flt const sampleStep = (endParam - startParam) / (samples - 1);

	flt const turningCircleRadius = ::data::theVehicleData::instance().getPropertyType<flt>("turningCircleRadius")->value();

	for (uint t = 0; t < samples; t++) {
		flt const curParam = startParam + sampleStep * t;

		if (startParam <= curParam && curParam <= endParam) {
			flt radius = ::math::radius(curPlan, curParam);
            flt r = rangeCut<flt>(0.f, 2.f * turningCircleRadius / radius, 1.f);			//14 meter (and smaller) radius is red
			result.push_back(Vec3(r, 1.f - r, 0.f));
		}
	}

	return result;
}




void DisplayMicroPlan::plotCurvature()
{
	aa::modules::nav::controller::Plan_ptr timedPlan;
	mPlanIn.read(timedPlan);


	if (!timedPlan) {
		return;
	}

	aa::modules::nav::controller::Plan const & plan = *timedPlan;

	if (plan.empty()) {
		return;
	}


	std::pair<flt, flt> const dom(plan.domain());
	flt const startParam = dom.first;
	flt const endParam = dom.second;

    flt const samples = rangeCut<flt>(mMinSamples.get(), mNumSamples.get(), mMaxSamples.get());
	flt const sampleStep = (endParam - startParam) / (samples - 1);

	for (uint t = 0; t < samples; t++) {
		flt const curParam = startParam + sampleStep * t;

		if (startParam <= curParam && curParam <= endParam) {
			flt radius = ::math::radius(plan, curParam);
			COUT("p: " << curParam - startParam << " r: " << radius);
		}
	}
}


}
}
}
}
