#include "DisplayStateMachine.h"

#include <rtt/Logger.hpp>
#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <util/Rtti.h>
#include <boost/foreach.hpp>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <gui/GlTools.h>

#include <aa/modules/nav/statemachine/StateMachine.h>

#include <QWidget>
#include <gui/KeyBindings.h>

#define VERBOSE
#if defined(VERBOSE)
#define COUT(X)		std::cout << X << std::endl
#define DEBUG(X)	RTT::Logger::log() << RTT::Logger::Debug << X << RTT::Logger::endl
#define INFO(X)		RTT::Logger::log() << RTT::Logger::Info << X << RTT::Logger::endl
#define WARNING(X)	RTT::Logger::log() << RTT::Logger::Warning << X << RTT::Logger::endl
#define ERROR(X)	RTT::Logger::log() << RTT::Logger::Error << X << RTT::Logger::endl
#define CRITICAL(X)	RTT::Logger::log() << RTT::Logger::Critical << X << RTT::Logger::endl
#define FATAL(X)	RTT::Logger::log() << RTT::Logger::Fatal << X << RTT::Logger::endl
#define PRETTYPRINT(X)	std::cout << PrettyPrint::goUp << X << PrettyPrint::coloroff << PrettyPrint::goBack << std::flush;
#else
#define COUT(X)		{}
#define DEBUG(X)	{}
#define INFO(X)		{}
#define WARNING(X)	{}
#define ERROR(X)	{}
#define CRITICAL(X)	{}
#define FATAL(X)	{}
#define PRETTYPRINT(X)	{}
#endif

namespace aa
{
namespace modules
{
namespace display
{
namespace statemachine
{

typedef std::map<QKeySequence, QSharedPointer<gui::KeyBindings> > KeyBindingsMap;
typedef patterns::Singleton<KeyBindingsMap> theBindings;


using namespace aa::modules::nav::statemachine;

REGISTERTASKCONTEXT(DisplayStateMachine);

#define LOG_CONTROL_CHANGES		1
#define LOG_HISTORY_SIZE		10

using namespace std;
using namespace boost;
using namespace math;
using namespace util;
using namespace RTT;
using RTT::Logger;

DisplayStateMachine::DisplayStateMachine(std::string const & name)
	: Painter3DTask(name)

	, mMaxStateHistorySize("MaxStateHistorySize", "number of states to be displayed in history", 5)
	, mReplanNowKeyBinding("")
{
	addProperty(mMaxStateHistorySize);
}

DisplayStateMachine::~DisplayStateMachine()
{
	stop();
}

bool DisplayStateMachine::startHook()
{
	Logger::In in("DisplayStateMachine");

	mStateMachine = findPeer(this, "StateMachine");

	if (!mStateMachine) {
		ERROR("missing StateMachine peer");
		return false;
	}

	osg::ApplicationUsage * usage = new osg::ApplicationUsage();

	BOOST_FOREACH(KeyBindingsMap::value_type const & b, theBindings::instance()) {
		if (!b.second->empty()) {
			usage->addKeyboardMouseBinding(b.first.toString().toStdString(), b.second->toolTip().toStdString());
		}
	}

	for (osg::ApplicationUsage::UsageMap::const_iterator iter = usage->getKeyboardMouseBindings().begin(); iter != usage->getKeyboardMouseBindings().end(); iter++) {
		if (iter->second.compare("Behaviour.ReplanNow") == 0) {
			mReplanNowKeyBinding = iter->first.c_str();
			break;
		}
	}

	return true;
}

void DisplayStateMachine::updateHook()
{
	QMutexLocker lock(&mMutex);
	Logger::In in("DisplayStateMachine");
}

void DisplayStateMachine::stopHook()
{
}

void DisplayStateMachine::init3D(SceneNodePtr sceneNode)
{
	Painter3DTask::init3D(sceneNode);
	sceneNode->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	sceneNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF);
	setSupportsDisplayList(false);
}

void DisplayStateMachine::draw3D(DrawArg viewer)
{
	QMutexLocker lock(&mMutex);


	if (mStateMachine) {
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

		flt const top = 0.69;
		flt const bottom = top - 0.04 - max(mMaxStateHistorySize.get(), 3) * 0.04;
		renderStateMachineInfo(-0.99, 0.682, top, bottom);

		OperationCaller<bool(void)> getReplanWarning(mStateMachine->getOperation("getReplanWarning"));

		if (getReplanWarning()) {
			renderReplanWarning(-0.99, 0.99, bottom - 0.01, bottom - 0.07);
		}

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		glPopAttrib();
	}
}


void DisplayStateMachine::renderReplanWarning(flt left, flt right, flt top, flt bottom) const
{
	//draw background
	glColor4f(1.0, 1.0, 0.0, 0.2);

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	glColor4f(1.0, 1.0, 0.0, 0.8);

	glBegin(GL_LINE_LOOP);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();

	ostringstream o;
	o.str("");
	o << "ReplanNow suggested action to perform (";
	o << mReplanNowKeyBinding;
	o << ")";

	gui::renderString(o.str().c_str(), left + 0.02, top - 0.04, 0.0, 1.0, 1.0, 0.0, 1.0, GLUT_BITMAP_HELVETICA_10);
}


void DisplayStateMachine::renderStateMachineInfo(flt left, flt right, flt top, flt bottom) const
{
	Vec4 backColor(0.0, 0.0, 0.0, 0.3);
	Vec4 borderColor(1.0, 1.0, 1.0, 0.6);
	Vec4 textColor(0.0, 0.0, 0.0, 1.0);

	//draw background
	glColor4f(backColor[0], backColor[1], backColor[2], backColor[3]);

	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();


	//get statemachine functions
	OperationCaller<int(int)> getControlMode(mStateMachine->getOperation("getControlMode"));
	OperationCaller<bool(int, int)> isSubStateOf(mStateMachine->getOperation("isSubStateOf"));
	OperationCaller<int(void)> getCurrentStateId(mStateMachine->getOperation("getCurrentStateId"));
	OperationCaller<aa::modules::nav::statemachine::LogHistoryType(void)> getLogHistory(mStateMachine->getOperation("getLogHistory"));




	//draw border
	if (getControlMode(CONTROL_MODE_SPEED) == getControlMode(CONTROL_MODE_STEER)) {
		glDisable(GL_LINE_STIPPLE);
	}
	else {
		glLineStipple(3, 0xAAAA);
		glEnable(GL_LINE_STIPPLE);
	}

	if (getControlMode(CONTROL_MODE_SPEED) == CONTROL_MANUAL || getControlMode(CONTROL_MODE_STEER) == CONTROL_MANUAL) {
		borderColor = Vec4(1.0, 1.0, 1.0, 0.6);		//white
		glLineWidth(1.0);
	}
	else if (isSubStateOf(getCurrentStateId(), STATE_LEARNEDDRIVE)) {
		borderColor = Vec4(1.0, 0.0, 1.0, 0.6);		//purple
		glLineWidth(2.5);
	}
	else if (isSubStateOf(getCurrentStateId(), CONTEXT_DRIVE)) {
		borderColor = Vec4(0.0, 1.0, 0.0, 0.6);		//green
		glLineWidth(1.0);
	}
	else if (isSubStateOf(getCurrentStateId(), CONTEXT_BRAKE)) {
		borderColor = Vec4(1.0, 1.0, 0.0, 0.6);		//yellow
		glLineWidth(1.0);
	}
	else if (isSubStateOf(getCurrentStateId(), CONTEXT_STOP)) {
		borderColor = Vec4(1.0, 0.0, 0.0, 0.6);		//red
		glLineWidth(1.0);
	}

	glColor4f(borderColor[0], borderColor[1], borderColor[2], borderColor[3]);

	glBegin(GL_LINE_LOOP);
	glVertex3f(left, top, 0.0);
	glVertex3f(right, top, 0.0);
	glVertex3f(right, bottom, 0.0);
	glVertex3f(left, bottom, 0.0);
	glEnd();
	glDisable(GL_LINE_STIPPLE);

	//30=5.2, 50=6, 100=8
	//draw text
	LogHistoryType const & log = getLogHistory();

	if (!log.empty()) {
		for (int i = log.size() - 1, j = 0; i >= 0; i--, j++) {
			if (j >= mMaxStateHistorySize.get()) {
				break;
			}

			std::ostringstream out;

			StateChangeInfo curStateChange = log[i];

			if (curStateChange.mIsControlModeChange) {

				if (i == log.size() - 1) {
					textColor = Vec4(0.0, 1.0, 1.0, 1.0);			//cyan
				}
				else {
					textColor = Vec4(0.5, 0.5, 0.5, 1.0);			//gray
				}


				out << curStateChange.getTimeString();
				out << "  ";
				out << curStateChange.getControlChangeString();
			}
			else {

				if (i == log.size() - 1) {
					if (isSubStateOf(curStateChange.mNewState.id, STATE_LEARNEDDRIVE)) {
						textColor = Vec4(1.0, 0.0, 1.0, 0.6);		//purple
					}
					else if (isSubStateOf(curStateChange.mNewState.id, CONTEXT_DRIVE)) {
						textColor = Vec4(0.0, 1.0, 0.0, 1.0);		//green
					}
					else if (isSubStateOf(curStateChange.mNewState.id, CONTEXT_BRAKE)) {
						textColor = Vec4(1.0, 1.0, 0.0, 1.0);		//yellow
					}
					else if (isSubStateOf(curStateChange.mNewState.id, CONTEXT_STOP)) {
						textColor = Vec4(1.0, 0.0, 0.0, 1.0);		//red
					}
				}
				else {
					textColor = Vec4(0.5, 0.5, 0.5, 1.0);			//gray
				}

				out << curStateChange.getTimeString();
				out << "  ";
				out << curStateChange.getStatePathString();
				out << curStateChange.getStateString();
			}

			if (i == log.size() - 1) {
				gui::renderString(out.str().c_str(), left + 0.02, bottom + (j + 1) * 0.04 - 0.02, 0.0, textColor[0], textColor[1], textColor[2], textColor[3], GLUT_BITMAP_HELVETICA_10);
				gui::renderString(curStateChange.getReasonString().c_str(), left + 0.8, bottom + (j + 1) * 0.04 - 0.02, 0.0, textColor[0], textColor[1], textColor[2], textColor[3], GLUT_BITMAP_HELVETICA_10);
			}
			else {
				gui::renderString(out.str().c_str(), left + 0.02, bottom + (j + 1) * 0.04, 0.0, textColor[0], textColor[1], textColor[2], textColor[3], GLUT_BITMAP_HELVETICA_10);
				gui::renderString(curStateChange.getReasonString().c_str(), left + 0.8, bottom + (j + 1) * 0.04, 0.0, 0.5, 0.5, 0.5, 1.0, GLUT_BITMAP_HELVETICA_10);
			}
		}
	}
}

}
}
}
}
