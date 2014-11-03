#include "DisplayCarState.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <boost/format.hpp>
#include <algorithm>

#include <rtt/Logger.hpp>
#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <gui/GlTools.h>

#include <osg/Group>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osg/Geode>

#define VERBOSE
#if defined(VERBOSE)
#define COUT(X)		std::cout << X << std::endl
#define DEBUG(X)	RTT::Logger::log() << RTT::Logger::Debug << X << RTT::Logger::endl
#define INFO(X)		RTT::Logger::log() << RTT::Logger::Info << X << RTT::Logger::endl
#define WARNING(X)	RTT::Logger::log() << RTT::Logger::Warning << X << RTT::Logger::endl
#define CRITICAL(X)	RTT::Logger::log() << RTT::Logger::Critical << X << RTT::Logger::endl
#define ERROR(X)	RTT::Logger::log() << RTT::Logger::Error << X << RTT::Logger::endl
#endif

namespace aa
{
namespace modules
{
namespace display
{
namespace carstate
{


REGISTERTASKCONTEXT(DisplayCarState);

DisplayCarState::DisplayCarState(std::string const & name)
	: Painter3DTask(name)
	// Members

	// Ports
	, mCarStateIn("CarStateIn")
	, mPassatCarStateIn("PassatCarStateIn")
{
	ports()->addPort(mCarStateIn);
	ports()->addPort(mPassatCarStateIn);
}

DisplayCarState::~DisplayCarState()
{
	stop();
}

bool DisplayCarState::startHook()
{
	RTT::Logger::In in("DisplayCarState");

	REQUIRED_PORT(mCarStateIn);
	OPTIONAL_PORT(mPassatCarStateIn);

	return true;
}

void DisplayCarState::updateHook()
{
	QMutexLocker lock(&m_mutex);
	RTT::Logger::In in("DisplayCarState");

	mCarStateIn.read(mCurCarState);

	mPassatCarStateIn.read(mCurPassatCarState);
}

void DisplayCarState::stopHook()
{
}

void DisplayCarState::init3D(SceneNodePtr sceneNode)
{
	gui::Painter3DTask::init3D(sceneNode);
	sceneNode->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
	sceneNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF);
	setSupportsDisplayList(false);
}

void DisplayCarState::draw3D(DrawArg)
{
	QMutexLocker lock(&m_mutex);

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

// 	if(mCarStateIn.connected()) {
	renderCarState(-0.98, -0.71, 0.68, -0.64);
// 	}

// 	if(mPassatCarStateIn.connected()) {
	renderPassatCarState(-0.20, 0.98, 0.68, -0.64);
// 	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glPopAttrib();
}

void DisplayCarState::renderCarState(math::flt left, math::flt right, math::flt top, math::flt bottom)
{
	math::flt offset = 0.02f;
	math::flt width = right - left;
	math::flt mid = (left + right) / 2;

	glColor4f(1.f, 1.f, 1.f, 0.3f);
	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.f);
	glVertex3f(right, top, 0.f);
	glVertex3f(right, bottom, 0.f);
	glVertex3f(left, bottom, 0.f);
	glEnd();

	gui::renderString("CarState:", left + offset, top - 0.05f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

// 	if(mCurCarState == TimeStamp()) return;

	drawMeter(left + offset, right - offset, top - 0.15f, top - 0.22f, (boost::format("Throttle (Soll): %0.3f") % mCurCarState.gasPositionInput).str(), mCurCarState.gasPositionInput);
	drawMeter(left + offset, right - offset, top - 0.33f, top - 0.40f, (boost::format("Throttle (Ist): %0.3f") % mCurCarState.gasPosition).str(), mCurCarState.gasPosition);
	drawMeter(left + offset, right - offset, top - 0.51f, top - 0.58f, (boost::format("Steer (Soll): %0.3f") % mCurCarState.wheelPositionInput).str(), mCurCarState.wheelPositionInput);
	drawMeter(left + offset, right - offset, top - 0.69f, top - 0.76f, (boost::format("Steer (Ist): %0.3f") % mCurCarState.wheelPosition).str(), mCurCarState.wheelPosition);

	drawToggle(right - 0.04f, top - 0.85f, -0.21f, "Autonom. Control", mCurCarState.autonomousControl);
	drawToggle(right - 0.04f, top - 0.91f, -0.21f, "Headlights", mCurCarState.headlights);
	drawToggle(right - 0.04f, top - 0.97f, -0.21f, "Siren", mCurCarState.siren);
	drawToggle(right - 0.04f, top - 1.03f, -0.21f, "Turn Signal", mCurCarState.turnSignal == 2 || mCurCarState.turnSignal == 3);		//right
	drawToggle(right - 0.07f, top - 1.03f, -0.21f, "", mCurCarState.turnSignal == 1 || mCurCarState.turnSignal == 3);					//left

	gui::renderString("Gear Position:", left + offset, top - 1.1f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString(::modules::models::carstate::CarState::gearString(mCurCarState.gearPosition), right - 0.05f, top - 1.1f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
}


void DisplayCarState::renderPassatCarState(math::flt left, math::flt right, math::flt top, math::flt bottom)
{
	math::flt offset = 0.02f;
	math::flt width = right - left;
	math::flt mid = (left + right) / 2;

	glColor4f(173.0 / 255.0, 216.0 / 255.0, 230.0 / 255.0, 0.3f);
	glBegin(GL_QUADS);
	glVertex3f(left, top, 0.f);
	glVertex3f(right, top, 0.f);
	glVertex3f(right, bottom, 0.f);
	glVertex3f(left, bottom, 0.f);
	glEnd();

	math::flt col1 = 0.18f;
	math::flt col2 = 0.48f;
	math::flt col3 = 0.84f;
	math::flt col4 = 1.14f;

	gui::renderString("PASSAT CarState:", left + offset, top - 0.05f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);


	drawMeter(left + offset, left + 0.22f, top - 0.15f, top - 0.22f, (boost::format("Throttle (Soll): %0.3f") % mCurPassatCarState.gasPositionInput).str(), mCurPassatCarState.gasPositionInput);
	drawMeter(left + offset, left + 0.22f, top - 0.33f, top - 0.40f, (boost::format("Throttle (Ist): %0.3f") % mCurPassatCarState.gasPosition).str(), mCurPassatCarState.gasPosition);
	drawMeter(left + offset, left + 0.22f, top - 0.51f, top - 0.58f, (boost::format("Steer (Soll): %0.3f") % mCurPassatCarState.wheelPositionInput).str(), mCurPassatCarState.wheelPositionInput);
	drawMeter(left + offset, left + 0.22f, top - 0.69f, top - 0.76f, (boost::format("Steer (Ist): %0.3f") % mCurPassatCarState.wheelPosition).str(), mCurPassatCarState.wheelPosition);





	//WatchdogStatus watchdogStatus
	drawToggle(left + col2, top - 0.120f, -0.24f, "GasBrakeEnabled", mCurPassatCarState.watchdogStatus.gasBrakeEnabled());
	drawToggle(left + col2, top - 0.165f, -0.24f, "GearEnabled", mCurPassatCarState.watchdogStatus.gearEnabled());

	gui::renderString("WatchdogState", left + col2 - 0.24f, top - 0.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString(mCurPassatCarState.watchdogStatus.watchdogStateString(mCurPassatCarState.watchdogStatus.watchdogState()), left + col2 - 0.02f, top - 0.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	drawToggle(left + col2, top - 0.255f, -0.24f, "Voltage", mCurPassatCarState.watchdogStatus.voltage());
	drawToggle(left + col2, top - 0.300f, -0.24f, "WatchdogCommand", mCurPassatCarState.watchdogStatus.watchdogCommand());
	drawToggle(left + col2, top - 0.345f, -0.24f, "ControllerGear", mCurPassatCarState.watchdogStatus.controllerGear());
	drawToggle(left + col2, top - 0.390f, -0.24f, "ControllerBrake", mCurPassatCarState.watchdogStatus.controllerBrake());
	drawToggle(left + col2, top - 0.435f, -0.24f, "ControllerThrottle", mCurPassatCarState.watchdogStatus.controllerThrottle());
	drawToggle(left + col2, top - 0.480f, -0.24f, "ControllerFunction", mCurPassatCarState.watchdogStatus.controllerFunction());
	drawToggle(left + col2, top - 0.525f, -0.24f, "StatusBrake", mCurPassatCarState.watchdogStatus.statusBrake());
	drawToggle(left + col2, top - 0.570f, -0.24f, "StatusThrottle", mCurPassatCarState.watchdogStatus.statusThrottle());
	drawToggle(left + col2, top - 0.615f, -0.24f, "StatusFunction", mCurPassatCarState.watchdogStatus.statusFunction());
	drawToggle(left + col2, top - 0.660f, -0.24f, "StatusGear", mCurPassatCarState.watchdogStatus.statusGear());

	gui::renderString("PowerDownReason", left + col2 - 0.24f, top - 0.715f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString(mCurPassatCarState.watchdogStatus.powerDownReasonString(mCurPassatCarState.watchdogStatus.powerOffReason()), left + col2 - 0.07f, top - 0.715f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	drawToggle(left + col2, top - 0.750f, -0.24f, "StatusActoricBox", mCurPassatCarState.watchdogStatus.statusActoricBox());
	drawToggle(left + col2, top - 0.795f, -0.24f, "DriverBrakes", mCurPassatCarState.watchdogStatus.driverBrakes());
	drawToggle(left + col2, top - 0.840f, -0.24f, "DriverAccelerates", mCurPassatCarState.watchdogStatus.driverAccelerates());
	drawToggle(left + col2, top - 0.885f, -0.24f, "DriverShiftsGear", mCurPassatCarState.watchdogStatus.driverShiftsGear());
	drawToggle(left + col2, top - 0.930f, -0.24f, "PoweredDown", mCurPassatCarState.watchdogStatus.poweredDown());
	drawToggle(left + col2, top - 0.975f, -0.24f, "EnableReqGasBrake", mCurPassatCarState.watchdogStatus.enableRequestGasBrake());
	drawToggle(left + col2, top - 1.020f, -0.24f, "EnableReqGear", mCurPassatCarState.watchdogStatus.enableRequestGear());

	//WatchdogBatteryStatus watchdogBatteryStatus
	gui::renderString("Bat Voltage:", left + col2 - 0.24f, top - 1.075f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%1% V") % mCurPassatCarState.watchdogBatteryStatus.voltage()).str().c_str(), left + col2 - 0.02f, top - 1.075f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	//Light1Status light1Status
	drawToggle(left + col2, top - 1.120f, -0.24f, "ParkingLight", mCurPassatCarState.light1Status.parkingLight());
	drawToggle(left + col2, top - 1.165f, -0.24f, "LowBeam", mCurPassatCarState.light1Status.lowBeam());
	drawToggle(left + col2, top - 1.210f, -0.24f, "WarningLights", mCurPassatCarState.light1Status.warningLights());






	//BrakeStatus1 brakeStatus1
	gui::renderString("ActualPressure1", left + col3 - 0.32f, top - 0.130f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.brakeStatus1.actualPressure()).str().c_str(), left + col3 - 0.06f, top - 0.130f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("TravelSensor1", left + col3 - 0.32f, top - 0.175f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.brakeStatus1.travelSensor()).str().c_str(), left + col3 - 0.045f, top - 0.175f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("MagnetCurrent1", left + col3 - 0.32f, top - 0.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.brakeStatus1.magnetCurrent()).str().c_str(), left + col3 - 0.045f, top - 0.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("MagnetPwmRatio1", left + col3 - 0.32f, top - 0.265f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.brakeStatus1.magnetPwmRatio()).str().c_str(), left + col3 - 0.045f, top - 0.265f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("MagnetVoltage1", left + col3 - 0.32f, top - 0.310f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.brakeStatus1.magnetVoltage()).str().c_str(), left + col3 - 0.045f, top - 0.310f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	drawToggle(left + col3, top - 0.355f, -0.32f, "BrakeActuatorAct", mCurPassatCarState.brakeStatus1.brakeAssistActive());

	//BrakeStatus2 brakeStatus2
	drawToggle(left + col3, top - 0.400f, -0.32f, "BitBAFunc", mCurPassatCarState.brakeStatus2.brakeAssist());
	drawToggle(left + col3, top - 0.445f, -0.32f, "BitPRCFunc", mCurPassatCarState.brakeStatus2.bitPRCFunc());
	drawToggle(left + col3, top - 0.490f, -0.32f, "BCU3StatusError", mCurPassatCarState.brakeStatus2.bcu3StatusError());
	drawToggle(left + col3, top - 0.535f, -0.32f, "BCU3StatusVoltageLowHigh", mCurPassatCarState.brakeStatus2.bcu3StatusVoltageLowHigh());
	drawToggle(left + col3, top - 0.580f, -0.32f, "BCU3StatusBAWarnLamp", mCurPassatCarState.brakeStatus2.bcu3StatusBAWarnLamp());
	drawToggle(left + col3, top - 0.625f, -0.32f, "BCU3StatusPRCWarnLamp", mCurPassatCarState.brakeStatus2.bcu3StatusPRCWarnLamp());
	drawToggle(left + col3, top - 0.670f, -0.32f, "BCU3StatusMode", mCurPassatCarState.brakeStatus2.bcu3StatusMode());
	drawToggle(left + col3, top - 0.715f, -0.32f, "BCU3StatusReleaseSwitch", mCurPassatCarState.brakeStatus2.bcu3StatusReleaseSwitch());
	drawToggle(left + col3, top - 0.760f, -0.32f, "BCU3StatusPressureControl", mCurPassatCarState.brakeStatus2.bcu3StatusPressureControlActive());
	drawToggle(left + col3, top - 0.805f, -0.32f, "DetFailureBCU3intern", mCurPassatCarState.brakeStatus2.detectedFailureBCU3intern());
	drawToggle(left + col3, top - 0.850f, -0.32f, "DetFailurePressureSensor", mCurPassatCarState.brakeStatus2.detectedFailurePressureSensor());
	drawToggle(left + col3, top - 0.895f, -0.32f, "DetFailureTravelSensor", mCurPassatCarState.brakeStatus2.detectedFailureTravelSensor());
	drawToggle(left + col3, top - 0.940f, -0.32f, "DetFailureReleaseSwitch", mCurPassatCarState.brakeStatus2.detectedFailureReleaseSwitch());
	drawToggle(left + col3, top - 0.985f, -0.32f, "DetFailureSolenoid", mCurPassatCarState.brakeStatus2.detectedFailureSolenoid());
	drawToggle(left + col3, top - 1.030f, -0.32f, "DetFailurePressControl", mCurPassatCarState.brakeStatus2.detectedFailurePressControl());
	drawToggle(left + col3, top - 1.075f, -0.32f, "DetFailureCAN", mCurPassatCarState.brakeStatus2.detectedFailureCan());
	drawToggle(left + col3, top - 1.120f, -0.32f, "DetFailurePreChargePump", mCurPassatCarState.brakeStatus2.detectedFailurePreChargePump());

	gui::renderString("ActualPressure2", left + col3 - 0.32f, top - 1.175f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.brakeStatus2.actualPressure()).str().c_str(), left + col3 - 0.045f, top - 1.175f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("PressureDemand2", left + col3 - 0.32f, top - 1.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.brakeStatus2.pressureDemand()).str().c_str(), left + col3 - 0.045f, top - 1.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);





	//ThrottleStatus throttleStatus
	gui::renderString("VoltageGaspedal1", left + col4 - 0.26f, top - 0.130f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f V") % mCurPassatCarState.throttleStatus.voltageGasPedal1()).str().c_str(), left + col4 - 0.07f, top - 0.130f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("VoltageGaspedal2", left + col4 - 0.26f, top - 0.175f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f V") % mCurPassatCarState.throttleStatus.voltageGasPedal2()).str().c_str(), left + col4 - 0.07f, top - 0.175f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("VoltageMotor1", left + col4 - 0.26f, top - 0.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f V") % mCurPassatCarState.throttleStatus.voltageMotor1()).str().c_str(), left + col4 - 0.07f, top - 0.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("VoltageMotor2", left + col4 - 0.26f, top - 0.265f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f V") % mCurPassatCarState.throttleStatus.voltageMotor2()).str().c_str(), left + col4 - 0.07f, top - 0.265f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	//GearStatus gearStatus
	gui::renderString("ChosenPos", left + col4 - 0.26f, top - 0.310f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString(mCurPassatCarState.gearStatus.chosenPosString(mCurPassatCarState.gearStatus.chosenPos()), left + col4 - 0.1f, top - 0.310f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);



	//SteerAssist3Status steerAssist3Status
	gui::renderString("SteerMomentum", left + col4 - 0.26f, top - 0.355f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.steerAssist3Status.steerMomentum()).str().c_str(), left + col4 - 0.045f, top - 0.355f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	//drawMeter(left + 0.84f, left + 1.07f, top - 0.355f, top - 0.425f, (format("SteerMomentum: %1%") % mCurPassatCarState.steerAssist3Status.steerMomentum()).str(), mCurPassatCarState.steerAssist3Status.steerMomentum(), 0, 15.0381);

	drawToggle(left + col4, top - 0.390f, -0.26f, "SteerMomentumValid", mCurPassatCarState.steerAssist3Status.steerMomentumValid());

	gui::renderString("SteerAngle", left + col4 - 0.26f, top - 0.445f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.steerAssist3Status.steerAngle()).str().c_str(), left + col4 - 0.05f, top - 0.445f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	//drawMeter(left + 0.84f, left + 1.07f, top - 0.445f, top - 0.515f, (format("SteerAngle: %1%") % mCurPassatCarState.steerAssist3Status.steerAngle()).str(), mCurPassatCarState.steerAssist3Status.steerAngle(), 0, 614.25);

	drawToggle(left + col4, top - 0.480f, -0.26f, "SteerAngleValid", mCurPassatCarState.steerAssist3Status.steerAngleValid());


	//SignalWipersStatus signalWipersStatus
	drawToggle(left + col4, top - 0.525f, -0.26f, "TurnSignal", mCurPassatCarState.signalWipersStatus.turnRight());		//right
	drawToggle(left + col4 - 0.03f, top - 0.525f, -0.26f, "", mCurPassatCarState.signalWipersStatus.turnLeft());		//left
	drawToggle(left + col4, top - 0.570f, -0.26f, "FlashLight", mCurPassatCarState.signalWipersStatus.flashLight());
	drawToggle(left + col4, top - 0.615f, -0.26f, "HighBeam", mCurPassatCarState.signalWipersStatus.highBeam());
	drawToggle(left + col4, top - 0.660f, -0.26f, "Horn", mCurPassatCarState.signalWipersStatus.horn());
	drawToggle(left + col4, top - 0.705f, -0.26f, "WipeTip", mCurPassatCarState.signalWipersStatus.wipeTip());
	drawToggle(left + col4, top - 0.750f, -0.26f, "WipeInterval", mCurPassatCarState.signalWipersStatus.wipeInterval());
	drawToggle(left + col4, top - 0.795f, -0.26f, "WipeLevel1", mCurPassatCarState.signalWipersStatus.wipeLevel1());
	drawToggle(left + col4, top - 0.840f, -0.26f, "WipeLevel2", mCurPassatCarState.signalWipersStatus.wipeLevel2());
	drawToggle(left + col4, top - 0.885f, -0.26f, "WashFront", mCurPassatCarState.signalWipersStatus.washFront());
	drawToggle(left + col4, top - 0.930f, -0.26f, "WashFrontMov", mCurPassatCarState.signalWipersStatus.washFrontMov());
	drawToggle(left + col4, top - 0.975f, -0.26f, "WipeRearInterval", mCurPassatCarState.signalWipersStatus.wipeRearInterval());
	drawToggle(left + col4, top - 1.020f, -0.26f, "WashRear", mCurPassatCarState.signalWipersStatus.washRear());

	gui::renderString("IntervalLevel", left + col4 - 0.26f, top - 1.075f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%1%") % mCurPassatCarState.signalWipersStatus.intervalLevel()).str().c_str(), left + col4, top - 1.075f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);


	//Motor1Status motor1Status
	drawToggle(left + col4, top - 1.115f, -0.26f, "PedalStatus", mCurPassatCarState.motor1Status.pedalStatus());
	drawToggle(left + col4, top - 1.160f, -0.26f, "KickDown", mCurPassatCarState.motor1Status.kickDown());

	gui::renderString("PedalValue", left + col4 - 0.26f, top - 1.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.motor1Status.pedalValue()).str().c_str(), left + col4 - 0.045f, top - 1.220f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);

	gui::renderString("RevolutionSpeed", left + col4 - 0.26f, top - 1.265f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
	gui::renderString((boost::format("%0.3f") % mCurPassatCarState.motor1Status.revolutionSpeed()).str().c_str(), left + col4 - 0.045f, top - 1.265f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);


}

void DisplayCarState::drawMeter(math::flt left, math::flt right, math::flt top, math::flt bottom, std::string text, math::flt value, math::flt minVal, math::flt maxVal)
{
	math::flt width = right - left;
	math::flt mid = (left + right) / 2;

	math::flt pos = left + (value - minVal) / (maxVal - minVal) * width;
	math::flt zero = std::max(left, left + (0 - minVal) / (maxVal - minVal) * width);

	if (!std::isnan(value)) {
		glColor4f(0.f, 1.f, 0.f, 0.6f);
		glBegin(GL_QUADS);
		glVertex3f(zero, top - 0.01f, 0.f);
		glVertex3f(pos, top - 0.01f, 0.f);
		glVertex3f(pos, bottom + 0.01f, 0.f);
		glVertex3f(zero, bottom + 0.01f, 0.f);
		glEnd();
	}

	glColor4f(1.f, 1.f, 1.f, 0.8f);
	glBegin(GL_LINE_LOOP);
	glVertex3f(left, top, 0.f);
	glVertex3f(right, top, 0.f);
	glVertex3f(right, bottom, 0.f);
	glVertex3f(left, bottom, 0.f);
	glEnd();

	if (minVal <= 0 && maxVal >= 0) {
		glBegin(GL_LINES);
		glVertex3f(zero, top + 0.005f, 0.f);
		glVertex3f(zero, bottom - 0.015f, 0.f);
		glEnd();
	}

	gui::renderString(text.c_str(), left, top + 0.02f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
}

void DisplayCarState::drawToggle(math::flt x, math::flt y, math::flt textOffset, std::string text, bool on)
{
	gui::drawStatusCircle(x, y, 0, on);

	gui::renderString(text.c_str(), x + textOffset, y - 0.01f, 0.f, 1.f, 1.f, 1.f, 0.6f, GLUT_BITMAP_HELVETICA_10);
}
}
}
}
}

