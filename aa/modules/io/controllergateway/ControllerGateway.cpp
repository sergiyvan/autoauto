#include "ControllerGateway.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <data/VehicleData.h>
#include <math/AutoMath.h>
//#include <aa/modules/nav/controller/learning/ModelLearning.h>
#include <aa/modules/nav/statemachine/StateMachine.h>

#include <rtt/Logger.hpp>
#include <time.h>





















namespace aa
{
namespace modules
{
namespace io
{
namespace controllergateway
{

REGISTERTASKCONTEXT(ControllerGateway);

using namespace std;
using namespace RTT;
using namespace boost;
using namespace ::math;
using namespace util;
using RTT::Logger;

bool getBool(RTT::InputPort<bool> & port)
{
	bool value = false;
	port.read(value);
	return value;
}

ControllerGateway::ControllerGateway(std::string const & name)
	: util::RtTaskContext(name)
	, mEgoStateIn("EgoStateIn")
	, mCarStateIn("CarStateIn")
	, mPassatCarStateIn("PassatCarStateIn")
	, mControllerDataIn("ControllerDataIn")

	, mControllerActivationRequestIn("ControllerActivationRequestIn")
	, mControllerSpeedIn("ControllerSpeedIn")
	, mControllerSteerIn("ControllerSteerIn")
	, mControllerGearIn("ControllerGearIn")
	, mControllerAuxDevicesIn("ControllerAuxDevicesIn")

	, mJoystickActivationRequestIn("JoystickActivationRequestIn")
	, mJoystickSpeedIn("JoystickSpeedIn")
	, mJoystickSteerIn("JoystickSteerIn")
	, mJoystickGearIn("JoystickGearIn")
	, mJoystickAuxDevicesIn("JoystickAuxDevicesIn")

	, mRemoteControlActivationRequestIn("RemoteControlActivationRequestIn")
	, mRemoteControlSpeedIn("RemoteControlSpeedIn")
	, mRemoteControlSteerIn("RemoteControlSteerIn")
	, mRemoteControlGearIn("RemoteControlGearIn")
	, mRemoteControlAuxDevicesIn("RemoteControlAuxDevicesIn")

	, mEyeControlActivationRequestIn("EyeControlActivationRequestIn")
	, mEyeControlSpeedIn("EyeControlSpeedIn")
	, mEyeControlSteerIn("EyeControlSteerIn")

	, mBrainControlActivationRequestIn("BrainControlActivationRequestIn")
	, mBrainControlSpeedIn("BrainControlSpeedIn")
	, mBrainControlSteerIn("BrainControlSteerIn")

	, mActivationRequestOut("ActivationRequestOut")
	, mSpeedOut("SpeedOut")
	, mSteerOut("SteerOut")
	, mGearOut("GearOut")
	, mAuxDevicesOut("AuxDevicesOut")

	, mControllerDataOut("ControllerDataOut")

	, mCounter("Counter", 0)

	, mNumMeasurements(0)
	, mAutonomousRunDistance(0.0)
	, mAutonomousRunAvgSpeed(0.0)
	, mCurControllerData()

{
	ports()->addPort(mEgoStateIn);
	ports()->addPort(mCarStateIn);
	ports()->addPort(mPassatCarStateIn);
	ports()->addPort(mControllerDataIn);

	ports()->addPort(mControllerActivationRequestIn);
	ports()->addPort(mControllerSpeedIn);
	ports()->addPort(mControllerSteerIn);
	ports()->addPort(mControllerGearIn);
	ports()->addPort(mControllerAuxDevicesIn);

	ports()->addPort(mJoystickActivationRequestIn);
	ports()->addPort(mJoystickSpeedIn);
	ports()->addPort(mJoystickSteerIn);
	ports()->addPort(mJoystickGearIn);
	ports()->addPort(mJoystickAuxDevicesIn);

	ports()->addPort(mRemoteControlActivationRequestIn);
	ports()->addPort(mRemoteControlSpeedIn);
	ports()->addPort(mRemoteControlSteerIn);
	ports()->addPort(mRemoteControlGearIn);
	ports()->addPort(mRemoteControlAuxDevicesIn);

	ports()->addPort(mEyeControlActivationRequestIn);
	ports()->addPort(mEyeControlSpeedIn);
	ports()->addPort(mEyeControlSteerIn);

	ports()->addPort(mBrainControlActivationRequestIn);
	ports()->addPort(mBrainControlSpeedIn);
	ports()->addPort(mBrainControlSteerIn);

	ports()->addPort(mActivationRequestOut);
	ports()->addPort(mSpeedOut);
	ports()->addPort(mSteerOut);
	ports()->addPort(mGearOut);
	ports()->addPort(mAuxDevicesOut);

	ports()->addPort(mControllerDataOut);


	addAttribute(mCounter);

	addOperation("SwitchToManualMode", &ControllerGateway::switchToManualMode, this, RTT::ClientThread).doc("switch speed and steer control to MANUAL");
	addOperation("SwitchToControllerMode", &ControllerGateway::switchToControllerMode, this, RTT::ClientThread).doc("switch speed and steer control to CONTROLLER");
	addOperation("SwitchToJoystickMode", &ControllerGateway::switchToJoystickMode, this, RTT::ClientThread).doc("switch speed and steer control to JOYSTICK");
	addOperation("SwitchToRemoteControlMode", &ControllerGateway::switchToRemoteControlMode, this, RTT::ClientThread).doc("switch speed and steer control to REMOTECONTROL");
	addOperation("SwitchToEyeControlMode", &ControllerGateway::switchToEyeControlMode, this, RTT::ClientThread).doc("switch speed and steer control to EYECONTROL");
	addOperation("SwitchToBrainControlMode", &ControllerGateway::switchToBrainControlMode, this, RTT::ClientThread).doc("switch speed and steer control to BRAINCONTROL");


	addOperation("printOutput", &ControllerGateway::printOutput, this, RTT::ClientThread).doc("print current output variables");
}

ControllerGateway::~ControllerGateway()
{
}

bool ControllerGateway::startHook()
{
	Logger::In in("ControllerGateway");

	mStateMachine = findPeer(this, "StateMachine");

	if (!mStateMachine) {
		logError() << "missing StateMachine peer";
		return false;
	}

	OPTIONAL_PORT(mEgoStateIn);
	OPTIONAL_PORT(mCarStateIn);
	OPTIONAL_PORT(mPassatCarStateIn);
	OPTIONAL_PORT(mControllerDataIn);

	OPTIONAL_PORT(mControllerActivationRequestIn);
	OPTIONAL_PORT(mControllerSpeedIn);
	OPTIONAL_PORT(mControllerSteerIn);
	OPTIONAL_PORT(mControllerGearIn);
	OPTIONAL_PORT(mControllerAuxDevicesIn);

	OPTIONAL_PORT(mJoystickActivationRequestIn);
	OPTIONAL_PORT(mJoystickSpeedIn);
	OPTIONAL_PORT(mJoystickSteerIn);
	OPTIONAL_PORT(mJoystickGearIn);
	OPTIONAL_PORT(mJoystickAuxDevicesIn);

	OPTIONAL_PORT(mRemoteControlActivationRequestIn);
	OPTIONAL_PORT(mRemoteControlSpeedIn);
	OPTIONAL_PORT(mRemoteControlSteerIn);
	OPTIONAL_PORT(mRemoteControlGearIn);
	OPTIONAL_PORT(mRemoteControlAuxDevicesIn);

	OPTIONAL_PORT(mEyeControlActivationRequestIn);
	OPTIONAL_PORT(mEyeControlSpeedIn);
	OPTIONAL_PORT(mEyeControlSteerIn);

	OPTIONAL_PORT(mBrainControlActivationRequestIn);
	OPTIONAL_PORT(mBrainControlSpeedIn);
	OPTIONAL_PORT(mBrainControlSteerIn);

	OPTIONAL_PORT(mActivationRequestOut);
	OPTIONAL_PORT(mSpeedOut);
	OPTIONAL_PORT(mSteerOut);
	OPTIONAL_PORT(mGearOut);
	OPTIONAL_PORT(mAuxDevicesOut);

	OPTIONAL_PORT(mControllerDataOut);

	mLastUpdate.stamp();
	mAutonomousRunStart.stamp();

	return true;
}

int zz = 0;
aa::modules::nav::controller::data::TimedControllerData lastCData;
TimedEgoState lastEgoState;


void ControllerGateway::updateHook()
{
	Logger::In in("ControllerGateway");

	mCounter.set(mCounter.get() + 1);

	TimeStamp now;
	now.stamp();

	// get vehicle properties
	flt const maxSpeed = ::data::theVehicleData::instance().getPropertyType<flt>("maxSpeed")->value();		//maximal speed of car in m/s
	flt const maxSteer = ::data::theVehicleData::instance().getPropertyType<flt>("maxSteer")->value();		//maximal angle of wheel in radian


	flt steerCorrection = 0;
	TimedDouble speedCorrection = TimedDouble(now, 0);
	int gearCorrection = 0;
	AuxDevicesData auxDevicesCorrection;
	bool activation = false;


	// read ports
	if (mEgoStateIn.connected()) {
		mEgoStateIn.read(mCurEgoState);
	}

	if (mCarStateIn.connected()) {
		mCarStateIn.read(mCurCarState);
	}

	if (mPassatCarStateIn.connected()) {
		mPassatCarStateIn.read(mCurPassatCarState);
	}


	//get statemachine functions
	OperationCaller<std::string(void)> getCurrentStateName(mStateMachine->getOperation("getCurrentStateName"));
	OperationCaller<bool(int, std::string, std::string)> enableManualOverride(mStateMachine->getOperation("enableManualOverride"));
	OperationCaller<bool(int, std::string, std::string)> disableManualOverride(mStateMachine->getOperation("disableManualOverride"));
	OperationCaller<int(int)> getControlMode(mStateMachine->getOperation("getControlMode"));



	//use existing controller data or create new one
	if (mControllerDataIn.connected()) {
		mControllerDataIn.read(mCurControllerData);
	}
	else {
		if (mEgoStateIn.connected() && mCarStateIn.connected()) {
			std::string curStateName = getCurrentStateName();
			mCurControllerData = aa::modules::nav::controller::data::TimedControllerData(now, aa::modules::nav::controller::data::ControllerData(mCurEgoState, mCurCarState, curStateName));
		}
		else {
			mCurControllerData = aa::modules::nav::controller::data::TimedControllerData(now, aa::modules::nav::controller::data::ControllerData());
		}
	}

	//update measured values from car state
	if (mCarStateIn.connected()) {
		if (std::isnan(mCurControllerData.throttlePositionMeasured)) {
			mCurControllerData.throttlePositionMeasured = mCurCarState.gasPosition;
		}

		if (std::isnan(mCurControllerData.steeringAngleMeasured)) {
			mCurControllerData.steeringAngleMeasured = mCurCarState.wheelPosition * maxSteer;
		}

// 		std::cout<<mCurControllerData.steeringAngleMeasured;
	}


	if (mCarStateIn.connected() && !mCurCarState.autonomousControl) {

		//if(mCounter.get() % 100 == 0) {
		//  std::cout<<"no auto";
		//}

		//not in autonomous mode
		activation = getBool(mControllerActivationRequestIn)
					 || getBool(mRemoteControlActivationRequestIn)
					 || getBool(mEyeControlActivationRequestIn)
					 || getBool(mBrainControlActivationRequestIn)
					 || getBool(mJoystickActivationRequestIn);

		speedCorrection.data = mCurCarState.gasPositionInput;
		steerCorrection = mCurCarState.wheelPositionInput;

		if (speedCorrection.data < -1.0 || speedCorrection.data > 1.0) {
			speedCorrection.data = 0;
		}

		if (steerCorrection < -1.0 || steerCorrection > 1.0) {
			steerCorrection = 0;
		}

		//gather statistics of last autonomos run
		flt autonomousDuration = 1E-9f * RTT::os::TimeService::ticks2nsecs(now - mAutonomousRunStart); //in seconds

		if (enableManualOverride(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, rtti::typeName(typeid(*this)), (format("manual take over (%s, %0.1f m, %0.2f km/h)") % toTimeString(autonomousDuration) % mAutonomousRunDistance % (mAutonomousRunAvgSpeed * MS_2_KMH)).str())) {

			if (mNumMeasurements > 10 && autonomousDuration > 60 && mAutonomousRunDistance > 100 && mAutonomousRunAvgSpeed > 0) {
				std::ofstream file;
				file.open("autonomous_runs.log", std::ios::out | std::ios::app);

				//print statistics
				time_t tnow = time(NULL);
				time_t tstart = tnow - autonomousDuration;

				struct tm tm_now, tm_start;
				localtime_r(&tnow, &tm_now);
				localtime_r(&tstart, &tm_start);

				char bnow[20];
				strftime(bnow, 20, "%d.%m.%Y %H:%M:%S", &tm_now);
				char bstart[20];
				strftime(bstart, 20, "%d.%m.%Y %H:%M:%S", &tm_start);


				file << "start: " << bstart
					 << "   end: " << bnow
					 << "   duration: " << toTimeString(autonomousDuration)
					 << "   distance: " << mAutonomousRunDistance
					 << "   avgspeed: " << mAutonomousRunAvgSpeed * MS_2_KMH << std::endl;



				file.close();
			}
		}


	}
	else {

		//if(mCounter.get() % 100 == 0) {
		//    std::cout<<"auto";
		//}

		if (disableManualOverride(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, rtti::typeName(typeid(*this)), "manual control disabled")) {
			mNumMeasurements = 0;
			mAutonomousRunDistance = 0;
			mAutonomousRunAvgSpeed = 0;
			mAutonomousRunStart.stamp();
		}

		//we choose input from desired port
		//speed
		if (getControlMode(CONTROL_MODE_SPEED) == CONTROL_CONTROLLER) {
			mControllerSpeedIn.read(speedCorrection);
			activation = getBool(mControllerActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_SPEED) == CONTROL_JOYSTICK) {
			mJoystickSpeedIn.read(speedCorrection);
			activation = getBool(mJoystickActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_SPEED) == CONTROL_REMOTECONTROL) {
			mRemoteControlSpeedIn.read(speedCorrection);
			activation = getBool(mRemoteControlActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_SPEED) == CONTROL_EYECONTROL) {
			mEyeControlSpeedIn.read(speedCorrection);
			activation = getBool(mEyeControlActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_SPEED) == CONTROL_BRAINCONTROL) {
			mBrainControlSpeedIn.read(speedCorrection);
			activation = getBool(mBrainControlActivationRequestIn);
		}

		//steer
		if (getControlMode(CONTROL_MODE_STEER) == CONTROL_CONTROLLER) {
			mControllerSteerIn.read(steerCorrection);
			activation &= getBool(mControllerActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_JOYSTICK) {
			mJoystickSteerIn.read(steerCorrection);
			activation &= getBool(mJoystickActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_REMOTECONTROL) {
			mRemoteControlSteerIn.read(steerCorrection);
			activation &= getBool(mRemoteControlActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_EYECONTROL) {
			mEyeControlSteerIn.read(steerCorrection);
			activation &= getBool(mEyeControlActivationRequestIn);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_BRAINCONTROL) {
			mBrainControlSteerIn.read(steerCorrection);
			activation &= getBool(mBrainControlActivationRequestIn);
		}

		//gear
		if (getControlMode(CONTROL_MODE_STEER) == CONTROL_CONTROLLER) {
			mControllerGearIn.read(gearCorrection);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_JOYSTICK) {
			mJoystickGearIn.read(gearCorrection);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_REMOTECONTROL) {
			mRemoteControlGearIn.read(gearCorrection);
		}

		//aux
		if (getControlMode(CONTROL_MODE_STEER) == CONTROL_CONTROLLER) {
			mControllerAuxDevicesIn.read(auxDevicesCorrection);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_JOYSTICK) {
			mJoystickAuxDevicesIn.read(auxDevicesCorrection);
		}
		else if (getControlMode(CONTROL_MODE_STEER) == CONTROL_REMOTECONTROL) {
			mRemoteControlAuxDevicesIn.read(auxDevicesCorrection);
		}


		//update statistics
		flt curSpeed = mCurEgoState.vehicleSpeed();

		if (mPassatCarStateIn.connected()) {
			curSpeed = mCurPassatCarState.wheelSpeeds.speedAvg();
		}

		flt timeDiff = 1E-9f * RTT::os::TimeService::ticks2nsecs(now - mLastUpdate); //in seconds

		mAutonomousRunAvgSpeed = ((mAutonomousRunAvgSpeed * mNumMeasurements) + curSpeed) / (mNumMeasurements + 1);	// in m/s
		mAutonomousRunDistance = mAutonomousRunDistance + curSpeed * timeDiff;		//in m
		mNumMeasurements++;


		//COUT("num: " << mNumMeasurements
		//     << "   curSpeed: " << curSpeed
		//     << "   distance: " << mAutonomousRunDistance
		//     << "   avgspeed: " << mAutonomousRunAvgSpeed * MS_2_KMH);
	}

// 	speedCorrection.stamp();
	mActivationRequestOut.write(activation);
	mSpeedOut.write(speedCorrection);
	mSteerOut.write(steerCorrection);
	mGearOut.write(gearCorrection);
	mAuxDevicesOut.write(auxDevicesCorrection);

	mCurControllerData.speedCorrection = speedCorrection.data;
	mCurControllerData.steerCorrection = steerCorrection;
	mCurControllerData.gearCorrection = gearCorrection;

	mCurControllerData.stamp();

//	std::cout<<"send activation: " << activation << " and steer: " << steerCorrection;

//    MinRiskTesting
//    std::cout<< "ContrGatew speed: " <<speedCorrection.data<< "controlMode" << getControlMode(CONTROL_MODE_SPEED) << std::endl;



	//TEST
// 	zz++;
// 	if((curControllerData.carPos - lastCData.carPos).norm() == 0) {
	//check timestamp
// 		flt timediff = 1E-9f * RTT::os::TimeService::ticks2nsecs( abs(curEgoState - lastEgoState) );
// 		std::cout<<endl<<"WARNING (frame " << zz << "): pos diff = 0, with speeds = " << lastCData.curSpeed << " and " << curControllerData.curSpeed<<endl;
// 		std::cout<<"TIMEDIFF: " << timediff <<endl;
// 	}

	mControllerDataOut.write(mCurControllerData);

	//get model prediction data
    /**
     * removed for ARND
     *
	if (theModelLearner::instance().isReady()) {
		if (mRecentControllerData.size() >= theModelLearner::instance().getRequiredHistorySize()) {
			//create ModelPredictorDataSet
			mCurModelPredictionDataSet.clear();

			vector<int> timeSteps;

			for (int t = 0; t <= theModelLearner::instance().getMaxPredictionSteps(); t += theModelLearner::instance().getPredictionStepSize()) {
				timeSteps.push_back(t);
			}

			for (flt ai = 0; ai < theModelLearner::instance().getActionSet().size(); ai++) {
				flt a = theModelLearner::instance().getActionSet()[ai];

				vector<Vec2>  predictedPos, predictedDir, historyPos, historyDir;
				tie(predictedPos, predictedDir, historyPos, historyDir) = theModelLearner::instance().getMultiPrediction(mRecentControllerData, mCurControllerData, a, timeSteps);

				aa::modules::nav::controller::data::TimedModelPredictionData tmpd(now, aa::modules::nav::controller::data::ModelPredictionData(a, false));
				tmpd.predictedCarPos = predictedPos;
				tmpd.predictedCarDir = predictedDir;
				tmpd.historyCarPos = historyPos;
				tmpd.historyCarDir = historyDir;


//				std::cout<<"Action: " << a;
//				std::cout<<"timestep " << timeSteps.back() << "  #  " << mRecentControllerData.size() << "  " << mRecentControllerData.front().carPos[0] << " " << mRecentControllerData.front().carPos[1];


				mCurModelPredictionDataSet.push_back(tmpd);
			}

			vector<Vec2>  predictedPos, predictedDir, historyPos, historyDir;
			tie(predictedPos, predictedDir, historyPos, historyDir) = theModelLearner::instance().getMultiPrediction(mRecentControllerData, mCurControllerData, mCurControllerData.steerCorrection, timeSteps);

			aa::modules::nav::controller::data::TimedModelPredictionData tmpd(now, aa::modules::nav::controller::data::ModelPredictionData(mCurControllerData.steerCorrection, true));
			tmpd.predictedCarPos = predictedPos;
			tmpd.predictedCarDir = predictedDir;
			tmpd.historyCarPos = historyPos;
			tmpd.historyCarDir = historyDir;

			mCurModelPredictionDataSet.push_back(tmpd);

			/// write out model prediction data
			mModelPredictionDataSetOut.write(mCurModelPredictionDataSet);
		}
	}
*/
	mRecentControllerData.push_front(mCurControllerData);
/**
 * removed for ARND
	while (mRecentControllerData.size() > theModelLearner::instance().getRequiredHistorySize()) {
		mRecentControllerData.pop_back();
	}
*/

// 	flt const frontShaftDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontShaftDistance")->value();

// 	Vec2 const
// 		imuPos = ::math::head( curEgoState.position() ),
// 		carDir = normalized(::math::head( curEgoState.forwardDirection() )),
// 		carPos = ::math::head( curEgoState.position() ) + frontShaftDistance * carDir;

// 	cout << "ControllerGateway: " << zz << " " << carPos << endl;
// 	std::cout<<zz << " " << curControllerData.carPos[0] << " " << curControllerData.carPos[1] << " " << curControllerData.carDir[0] << " " << curControllerData.carDir[1] << " " << curControllerData.steeringAngleMeasured << " " << curControllerData.curSpeed << " " << curControllerData.steerCorrection*maxSteer;

	//log for Raul
// 	flt timediff = 1E-6f * RTT::os::TimeService::ticks2nsecs( abs(curEgoState - lastEgoState) );
// 	std::cout<<timediff << " " << curControllerData.carPos[0] << " " << curControllerData.carPos[1] << " " << curControllerData.carDir[0] << " " << curControllerData.carDir[1] << " " << curControllerData.steeringAngleMeasured << " " << curControllerData.curSpeed << " " << curControllerData.steerCorrection*maxSteer;
/**
 * remove for ARND
	if (theModelLearner::instance().isRecording()) {
		theModelLearner::instance().addSample(mCurControllerData);
	}
*/
	lastCData = mCurControllerData;
	lastEgoState = mCurEgoState;

	mLastUpdate = now;
}


void ControllerGateway::stopHook()
{
}

void ControllerGateway::errorHook()
{
}










/** Speed and Steer Control Mode commands */
bool ControllerGateway::switchToManualMode()
{
	Logger::In in("ControllerGateway");

	OperationCaller<bool(int, std::string, std::string)> enableManualOverride(mStateMachine->getOperation("enableManualOverride"));

	enableManualOverride(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, rtti::typeName(typeid(*this)), "switched to MANUAL mode by manual method call");
	logInfo() << "Control Mode has switched to MANUAL";
	return true;
}

bool ControllerGateway::switchToControllerMode()
{
	Logger::In in("ControllerGateway");

	OperationCaller<void(int, int, std::string, std::string)> setControlMode(mStateMachine->getOperation("setControlMode"));

	setControlMode(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, CONTROL_CONTROLLER, rtti::typeName(typeid(*this)), "switched to CONTROLLER mode by manual method call");
	logInfo() << "Control Mode has switched to CONTROLLER";
	return true;
}

bool ControllerGateway::switchToJoystickMode()
{
	Logger::In in("ControllerGateway");

	OperationCaller<void(int, int, std::string, std::string)> setControlMode(mStateMachine->getOperation("setControlMode"));

	setControlMode(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, CONTROL_JOYSTICK, rtti::typeName(typeid(*this)), "switched to JOYSTICK mode by manual method call");
	logInfo() << "Control Mode has switched to JOYSTICK";
	return true;
}

bool ControllerGateway::switchToRemoteControlMode()
{
	Logger::In in("ControllerGateway");

	OperationCaller<void(int, int, std::string, std::string)> setControlMode(mStateMachine->getOperation("setControlMode"));

	setControlMode(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, CONTROL_REMOTECONTROL, rtti::typeName(typeid(*this)), "switched to REMOTECONTROL mode by manual method call");
	logInfo() << "Control Mode has switched to REMOTECONTROL";
	return true;
}

bool ControllerGateway::switchToEyeControlMode()
{
	Logger::In in("ControllerGateway");

	OperationCaller<void(int, int, std::string, std::string)> setControlMode(mStateMachine->getOperation("setControlMode"));

	setControlMode(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, CONTROL_EYECONTROL, rtti::typeName(typeid(*this)), "switched to EYECONTROL mode by manual method call");
	logInfo() << "Control Mode has switched to EYECONTROL";
	return true;
}

bool ControllerGateway::switchToBrainControlMode()
{
	Logger::In in("ControllerGateway");

	OperationCaller<void(int, int, std::string, std::string)> setControlMode(mStateMachine->getOperation("setControlMode"));

	setControlMode(CONTROL_MODE_SPEED | CONTROL_MODE_STEER, CONTROL_BRAINCONTROL, rtti::typeName(typeid(*this)), "switched to BRAINCONTROL mode by manual method call");
	logInfo() << "Control Mode has switched to BRAINCONTROL";
	return true;
}

std::string ControllerGateway::toTimeString(flt seconds) const
{
	tm time;
	time.tm_sec = (int)seconds % 60;
	time.tm_min = seconds / 60;
	time.tm_hour = seconds / 60 / 60;

	char output[12];
	strftime(output, 12, "%H:%M:%S", &time);

	return string(output);
}


bool ControllerGateway::printOutput() const
{

	//get statemachine functions
	OperationCaller<void(void)> printState(mStateMachine->getOperation("printState"));
	OperationCaller<void(void)> printControlMode(mStateMachine->getOperation("printControlMode"));


	std::cout << std::endl << "StateMachine: ";
	printState();
	printControlMode();
	std::cout << "";

	return true;
}

}
}
}
}
