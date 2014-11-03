#include "Joystick.h"

#include <rtt/Logger.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>
#include <SDL/SDL.h>
#include <cmath>

#include <util/TaskContextFactory.h>
#include <util/PrettyPrint.h>
#include <math/AutoMath.h>
#include <aa/modules/nav/statemachine/StateMachine.h>
#include <rtt/types/CArrayTypeInfo.hpp>












REGISTERTASKCONTEXT(Joystick);


using namespace std;
using namespace math;
using namespace modules::models::carstate;
using RTT::Logger;

Joystick::Joystick(string const & name)
	: util::RtTaskContext(name)
	, mEgoStateIn("EgoStateIn")
	, mCarStateIn("CarStateIn")

	, mJoystickDataOut("JoystickDataOut")
	, mActivationRequestOut("ActivationRequestOut")
	, mSpeedOut("SpeedOut")
	, mSteerOut("SteerOut")
	, mGearOut("GearOut")
	, mAuxDevicesOut("AuxDevicesOut")

	, mJoysticks("AvailableJoysticks")
	, mSelectedJoystick("SelectedJoystick", "Name of the selected joystick")
	, mVerbose("Verbose", "Output joystick data.", false)
	, mSelectedJoystickNumber(-1)
	, mpJoystick(NULL)
{
	// RTT::types::TypeInfoRepository::Instance()->addType(new RTT::types::CArrayTypeInfo<const char *, false>("strcarray")); FIXME

	addAttribute(mJoysticks);

	addProperty(mSelectedJoystick);
	addProperty(mVerbose);

	ports()->addPort(mEgoStateIn);
	ports()->addPort(mCarStateIn);

	ports()->addPort(mJoystickDataOut);
	ports()->addPort(mActivationRequestOut);
	ports()->addPort(mSpeedOut);
	ports()->addPort(mSteerOut);
	ports()->addPort(mGearOut);
	ports()->addPort(mAuxDevicesOut);

	addOperation("listJoysticks", &Joystick::listJoysticks, this, RTT::ClientThread).doc("returns a list of all connected control devices");
	addOperation("selectJoystickByName", &Joystick::selectJoystickByName, this, RTT::ClientThread).doc("sets the new joystick by name").arg("name", "name");
	addOperation("selectJoystickByIndex", &Joystick::selectJoystickByIndex, this, RTT::ClientThread).doc("sets the new joystick to device i").arg("i", "i");

	addOperation("printOutput", &Joystick::printOutput, this, RTT::ClientThread).doc("print current output variables");
}

Joystick::~Joystick()
{}

bool Joystick::startHook()
{
	Logger::In in("Joystick");


	if (0 != SDL_Init(SDL_INIT_JOYSTICK)) {
		RTT::Logger::log(RTT::Logger::Error) << "Failed to initialise joystick subsystem" << RTT::Logger::endl;
		return false;
	}

	atexit(&SDL_Quit);

	int num = SDL_NumJoysticks();
	RTT::Logger::log(RTT::Logger::Info) << "Found " << num << " joystick devices. Use Joystick.listJoysticks to list them." << RTT::Logger::endl;

	if (num == 0) {
		return true;
	}
	else if (num < 0) {
		return false;
	}

	if (mSelectedJoystick.rvalue() == "") {
		mSelectedJoystick.set(SDL_JoystickName(0));
	}

	std::vector<const char *> joy(num);

	for (unsigned i = 0; i < (unsigned int)num; ++i) {
		joy[i] = SDL_JoystickName(i);

		if (joy[i] == mSelectedJoystick.rvalue()) {
			setup(i);
		}
	}

	mJoysticks.set(joy);

	return NULL != mpJoystick;
}

bool Joystick::setup(int i)
{
	if (NULL != mpJoystick) {
		SDL_JoystickClose(mpJoystick);
	}

	mpJoystick = SDL_JoystickOpen(i);

	if (NULL == mpJoystick) {
		return false;
	}

	SDL_JoystickUpdate();

	mSelectedJoystickNumber = i;
	mSelectedJoystick.set(SDL_JoystickName(i));
	mData.axis.resize(SDL_JoystickNumAxes(mpJoystick));
	mData.button.resize(SDL_JoystickNumButtons(mpJoystick));
	return true;
}

void Joystick::updateHook()
{
	Logger::In in("Joystick");

	if (NULL == mpJoystick) {
		return;
	}

	TimeStamp now;
	now.stamp();

	SDL_JoystickUpdate();
// 	std::cout<<mData.axis.size() << ", " << (unsigned int)SDL_JoystickNumAxes(mpJoystick);
//  	assert(mData.axis.size() == (unsigned int)SDL_JoystickNumAxes(mpJoystick));
//  	assert(mData.button.size() == (unsigned int)SDL_JoystickNumButtons(mpJoystick));

	for (unsigned int i = 0; i < mData.axis.size(); ++i) {
		mData.axis[i] =  SDL_JoystickGetAxis(mpJoystick, i) / 32767.0f;
	}

	for (unsigned int i = 0; i < mData.button.size(); ++i) {
		mData.button[i] = (1 == SDL_JoystickGetButton(mpJoystick, i));
	}

	mJoystickDataOut.write(mData);

	TimedDouble speedValue;
	flt steerValue;
	int gearValue;
	AuxDevicesData auxDevicesValue;


	//Speed
	speedValue.data = -mData.axis[1];

	// Logitech Wheel, Gas: Button 7, Bremse: Button8 or the axis from gas/brake panel
	if (mData.button[6]) {
		speedValue.data = 1.0;
	}

	if (mData.button[7]) {
		speedValue.data = -1.0;
	}

	//Steer
	steerValue = mData.axis[0];

	if (mEgoStateIn.connected()) {

		TimedEgoState egoState;
		mEgoStateIn.read(egoState);

		flt curVehicleSpeedInKmh = egoState.vehicleSpeed() * MS_2_KMH;


		if (curVehicleSpeedInKmh >= 30.0) {
			// 		flt result = (-0.9 * ((curVehicleSpeedInKmh - 30) / 30.0) * ((curVehicleSpeedInKmh - 30) / 30.0) + 1);
			// 		cout << "steerValue: " << steerValue * result << endl;
			// reduce steering at higher speed,
			// 		steerValue = steerValue * result;
		}

	}

	int curGear = CarState::GEAR_DRIVE;

	TimedCarState carState;

	if (RTT::NoData != mCarStateIn.read(carState)) {
		curGear = carState.gearPosition;
	}

	if (!CarState::isValidGear(curGear)) {
// 		std::cout<<"default: p";
		gearValue = CarState::GEAR_PARK;
	}
	else {
// 		std::cout<<"cur : " << curGear;
		gearValue = curGear;
	}

	if (mData.button[0]) {
		gearValue = CarState::GEAR_PARK;
	}
	else if (mData.button[1]) {
		gearValue = CarState::GEAR_DRIVE;
	}
	else if (mData.button[2]) {
		gearValue = CarState::GEAR_REVERSE;
	}

	if (mData.button[4]) {
		//hochschalten
		gearValue = CarState::gearUp(curGear);

	}
	else if (mData.button[5]) {
		//runterschalten
		gearValue = CarState::gearDown(curGear);
	}


	if (mVerbose.get()) {
		for (unsigned int i = 0; i < mData.axis.size(); ++i) {
			Logger::log() << Logger::Debug << "Axis " << i << ": " << mData.axis[i] << Logger::endl;
		}

		printOutput();
	}

	steerValue = math::rangeCut(-1.0, steerValue, 1.0);
	speedValue.data = math::rangeCut(-1.0, speedValue.data, 1.0);
	speedValue.stamp();

	mActivationRequestOut.write(true);
	mSpeedOut.write(speedValue);
	mSteerOut.write(steerValue);
	mGearOut.write(gearValue);
	mAuxDevicesOut.write(auxDevicesValue);
}

void Joystick::stopHook()
{
	if (NULL != mpJoystick) {
		SDL_JoystickClose(mpJoystick);
	}

	mpJoystick = NULL;
	mSelectedJoystickNumber = -1;
}

void Joystick::errorHook()
{
}


/** Joystick Menu
 *
 *  lists all connected joystick devices, which can be selected by name or index
 *  with the corresponding methods listet below
 */
bool Joystick::listJoysticks()
{
	int numJoysticks = SDL_NumJoysticks();

	cout << "======================================================================" << endl;

	if (numJoysticks == 0) {
		cout << "\nNo joysticks found!" << endl;
	}
	else {

		for (int i = 0; i < numJoysticks; i++) {
			cout << i << ". " << SDL_JoystickName(i) << endl;
		}

		cout << endl << "Now select a joystick with" << endl;
		cout << "   - \"selectJoystickByName(name)\" or" << endl;
		cout << "   - \"selectByIndex(i)\"." << endl;
	}

	cout << "======================================================================" << endl;
	return true;
}

bool Joystick::selectJoystickByIndex(int i)
{
	if (i < 0) {
		i = 0;
	}
	else if (i > SDL_NumJoysticks()) {
		i = SDL_NumJoysticks();
	}

	return setup(i);
}

bool Joystick::selectJoystickByName(string name)
{
	for (int i = 0; i < SDL_NumJoysticks(); i++) {
		if (SDL_JoystickName(i) == name) {
			return setup(i);
		}
	}

	cout << "Device-name does not exist" << endl;
	return false;
}

bool Joystick::printOutput()
{
	return true;
}
