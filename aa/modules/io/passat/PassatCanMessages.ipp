#include "PassatCanMessages.h"
#include <util/ByteOrder.h>
#include <cstring> // memset

namespace aa
{
namespace modules
{
namespace io
{
namespace passat
{

inline boost::uint16_t fromLittleEndian(boost::uint8_t const * x, unsigned int offset)
{
	using namespace util::byteorder;
	return reorder<boost::uint16_t, LittleEndian, HostEndianness>(*(boost::uint16_t *)(x + offset));
}

inline boost::uint16_t fromBigEndian(boost::uint8_t const * x, unsigned int offset)
{
	using namespace util::byteorder;
	return reorder<boost::uint16_t, BigEndian, HostEndianness>(*(boost::uint16_t *)(x + offset));
}

template<typename MsgType>
void operator>>(WatchdogCommand const & watchdogCommand, MsgType & msg)
{
	msg.can_id = WatchdogCommand::ID;
	msg.can_dlc = 8;
	//init empty

	for (int i = 1; i < msg.can_dlc; i++) {
		msg.data[i] = 0x00;
	}

	//set values
	msg.data[1] = watchdogCommand.mMessageCounter & 0x0f;

	if (watchdogCommand.mEnableGasBrake) {
		msg.data[1] |= 0x10;
	}

	if (watchdogCommand.mEnableGear) {
		msg.data[1] |= 0x20;
	}

	if (watchdogCommand.mActivate) {
		msg.data[1] |= 0x40;
	}

	//get checksum
	boost::uint8_t checksum = 0x00;

	for (uint i = 1; i < 8; i++) {
		checksum ^= msg.data[i];
	}

	msg.data[0] = checksum;
}


template<typename MsgType>
void operator>>(MsgType  const & msg, WatchdogStatus & watchdogStatusMsg)
{
	// Byte 0
	boost::uint8_t checksum = msg.data[0];
	// Byte 1
	checksum ^= msg.data[1];
	watchdogStatusMsg.mMessageCounter = (msg.data[1] & 0x0f);
	watchdogStatusMsg.mGasBrakeEnabled = ((msg.data[1] & 0x10) == 0x10);
	watchdogStatusMsg.mGearEnabled = ((msg.data[1] & 0x20) == 0x20);
	// Byte 2
	checksum ^= msg.data[2];
	watchdogStatusMsg.mWatchdogState = WatchdogStatus::WatchdogState(msg.data[2]);
	// Byte 3
	checksum ^= msg.data[3] ;
	watchdogStatusMsg.mVoltage = ((msg.data[3] & 0x01) == 0x01);
	watchdogStatusMsg.mWatchdogCommand = ((msg.data[3] & 0x02) == 0x02);
	watchdogStatusMsg.mControllerGear = ((msg.data[3] & 0x08) == 0x08);
	watchdogStatusMsg.mControllerBrake = ((msg.data[3] & 0x20) == 0x20);
	watchdogStatusMsg.mControllerThrottle = ((msg.data[3] & 0x40) == 0x40);
	watchdogStatusMsg.mControllerFunction = ((msg.data[3] & 0x80) == 0x80);
	// Byte 4
	checksum ^= msg.data[4];
	watchdogStatusMsg.mStatusBrake = ((msg.data[4] & 0x02) == 0x02);
	watchdogStatusMsg.mStatusThrottle = ((msg.data[4] & 0x04) == 0x04);
	watchdogStatusMsg.mStatusFunction = ((msg.data[4] & 0x08) == 0x08);
	watchdogStatusMsg.mStatusGear = ((msg.data[4] & 0x10) == 0x10);
	// Byte 5
	checksum ^= msg.data[5];
	watchdogStatusMsg.mPowerDownReason = WatchdogStatus::PowerDownReason(msg.data[5]);
	// Byte 6
	checksum ^= msg.data[6];
	watchdogStatusMsg.mStatusActoricBox = ((msg.data[6] & 0x01) == 0x01);
	watchdogStatusMsg.mDriverBrakes = ((msg.data[6] & 0x20) == 0x20);
	watchdogStatusMsg.mDriverAccelerates = ((msg.data[6] & 0x40) == 0x40);
	watchdogStatusMsg.mDriverShiftsGear = ((msg.data[6] & 0x80) == 0x80);//
	// Byte 7
	checksum ^= msg.data[7];
	watchdogStatusMsg.mPoweredDown = ((msg.data[7] & 0x02) == 0x02);
	watchdogStatusMsg.mGasBrakeRequested = ((msg.data[7] & 0x04) == 0x04);
	watchdogStatusMsg.mGearRequested = ((msg.data[7] & 0x08) == 0x08);

	watchdogStatusMsg.mValid = (checksum == 0);
}

template<typename MsgType>
void operator>>(MsgType const & msg, WatchdogBatteryStatus & watchdogBatteriestatusMsg)
{
	boost::uint8_t checksum(msg.data[0]);
	boost::uint8_t testChecksum(0);

	for (int i = 1; i < 6; i++) {
		testChecksum ^= msg.data[i];
	}

	// Byte 0 -> Checksum

	// Byte 1
	watchdogBatteriestatusMsg.mZykluszaehler = (msg.data[1] & 0x0f);

	// Byte 2-3
	watchdogBatteriestatusMsg.mVoltage = (fromLittleEndian(msg.data, 2) * 0.001f) - 32.767f;

	watchdogBatteriestatusMsg.mValid = true; // TODO: Check the checksum (checksum == testChecksum);
}

template<typename MsgType>
void operator>>(BrakeCommand const & bremseReglerMsg, MsgType & msg)
{
	msg.can_id = BrakeCommand::ID;
	msg.can_dlc = 8;
	//init empty

	for (int i = 0; i < msg.can_dlc; i++) {
		msg.data[i] = 0x00;
	}

	//set values
	//byte 0

	if (bremseReglerMsg.mEnableBrakeAssist) {
		msg.data[0] |= 0x40;
	}

	if (bremseReglerMsg.mBitPRCFunc) {
		msg.data[0] |= 0x80;
	}

	//byte 1-2
	uint16_t const pressureDemand = (uint16_t)(bremseReglerMsg.mPressureDemand / 0.05f);
	uint8_t const pressureDemandHighByte = (pressureDemand & 0xff00) >> 8;
	uint8_t const pressureDemandLowByte = (pressureDemand & 0x00ff);

	msg.data[1] = pressureDemandHighByte;
	msg.data[2] = pressureDemandLowByte;

	//byte 3
	msg.data[3] = bremseReglerMsg.mBrakeLightSwitch;

	msg.data[6] = (bremseReglerMsg.mMessageCounter & 0x0f) << 4;
	//get checksum
	boost::uint8_t checksum(0);

	for (uint i = 0; i < 7; i++) {
		checksum += msg.data[i];
	}

	msg.data[7] = checksum;
}

template<typename MsgType>
void operator>>(MsgType const & msg, BrakeStatus1 & statusBremsboosterMsg)
{
	boost::uint8_t checksum(0);

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	statusBremsboosterMsg.mValid = true;

	// Byte 0 - 1
	statusBremsboosterMsg.mActualPressure = 0.01 * fromBigEndian(msg.data, 0);
	// Byte 2
	statusBremsboosterMsg.mTravelSensor = 0.169 * msg.data[2];
	// Byte 3
	statusBremsboosterMsg.mMagnetCurrent = 15.5 * msg.data[3];
	// Byte 4
	statusBremsboosterMsg.mMagnetPwmRatio = msg.data[4];
	// Byte 5
	statusBremsboosterMsg.mMagnetVoltage = 0.07343 * msg.data[5];
	// Byte 6
	statusBremsboosterMsg.mBrakeAssistActive = ((msg.data[6] & 0x01) == 0x01);
}

/**
 * Operator to init a BrakeStatus2 from a msg_data_ind.
 */
template<typename MsgType>
void operator>>(MsgType const & msg, BrakeStatus2 & status2BremsboosterMsg)
{
	boost::uint8_t checksum(0);

	for (int i = 0; i < 7; ++i) {
		checksum += msg.data[i];
	}

	// Byte 0
	status2BremsboosterMsg.mBrakeAssist = ((msg.data[0] & 0x40) == 0x40);
	status2BremsboosterMsg.mBitPRCFunc = ((msg.data[0] & 0x80) == 0x80);

	// Byte 1
	status2BremsboosterMsg.mBCU3StatusError = ((msg.data[1] & 0x01) == 0x01);
// 	status2BremsboosterMsg.mWatchdogCommand = ((msg.data[1] & 0x02) == 0x02);
	status2BremsboosterMsg.mBCU3StatusVoltageLowHigh = ((msg.data[1] & 0x04) == 0x04);
	status2BremsboosterMsg.mBCU3StatusBAWarnLamp = ((msg.data[1] & 0x08) == 0x08);
	status2BremsboosterMsg.mBCU3StatusPRCWarnLamp = ((msg.data[1] & 0x10) == 0x10);
	status2BremsboosterMsg.mBCU3StatusMode = ((msg.data[1] & 0x20) == 0x20);
	status2BremsboosterMsg.mBCU3StatusReleaseSwitch = ((msg.data[1] & 0x40) == 0x40);
	status2BremsboosterMsg.mBCU3StatusPressureControlActive = ((msg.data[1] & 0x80) == 0x80);

	// Byte 2
	status2BremsboosterMsg.mDetectedFailureBCU3intern = ((msg.data[2] & 0x01) == 0x01);
	status2BremsboosterMsg.mDetectedFailurePressureSensor = ((msg.data[2] & 0x02) == 0x02);
	status2BremsboosterMsg.mDetectedFailureTravelSensor = ((msg.data[2] & 0x04) == 0x04);
	status2BremsboosterMsg.mDetectedFailureReleaseSwitch = ((msg.data[2] & 0x08) == 0x08);
	status2BremsboosterMsg.mDetectedFailureSolenoid = ((msg.data[2] & 0x10) == 0x10);
	status2BremsboosterMsg.mDetectedFailurePressControl = ((msg.data[2] & 0x20) == 0x20);
	status2BremsboosterMsg.mDetectedFailureCAN = ((msg.data[2] & 0x40) == 0x40);
	status2BremsboosterMsg.mDetectedFailurePreChargePump = ((msg.data[2] & 0x80) == 0x80);
	// Byte 3-4
	status2BremsboosterMsg.mActualPressure = 0.01f * fromBigEndian(msg.data, 3);
	// Byte 5
	status2BremsboosterMsg.mPressureDemand = 0.5f * msg.data[5];
	// Byte 6
	status2BremsboosterMsg.mMessageCounter = (msg.data[6] >> 4) & 0x0f;
	// Byte 7

	status2BremsboosterMsg.mValid = (checksum == msg.data[7]);
}


/**
 * Operator to init a MsgType from a actoricBoxAnalogOutMsg.
 */
template<typename MsgType>
void operator>>(ThrottleCommand const & cmd, MsgType & msg)
{
	/* TODO Check byte order or similar: 0.01 is full speed */
	memset(&msg, 0, sizeof(msg));

	msg.can_id = ThrottleCommand::ID;
	msg.can_dlc = 8;

	//set values
	//byte 0
	msg.data[1] = cmd.mMessageCounter & 0x0f;

	//byte 1-2
	uint16_t const wantedVoltage = (uint16_t)(cmd.mWantedVoltage / 0.005f);
	uint8_t const wantedVoltageHighByte = (wantedVoltage & 0xff00) >> 8;
	uint8_t const wantedVoltageLowByte = (wantedVoltage & 0x00ff);

	msg.data[2] = wantedVoltageLowByte;
	msg.data[3] = wantedVoltageHighByte;

	//get checksum
	boost::uint8_t checksum(0);

	for (uint i = 1; i < 8; i++) {
		checksum ^= msg.data[i];
	}

	msg.data[0] = checksum;
}

/**
 * Operator to init a ThrottleStatus from a msg_data_ind.
 */
template<typename MsgType>
void operator>>(MsgType const & msg, ThrottleStatus & throttleStatus)
{
	boost::uint8_t checksum(0);

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	throttleStatus.mValid = (checksum == 0);

	/// Byte 0
	// Checksum
	/// Byte 1
	throttleStatus.mMessageCounter = msg.data[1] & 0x0f;

	/// Byte 2: beginning of gaspedal1
	boost::uint16_t gaspedal(msg.data[2]);
	/// Byte 3: end of gaspedal1, start of gaspedal2
	//lower 2 bit are the last of the 10bit-gaspedal1
	boost::uint16_t gaspedalRest;
	gaspedalRest = msg.data[3] & 0x03;
	gaspedal |= (gaspedalRest << 8);
	throttleStatus.mVoltageGaspedal1 = 0.005f * gaspedal;

	//upper 6 bit are the first of the 10bit-gaspedal2
	gaspedal = msg.data[3] >> 2;
	/// Byte 4: end of gaspedal2, start of motor1
	//lower 4 bit are the last of the 10bit-gaspedal2
	gaspedalRest = msg.data[4] & 0x0f;
	gaspedal = gaspedal | (gaspedalRest << 6);
	throttleStatus.mVoltageGaspedal2 = 0.01f * gaspedal; //against specification

	//upper 4 bit are the first of the 10bit-motor1
	boost::uint16_t motor = msg.data[4] >> 4;
	/// Byte 5: end of motor1, start of motor2
	//lower 6 bit are the last of the 10bit-motor1
	boost::uint16_t motorRest = msg.data[5] & 0x3f;
	motor = motor | (motorRest << 4);
	throttleStatus.mVoltageMotor1 = 0.005f * motor;

	//upper 2 bit are the first of the 10bit-motor2
	motor = msg.data[5] >> 6;
	/// Byte 6: end of motor2
	motorRest = msg.data[6];
	motor = motor | (motorRest << 2);
	throttleStatus.mVoltageMotor2 = 0.01f * motor;	//against specification
}
/**
 * Operator to init a MsgType from a GearCommand.
 */
template<typename MsgType>
void operator>>(GearCommand const & cmd, MsgType & msg)
{
	memset(&msg, 0, sizeof(msg));
	msg.can_id = GearCommand::ID;
	msg.can_dlc = 8;
	msg.data[1] = 0x0f & cmd.mCycleCount;
	boost::uint8_t gearpos(0x0f & cmd.mGearPosition);
	msg.data[1] |= gearpos << 4;
	msg.data[2] = 0x0f ^ gearpos;

	for (uint i = 1; i < 8; ++i) {
		msg.data[0] ^= msg.data[i];
	}
}

/**
 * Operator to init a MsgType from a FunctionCommand.
 */
template<typename MsgType>
void operator>>(FunctionCommand const & cmd, MsgType & msg)
{
	memset(&msg, 0, sizeof(msg));

	msg.can_id = FunctionCommand::ID;
	msg.can_dlc = 8;

	//set values
	//byte 1: lower 4 bit = counter
	msg.data[1] = cmd.mCycleCount & 0x0f;

	if (cmd.mHorn) {	// Works
		msg.data[1] |= 0x10;
	}

	msg.data[2] =
		(cmd.mFlashLight		? 0x04 : 0)	// Fernlicht
		| (cmd.mHighBeam		? 0x08 : 0)	// Unknown
		| (cmd.mTurnSignalLeft	? 0x20 : 0)	// Left
		| (cmd.mTurnSignalRight	? 0x40 : 0)	// Right
		| (cmd.mWarningLights	? 0x80 : 0);

	//std::cout << "2: " << (int)msg.data[2] << " ***** " << std::endl;

	msg.data[4] =
		(cmd.mDriverWindowUp		? 0x01 : 0)
		| (cmd.mDriverWindowDown	? 0x02 : 0)
		| (cmd.mCoDriverWindowUp	? 0x04 : 0)
		| (cmd.mCoDriverWindowDown	? 0x08 : 0);

	msg.data[5] =
		(cmd.mWiperIntervalFront	? 1 << 1 : 0)
		| (cmd.mWiperLevel1Front	? 1 << 2 : 0)
		| (cmd.mWiperLevel2Front	? 1 << 3 : 0)
		| (cmd.mWiperWaterFront		? 1 << 4 : 0)
		| (cmd.mWiperBack			? 1 << 5 : 0)
		| (cmd.mWiperWaterBack		? 1 << 6 : 0);


	//get checksum
	boost::uint8_t checksum(0);

	for (uint i = 1; i < 8; i++) {
		checksum ^= msg.data[i];
	}

	msg.data[0] = checksum;
}


/**
 * Operator to init a MsgType from a AbsEspFake.
 */
template<typename MsgType>
void operator>>(AbsEspFake const & cmd, MsgType & msg)
{
	memset(&msg, 0, sizeof(msg));
	msg.can_id = AbsEspFake::ID;
	msg.can_dlc = 8;
	msg.data[0] = 0x0;// & cmd.mCounter;
	msg.data[1]	= 0x0;

	boost::uint16_t speed((boost::uint16_t)(cmd.mSpeed * 100 * 3.6));
	speed = speed << 2;
	msg.data[2] = (boost::uint8_t)(0x00ff & speed);
	msg.data[3] = (boost::uint8_t)((0xff00 & speed) >> 8);
	msg.data[4] = msg.data[5] = msg.data[6] = 0;
	msg.data[7] = 0x0f & cmd.mCounter;
}

/**
 * Operator to init a MsgType from a WheelSpeedsFake.
 */
template<typename MsgType>
void operator>>(WheelSpeedsFake const & cmd, MsgType & msg)
{
	memset(&msg, 0, sizeof(msg));
	msg.can_id = WheelSpeedsFake::ID;
	msg.can_dlc = 8;

	boost::uint16_t speed((boost::uint16_t)(cmd.mSpeedFrontL * 100 * 3.6)); //14.4 km/h = 4 m/s
	speed = speed << 2;
	msg.data[0] = boost::uint8_t(0x00ff & speed);
	msg.data[1] = boost::uint8_t((0xff00 & speed) >> 8);

	speed = boost::uint16_t(cmd.mSpeedFrontR * 100 * 3.6);
	msg.data[2] = boost::uint8_t(0x00ff & speed);
	msg.data[3] = boost::uint8_t((0xff00 & speed) >> 8);

	speed = (boost::uint16_t)(cmd.mSpeedRearL * 100 * 3.6);
	msg.data[4] = boost::uint8_t(0x00ff & speed);
	msg.data[5] = boost::uint8_t((0xff00 & speed) >> 8);

	speed = boost::uint16_t(cmd.mSpeedRearR * 100 * 3.6);
	msg.data[6] = boost::uint8_t(0x00ff & speed);
	msg.data[7] = boost::uint8_t((0xff00 & speed) >> 8);
}

template<typename MsgType>
void operator>>(MsgType const & msg, GearStatus & gearStatus)
{
	/// Byte 0;
	gearStatus.mChosenPos = GearStatus::ChosenPos(msg.data[0] >> 4);
	// Byte 1;
	// Byte 2;
	gearStatus.mCycleCounter = msg.data[2] & 0x0f;
	// Byte 3;
	// Byte 4;
	gearStatus.mValid = ((gearStatus.mChosenPos ^ (msg.data[4] >> 4)) == 0x0f);
}

template<typename MsgType>
void operator>>(MsgType const & msg, ParkAssistData & parkAssistData)
{
	for (unsigned int i = 0; i < 8; i++) {
		parkAssistData.mData[i] = msg.data[i];
	}
}

template<typename MsgType>
void operator>>(MsgType const & msg, SteerAssist3Status & steerAssist3Status)
{
	boost::uint8_t checksum(0);

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	steerAssist3Status.mValid = (checksum == 0);	 // TODO: Enable, if correct. Otherwise write a comment
	steerAssist3Status.mValid = true; // (checksum == 0);

	/// Byte 0: checksum
	/// Byte 1: counter
	steerAssist3Status.mMessageCounter = (boost::uint8_t)((msg.data[1] & 0xf0) >> 4);

	/// Byte 2: beginning of steermoment
	boost::uint16_t steerMomentum(msg.data[2]);
	/// Byte 3: end of steermoment
	boost::uint16_t steerMomentumRest(msg.data[3] & 0x03);
	steerMomentum |= (steerMomentumRest << 8);
	steerAssist3Status.mSteerMomentum = steerMomentum * 0.0147;

	if ((msg.data[3] & 0x04) == 0x04) { //signed?
		steerAssist3Status.mSteerMomentum *= -1;
	}

	steerAssist3Status.mSteerMomentumValid = ((msg.data[3] & 0x08) == 0x08); //valid?

	/// Byte 4: beginning of steerangle
	boost::uint16_t steerAngle(msg.data[4]);

	/// Byte 5: end of steerangle
	boost::uint16_t steerAngleRest(msg.data[5] & 0x0f);
	steerAngle |= (steerAngleRest << 8);
	steerAssist3Status.mSteerAngle = steerAngle * 0.15;

	if ((msg.data[5] & 0x10) == 0x10) {  							//signed?
		steerAssist3Status.mSteerAngle *= -1;
	}

	steerAssist3Status.mSteerAngleValid = ((msg.data[5] & 0x20) == 0x20); //valid?
}


template<typename MsgType>
void operator>>(MsgType const & msg, SignalWipersStatus & signalWipersStatus)
{
	signalWipersStatus.mValid = true; // TODO: Isn't there a checksum?

	/// Byte 0:
	signalWipersStatus.mTurnLeft = ((msg.data[0] & 0x01) == 0x01);
	signalWipersStatus.mTurnRight = ((msg.data[0] & 0x02) == 0x02);
	signalWipersStatus.mFlashLight = ((msg.data[0] & 0x04) == 0x04);
	signalWipersStatus.mHighBeam = ((msg.data[0] & 0x08) == 0x08);
	signalWipersStatus.mHorn = ((msg.data[0] & 0x80) == 0x80);  //Bit7

	/// Byte 1:
	signalWipersStatus.mWipeTip = ((msg.data[1] & 0x01) == 0x01);
	signalWipersStatus.mWipeInterval = ((msg.data[1] & 0x02) == 0x02);
	signalWipersStatus.mWipeLevel1 = ((msg.data[1] & 0x04) == 0x04);
	signalWipersStatus.mWipeLevel2 = ((msg.data[1] & 0x08) == 0x08);

	signalWipersStatus.mWashFront = ((msg.data[1] & 0x10) == 0x10);			//Frontwaschen
	signalWipersStatus.mWashFrontMov = ((msg.data[1] & 0x20) == 0x20);		//Bew Frontwaschen
	signalWipersStatus.mWipeRearInterval = ((msg.data[1] & 0x40) == 0x40);	//Heckinterval
	signalWipersStatus.mWashRear = ((msg.data[1] & 0x80) == 0x80);			//Heckwaschen

	/// Byte 2:
	signalWipersStatus.mIntervalLevel = (msg.data[2] & 0x0f);	//Intervalstufen
}

template<typename MsgType>
void operator>>(MsgType const & msg, Light1Status & light1Status)
{
	boost::uint8_t checksum(0);

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	light1Status.mValid = (checksum == 0); // TODO: Enable, if correct. Otherwise write a comment
	light1Status.mValid = true;	//????

	/// Byte 0:
	light1Status.mParkingLight = ((msg.data[0] & 0x01) == 0x01);
	light1Status.mLowBeam = ((msg.data[0] & 0x02) == 0x02);
	/// Byte 1:
	light1Status.mWarningLights = ((msg.data[1] & 0x08) == 0x08);
	/// Byte 2:
	light1Status.mCycleCounter = ((msg.data[2] & 0xf0) == 0xf0);
}

template<typename MsgType>
void operator>>(MsgType const & msg, Motor1Status & motor1Status)
{
	boost::uint8_t checksum(0);

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	// motor1Status.mValid = (checksum == 0); // TODO: Enable, if correct. Otherwise write a comment
	motor1Status.mValid = true;

	/// Byte 0:
	motor1Status.mPedalStatus = !(msg.data[0] & 0x02);
	motor1Status.mKickDown    = msg.data[0] & 0x04;
	/// Byte 1:

	/// Byte 2 and Byte 3:
	motor1Status.mRevolutionSpeed = fromLittleEndian(msg.data, 2) * 0.25f;
	/// Byte 4:
	/// Byte 5:
	motor1Status.mPedalValue = msg.data[5] * 0.4f;
}

/**TODO: ask Andree Hört: what's the Main Switch ??? **/
template<typename MsgType>
void operator>>(MsgType const & msg, GraMainSwitch & graMainSwitch)
{
	boost::uint8_t checksum = 0;

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	//Checksum:
	graMainSwitch.mValid = (checksum == 0);

	//Byte 0: contains checksum => no need to read, because the checksum was recalculated!

	//Byte 1: the one bit of mMainSwitchSign, mCancel, mDownShort, mUpShort, mDownLong, mUpLong
	graMainSwitch.mMainSwitchSign = ((msg.data[0] & 0x01) == 0x01);		// GRA_Hauptschalt
	graMainSwitch.mCancel 		  = ((msg.data[0] & 0x02) == 0x02);		// GRA_Abbrechen
	graMainSwitch.mDownShort 	  = ((msg.data[0] & 0x04) == 0x04);		// GRA_Down_kurz
	graMainSwitch.mUpShort 		  = ((msg.data[0] & 0x08) == 0x08);		// GRA_Up_kurz
	graMainSwitch.mDownLong 	  = ((msg.data[0] & 0x20) == 0x20);		// GRA_Down_lang
	graMainSwitch.mUpLong 		  = ((msg.data[0] & 0x40) == 0x40); 	// GRA_Up_lang

	//Byte 2: the one bit of mReset, mRecall and the 4 bits of mNewCounter
	graMainSwitch.mReset  	  = ((msg.data[1] & 0x01) == 0x01);		// GRA_Neu_Setzen
	graMainSwitch.mRecall 	  = ((msg.data[1] & 0x02) == 0x02);		// GRA_Recall
	graMainSwitch.mNewCounter = (msg.data[1] & 0xf0);		// GRA_Neu_Zaehler (4 bit)

	//Byte 3: the one bit of
	graMainSwitch.mTipDown = ((msg.data[2] & 0x01) == 0x01);			// GRA_Tip_Down
	graMainSwitch.mTipUp   = ((msg.data[2]  & 0x02) == 0x02);			// GRA_Tip_Up
	graMainSwitch.mTimegap = (msg.data[2] & 0x0c);				// GRA_Zeitluecke (2 bit)
	graMainSwitch.mType    = ((msg.data[2] & 0x20) == 0x20);     		// GRA_Typ_Hauptschalt
}

template<typename MsgType>
void operator>>(MsgType const & msg, Airbag & airbag)
{
	boost::uint8_t checksum = 0;

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	//Checksum:
	airbag.mValid = (checksum == 0);

	//Byte 0: empty

	//Byte 1: all bits of mBelt
	airbag.mSeatbelt = ((msg.data[1] & 0x30) >> 4);		// AB1_Gurt_Fa

	//Byte 2: all bits of mCounter
	airbag.mCounter = ((msg.data[2] & 0xf0) >> 4);		// AB1_Zaehler

	//Byte 3: contains checksum => no need to read, because the checksum was recalculated!
}

/**TODO: test**/
template<typename MsgType>
void operator>>(MsgType const & msg, AbsEsp & absEsp)
{
	boost::uint8_t checksum = 0;

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	//Checksum:
	absEsp.mValid = (checksum == 0);

	//Byte 0:
	absEsp.mAbsSign = ((msg.data[0] & 0x04) == 0x04);		// BR1_ABS_Brems
	absEsp.mEspSign = ((msg.data[0] & 0x08) == 0x08);		// BR1_ESP_Eingr

	//Byte 1: empty

	//Byte 2 and 3: lower 7 bits from mSpeed AND higher 8 bits from mSpeed
	absEsp.mSpeed = ((fromLittleEndian(msg.data, 2) & 0xfffe) >> 1);
	absEsp.mSpeed *= 0.01;	 								// BR1_Rad_kmh
	absEsp.mSpeed /= 3.6;  // convert km/h to m/s

	//Byte 4: empty
	//Byte 5: empty
	//Byte 6: empty

	//Byte 7: 4 bits from mCounter and the one bit of mBackupSign
	absEsp.mCounter    = (msg.data[7] & 0x0f);				//BR1_Zaehler
	absEsp.mBackupSign = ((msg.data[7] & 0x80) == 0x80);	//BR1_Ersatz_Kmh
}


template<typename MsgType>
void operator>>(MsgType const & msg, PathPulse & pathPulse)
{
	boost::uint8_t checksum = 0;

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	//Checksum: Byte 0 (not used!)
	pathPulse.mValid = (checksum == 0);

	//Counter: Byte 1 (from bit 0 to bit 4)
	pathPulse.mCounter = (msg.data[1] & 0x0f);

	//QualityBits: Byte 1 (from bit 4 to bit 7)
	pathPulse.mQBitFrontL = ((msg.data[1] & 0x10) == 0x10);  	// B10_QB_Wegimp_VL
	pathPulse.mQBitFrontR = ((msg.data[1] & 0x20) == 0x20);		// B10_QB_Wegimp_VR
	pathPulse.mQBitRearL  = ((msg.data[1] & 0x40) == 0x40);		// B10_QB_Wegimp_HL
	pathPulse.mQBitRearR  = ((msg.data[1] & 0x80) == 0x80);		// B10_QB_Wegimp_HR

	//get mPulseFrontL
	//Byte 2 (lower 8 bits from pathPulseFrontL)
	pathPulse.mPulseFrontL = boost::uint16_t(msg.data[2]);
	//Byte 3 (higher 2 bits from mPulseFrontL and lower 6 bits from mPulseFrontR)
	pathPulse.mPulseFrontL |= (boost::uint16_t(msg.data[3] & 0x03) << 8);

	//get mPulseFrontR
	pathPulse.mPulseFrontR = (msg.data[3] >> 2);
	//Byte 4 (higher 4 bits from mPulseFrontR and lower 4 bits from mPulseRearL)
	pathPulse.mPulseFrontR |= (boost::uint16_t(msg.data[4] & 0x0f) << 6);

	//get mPulseRearL
	pathPulse.mPulseRearL = (msg.data[4] >> 4);
	//Byte 5 (higher 6 bits from mPulseRearL and lower 2 bits from mPulseRearR)
	pathPulse.mPulseRearL |= (boost::uint16_t(msg.data[5] & 0x3f) << 4);

	//get mPulseRearR
	pathPulse.mPulseRearR = (msg.data[5] >> 6);
	//Byte 6 (higher 8 bit from mPulseRearR)
	pathPulse.mPulseRearR |= boost::uint16_t(msg.data[6]) << 2;

}

template<typename MsgType>
void operator>>(MsgType const & msg, WheelSpeeds & wheelSpeeds)
{
	boost::uint16_t wheelSpeed;

	/// Byte 0:  signed bit and lower 7 bits of wheel FrontL
	wheelSpeeds.mSignFrontL = (msg.data[0] & 0x01);
	wheelSpeed = 0;
	wheelSpeed = (boost::uint8_t)(msg.data[0] >> 1);
	/// Byte 1: upper 8 bit of wheel FrontL
	boost::uint16_t wheelSpeedRest = 0;
	wheelSpeedRest = msg.data[1];
	wheelSpeed |= (wheelSpeedRest << 7);

	if (wheelSpeeds.mSignFrontL == 1) {
		wheelSpeed = wheelSpeed * -1;
	}

	wheelSpeeds.mSpeedFrontL = wheelSpeed * 0.01 / 3.6;		//meters per second
	/// Byte 2:  signed bit and lower 7 bits of wheel FrontR
	wheelSpeeds.mSignFrontR = (msg.data[2] & 0x01);
	wheelSpeed = (boost::uint8_t)(msg.data[2] >> 1);
	/// Byte 3: upper 8 bit of wheel FrontR
	wheelSpeedRest = msg.data[3];
	wheelSpeed |= (wheelSpeedRest << 7);

	if (wheelSpeeds.mSignFrontR == 1) {
		wheelSpeed = wheelSpeed * -1;
	}

	wheelSpeeds.mSpeedFrontR = wheelSpeed * 0.01 / 3.6;		//meters per second
	/// Byte 4:  signed bit and lower 7 bits of wheel RearL
	wheelSpeeds.mSignRearL = (msg.data[4] & 0x01);
	wheelSpeed = (boost::uint8_t)(msg.data[4] >> 1);
	/// Byte 5: upper 8 bit of wheel RearL
	wheelSpeedRest = msg.data[5];
	wheelSpeed |= (wheelSpeedRest << 7);

	if (wheelSpeeds.mSignRearL == 1) {
		wheelSpeed = wheelSpeed * -1;
	}

	wheelSpeeds.mSpeedRearL = wheelSpeed * 0.01 / 3.6;		//meters per second
	/// Byte 6:  signed bit and lower 7 bits of wheel RearR
	wheelSpeeds.mSignRearR = (msg.data[6] & 0x01);
	wheelSpeed = (boost::uint8_t)(msg.data[6] >> 1);
	/// Byte 7: upper 8 bit of wheel RearR
	wheelSpeedRest = msg.data[7];
	wheelSpeed |= (wheelSpeedRest << 7);

	if (wheelSpeeds.mSignRearR == 1) {
		wheelSpeed = wheelSpeed * -1;
	}

	wheelSpeeds.mSpeedRearR = wheelSpeed * 0.01 / 3.6;		//meters per second

	wheelSpeeds.mValid = true; // TODO: Checksum?
}

template<typename MsgType>
void operator>>(MsgType const & msg, SteeringWheelSpeed & steeringWheelSpeed)
{
	boost::uint8_t checksum(0);

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	steeringWheelSpeed.mValid = (checksum == 0);

	float value = 0;

	/// Byte 0 and 1:  15 bits of steering wheel angle
	boost::uint16_t currentUShort = fromLittleEndian(msg.data, 0);
	value = (currentUShort & 0x7fff);

	if ((currentUShort & 0x8000) == 0x8000) {
		value = value * (-1);
		steeringWheelSpeed.mSteeringWheelAngleSign = true;
	}

	steeringWheelSpeed.mSteeringWheelAngle = value * 0.04375;

	/// Byte 2 and 3:  15 bits of steering wheel speed
	currentUShort = fromLittleEndian(msg.data, 2);
	value = (currentUShort & 0x7fff);

	if ((currentUShort & 0x8000) == 0x8000) {
		value = value * (-1);
		steeringWheelSpeed.mSteeringWheelSpeedSign = true;
	}

	steeringWheelSpeed.mSteeringWheelSpeed = value * 0.04375;

	/// Byte 4:  LW1 ID
	steeringWheelSpeed.id = msg.data[4];

	/// Byte 5:  LW1_Zaehler
	steeringWheelSpeed.mCounter = ((msg.data[5] & 0xf0) >> 4);
}

template<typename MsgType>
void operator>>(MsgType const & msg, TargetGear & targetGear)
{
	//Byte 0: empty

	//Byte 1: lower 4 bit's for mTargetPos and higher 4 bit's for mSelectedPos
	targetGear.mTargetPos   = TargetGear::TargetPos(msg.data[0] & 0x0f);
	targetGear.mSelectedPos = TargetGear::SelectedPos((msg.data[0] & 0xf0) >> 4);

	//Byte 2: empty
	//Byte 3: empty
	//Byte 4: empty
	//Byte 5: empty

	//Byte 6: 4 bit's of mCounter
	targetGear.mCounter = ((msg.data[6] & 0x78) >> 3);

}

template<typename MsgType>
void operator>>(MsgType const & msg, Acceleration & acceleration)
{
	boost::uint8_t checksum = 0;

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	//Checksum:
	acceleration.mValid = (checksum == 0);

	//Byte 0: checksum -> not used!

	//Byte 1: mCounter
	acceleration.mCounter = msg.data[1] & 0x0f;

	//Byte 2: empty
	//Byte 3: empty

	//Byte 4 and 5: lower 8 bist's from mCurrentAcc AND higher 1 bit from mCurrentAcc AND 1 bit mAccSign
	boost::uint16_t currentUShort = fromLittleEndian(msg.data, 4);
	acceleration.mCurrentAcc  = (currentUShort & 0x01ff);
	acceleration.mCurrentAcc  = acceleration.mCurrentAcc * 0.02 - 7.22;

	acceleration.mAccSign     = ((currentUShort & 0x0400) == 0x0400);

	//Byte 6 and 7: lower 8 bist's from mLogitudinalAcc AND higher 2 bist's from mLogitudinalAcc
	currentUShort = fromLittleEndian(msg.data, 6);
	acceleration.mLogitudinalAcc  = (currentUShort & 0x03ff);
	acceleration.mLogitudinalAcc  = acceleration.mLogitudinalAcc * 0.03125 - 16 ;
}

template<typename MsgType>
void operator>>(MsgType const & msg, YawAndBrakePressure & yawAndBrakePressure)
{
	boost::uint8_t checksum = 0;

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	boost::uint16_t tmpBits16, tmpDoubleBits;

	//Checksum:
	yawAndBrakePressure.mValid = (checksum == 0);

	//Byte 0: lower 8 bit's from mYawVelocity
	tmpDoubleBits = msg.data[0];

	//Byte 1: higher 6 bit's from mYawVelocity, 1 bit from mStatusYawVelocity and 1 bit from Sign Velocity
	tmpBits16 = boost::uint16_t(msg.data[1] & 0x3f);
	yawAndBrakePressure.mYawRate  = 0.01 * ((tmpBits16 << 8) | tmpDoubleBits);

	yawAndBrakePressure.mYawRateValid = ((msg.data[1] & 0x40) != 0x40);

	//Sign
	if ((msg.data[1] & 0x80) == 0x80) {
		yawAndBrakePressure.mYawRate *= -1;
	}

	//Byte 2: lower 8 bit's from mBrakePressure
	tmpDoubleBits = msg.data[2];

	//Byte 3: higher 4 bit's from mBrakePressure 1 bit from mPressureValid and 1 bit from Sign Pressure
	tmpBits16 = (msg.data[3] & 0x0f);
	yawAndBrakePressure.mBrakePressure  = 0.1 * ((tmpBits16 << 8) | tmpDoubleBits);
	yawAndBrakePressure.mPressureValid = ((msg.data[3] & 0x20) != 0x20);

	//Sign
	if ((msg.data[3] & 0x80) == 0x80) {
		yawAndBrakePressure.mBrakePressure *= -1;
	}

	//Byte 4: empty
	//Byte 5: empty

	//Byte 6: mCounter
	yawAndBrakePressure.mCounter = ((msg.data[6] & 0xf0) >> 4);

	//Byte 7: Checksum -> not used!

}

template<typename MsgType>
void operator>>(MsgType const & msg, AccelerationAndTeeth & accelerationAndTeeth)
{
	boost::uint8_t checksum = 0;

	for (int i = 0; i < msg.can_dlc; i++) {
		checksum ^= msg.data[i];
	}

	//Checksum:
	accelerationAndTeeth.mValid = (checksum == 0);

	//Byte 0: all 6 bit's from mNrOfTeeth OR all 8 bits's from mLateralAcc; DEPENDET on the mMux Value in Byte 1

	//Byte 1: 1 bit mMux Value
	accelerationAndTeeth.mMux = ((msg.data[1] & 0x01) == 0x01);

	//set mNrOfTeeth OR mLateralAccelaration, dependet form the mMux value
	if (accelerationAndTeeth.mMux) {
		accelerationAndTeeth.mNrOfTeeth  = (boost::uint8_t)(msg.data[0] & 0x3f);
	}
	else {
		accelerationAndTeeth.mLateralAcceleration = msg.data[0] * 0.01 - 1.27;
		accelerationAndTeeth.mLateralAcceleration *= 9.81; // convert g to m/s²
	}


	//Byte 2: empty

	//Byte 3: all 4 bit's Counter
	accelerationAndTeeth.mCounter = (boost::uint8_t)((msg.data[3] & 0xf0) >> 4);

	//Byte 4: empty
	//Byte 5: empty

	//Byte 6: 1 bit of mLateralAccValid
	accelerationAndTeeth.mLateralAccValid = ((msg.data[6] & 0x40) != 0x40);
}


} // namespace
}
}
}