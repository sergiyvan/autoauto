#pragma once
#include <boost/cstdint.hpp>
#include <modules/io/anagate/AnaGateMessages.h>
#include <string>
#include <iostream>
// #include <math/Types.h>
namespace math
{
typedef double flt;
}

namespace aa
{
namespace modules
{
namespace io
{
namespace passat
{

using ::modules::io::anagate::CanMessage;

/**
 * Contains structs for can-messages according to the document "Funktionsbeschreibung Aktorik-Schnittstelle über Watchdog-Gateway"
 *
 * Version 1.0
 *
 * \author David Latotzky
 * \author Daniel Göhring
 * \author Gretta Hohl
 *
 */

class WatchdogCommand;
/**
 * Operator to init a msg_data_ind from a  WatchdogCommand.
 */
void operator>>(WatchdogCommand const & watchdogCommand, CanMessage & can_msg);


/**
 * Aktivierung/Deaktivierung (Freigabe der Aktorik)
 * Die Aktivierung/Deaktivierung (Zustandsübergänge 11-12/12-11) erfolgt über die  Botschaft WatchdogCommand.
 * Welcher Aktorik-Breich freigegeben werden soll,
 * muss dem Watchdog über die Signale FreigabeanforderungLaengsAktorik und FreigabeanforderungWaehlhebel mitgeteilt werden
 * (Die Auswertung der beiden Freigabesignale erfolgt nur in den Stati SpannungAn und Funktionsbereit.).
 * Die eigentliche Aktivierung/Deaktivierung erfolgt dann über das Signal Aktivierung:
 *
 * Botschaft: 	WatchdogCommand
 * Identifier:	001 H
 * Länge:	8 Byte
 * Zykluszeit:	50 ms
 */

class WatchdogCommand
{
public:
	static boost::uint32_t const ID = 0x1;
	static boost::uint32_t const CYCLE_TIME = 50;
	template<typename MsgType> friend void operator>>(WatchdogCommand const & watchdogCommand, MsgType & msg);

	WatchdogCommand()
		: mMessageCounter(0)
		, mEnableGasBrake(false)
		, mEnableGear(false)
		, mActivate(false)
	{}

	/** sets messagecount.*/
	void setMessageCounter(boost::uint8_t c) {
		mMessageCounter = c;
	}

	/** set true if gas/brake control should be enabled.*/
	void setEnableGasBrake(bool f) {
		mEnableGasBrake = f;
	}

	/** set true if gear control should be enabled.*/
	void setEnableGear(bool f) {
		mEnableGear = f;
	}

	/** set true if controls shozuld be enabled*/
	void setActivate(bool a) {
		mActivate = a;
	}

private:
	boost::uint8_t mMessageCounter;
	bool mEnableGasBrake;
	bool mEnableGear;
	bool mActivate;
};

/**
 * Operator to init a WatchdogCommand from a msg_data_ind.
 */
// void operator>>( msg_data_ind  const & can_msg, WatchdogCommand  & watchdogCommand)
// {
// 	watchdogCommand.mId = can_msg.can_id;
// 	QByteArray ar;
// 	ar.resize(8);
// 	for(int i = 0; i<can_msg.size; i++) {
// 		ar[i]=can_msg.data[i];
// 	}
// 	QDataStream datads(ar);
// 	datads.setVersion(QDataStream::Qt_4_5);
// 	datads.setByteOrder(QDataStream::LittleEndian);
// 	boost::uint8_t data;
// 	datads >> data;
// 	watchdogCommand.mZykluszaehler = (data & 0x0f);
// 	watchdogCommand.mFreigabeanforderungLaengsAktorik = ((data & 0x10) == 0x10);
// 	watchdogCommand.mFreigabeanforderungWaehlhebel = ((data & 0x20) == 0x20);
// 	watchdogCommand.mAktivierung = ((data & 0x40) == 0x40);
// }

/**
 *
 * Statusbotschaft
 * Vom Watchdog-GW wird zyklisch (50 ms) und bei jeder Änderung des Zustandes auf dem Regler-CAN eine Statusbotschaft
 * (WatchdogStatus) gesendet:
 *
 * Botschaft: 	WatchdogStatus	Sender:	Watchdog-Gateway
 * Identifier:	003 H
 * Länge:	8 Byte
 * Zykluszeit:	50 ms
 *
 */

class WatchdogStatus
{
public:
	static boost::uint32_t const ID = 0x003;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, WatchdogStatus & watchdogStatusMsg);

	WatchdogStatus()
		: mValid(false)
		, mMessageCounter(0)
		, mGasBrakeEnabled(false)
		, mGearEnabled(false)
		, mWatchdogState(Off)
		, mVoltage(false)
		, mWatchdogCommand(false)
		, mControllerGear(false)
		, mControllerBrake(false)
		, mControllerThrottle(false)
		, mControllerFunction(false)
		, mStatusBrake(false)
		, mStatusThrottle(false)
		, mStatusFunction(false)
		, mStatusGear(false)
		, mPowerDownReason(NoPowerOff)
		, mStatusActoricBox(false)
		, mDriverBrakes(false)
		, mDriverAccelerates(false)
		, mDriverShiftsGear(false)
		, mPoweredDown(false)
		, mGasBrakeRequested(false)
		, mGearRequested(false)
	{}

	enum WatchdogState {
		Off = 1,			//Aus
		S1Off = 2,			//S1Aus
		S2Off = 4,			//S2Aus
		VoltageOn = 8,			//SpannungAn
		VoltageOn_Error = 9,		//SpannungAn_Fehler
		OperationReady = 11,		//Funktionsbereit
		ActoricReady = 12		//Ansteuerung frei
	};

	static char const * const watchdogStateString(WatchdogState state) {
		switch (state) {
		case Off:
			return "Off";

		case S1Off:
			return "S1Off";

		case S2Off:
			return "S2Off";

		case VoltageOn:
			return "VoltageOn";

		case VoltageOn_Error:
			return "VoltageOn_Error";

		case OperationReady:
			return "OperationReady";

		case ActoricReady:
			return "ActoricReady";

		default:
			return "-";
		}
	}

	enum PowerDownReason {
		NoPowerOff = 0,		//KeineAbschaltung
		IgnitionOff = 1,	//ZuendungAus
		S1 = 2,
		S2 = 3,
		Voltageproblem = 4,		//Spannungsproblem
		ErrorWatchdogCommand = 5, 	//FehlerWatchdogCommand
		ErrorControllerBrake =  7,  	//FehlerReglerBremse
		ErrorControllerGaspedal = 8,	//FehlerReglerGaspedal
		MalfunctionBrake = 10,	//FunktionsstoerungBremse
		MalfunctionGaspedal = 11,	//FunktionsstoerungGaspedal
		Deactivation = 12,		//Deaktivierung
		DriverIntervention = 13,	//Fahrereingriff
		MalfunctionActoricBox = 16,	//FunktionsstoerungAktorikBox
		NotEnabled = 17,		//KeineFreigaben
		ErrorControllerGear = 18,	//FehlerReglerGetriebe
		MalfunctionGear = 19,	//FunktionsstoerungGetriebe
		Unknown = 255		//UnbekannteUrsache
	};

	static char const * const powerDownReasonString(PowerDownReason reason) {
		switch (reason) {
		case NoPowerOff:
			return "NoPowerOff";

		case IgnitionOff:
			return "IgnitionOff";

		case S1:
			return "S1";

		case S2:
			return "S2";

		case Voltageproblem:
			return "Voltageproblem";

		case ErrorWatchdogCommand:
			return "ErrorWatchdogCommand";

		case ErrorControllerBrake:
			return "ErrorControllerBrake";

		case ErrorControllerGaspedal:
			return "ErrorControllerGaspedal";

		case MalfunctionBrake:
			return "MalfunctionBrake";

		case MalfunctionGaspedal:
			return "MalfunctionGaspedal";

		case Deactivation:
			return "Deactivation";

		case DriverIntervention:
			return "DriverIntervention";

		case MalfunctionActoricBox:
			return "MalfunctionActoricBox";

		case NotEnabled:
			return "NotEnabled";

		case ErrorControllerGear:
			return "ErrorControllerGear";

		case MalfunctionGear:
			return "MalfunctionGear";

		case Unknown:
			return "Unknown";

		default:
			return "-";
		}
	}

	/** Returns cycle-cout for packets*/
	int messageCounter() const {
		return mMessageCounter;
	}

	/** */
	bool gasBrakeEnabled() const {
		return mGasBrakeEnabled;
	}

	/** */
	bool gearEnabled() const {
		return mGearEnabled;
	}

	/** */
	WatchdogState watchdogState() const {
		return mWatchdogState;
	}

	/** */
	bool voltage() const {
		return mVoltage;
	}

	/** */
	bool watchdogCommand() const {
		return mWatchdogCommand;
	}

	/** */
	bool controllerGear() const {
		return mControllerGear;
	}

	/** */
	bool controllerBrake() const {
		return mControllerBrake;
	}

	/** */
	bool controllerThrottle() const {
		return mControllerThrottle;
	}

	/** */
	bool controllerFunction() const {
		return mControllerFunction;
	}

	/** */
	bool statusBrake() const {
		return mStatusBrake;
	}

	/** */
	bool statusThrottle() const {
		return mStatusThrottle;
	}

	/** */
	bool statusFunction() const {
		return mStatusFunction;
	}

	/** */
	bool statusGear() const {
		return mStatusGear;
	}

	/** */
	PowerDownReason powerOffReason() const {
		return mPowerDownReason;
	}

	/** */
	bool statusActoricBox() const {
		return mStatusActoricBox;
	}

	/** */
	bool driverBrakes() const {
		return mDriverBrakes;
	}

	/** */
	bool driverAccelerates() const {
		return mDriverAccelerates;
	}

	/** */
	bool driverShiftsGear() const {
		return mDriverShiftsGear;
	}

	/** */
	bool poweredDown() const {
		return mPoweredDown;
	}

	/** */
	bool enableRequestGasBrake() const {
		return mGasBrakeRequested;
	}

	/** */
	bool enableRequestGear() const {
		return mGearRequested;
	}

	bool valid() const {
		return mValid;
	}

	/** returns a string describing the message*/
	std::string toString() const;
private:
	bool mValid;
	boost::uint8_t mMessageCounter;//mZykluszaehler
	bool mGasBrakeEnabled;///mFreigabeLaengsAktorik, true: freigegeben
	bool mGearEnabled;///mFreigabeWaehlhebel, true: freigegeben
	WatchdogState mWatchdogState;///mWatchdogZustand, Use WatchdogStatus::WatchdogState to interprete values. Values not in WatchdogState are possible ('reserved').
	bool mVoltage;///mSpannungsUeberw, false: error, true: OK
	bool mWatchdogCommand;///false: error, true: OK
	bool mControllerGear;///mReglerWaehlhebel, false: error, true: OK
	bool mControllerBrake;///mReglerBremse, false: error, true: OK
	bool mControllerThrottle;///mReglerGaspedal, false: error, true: OK
	bool mControllerFunction;///mReglerFunktion, false: error, true: OK (Blinker etc.)
	bool mStatusBrake;///mStatusBremse, false: error, true: OK
	bool mStatusThrottle;///mStatusGaspedal, false: error, true: OK
	bool mStatusFunction;///mStatusFunktion, false: error, true: OK
	bool mStatusGear;///mStatusWaehlhebel, false: error, true: OK
	PowerDownReason mPowerDownReason;///mAbschaltursache, Use WatchdogStatus::PowerOffReason to interprete values. Values not in PowerOffReason are possible ('reserved').
	bool mStatusActoricBox;///mStatusAktorikBox, false: error, true: OK
	bool mDriverBrakes;///mFahrerBremst, false: nein, true: ja
	bool mDriverAccelerates;///mFahrerGibtGas, false: nein, true: ja
	bool mDriverShiftsGear;///mFahrerSchaltet, false: nein, true: ja
	bool mPoweredDown;///mAbschaltbedingung, false: nein, true: ja
	bool mGasBrakeRequested;///mFreigabeanforderungLaengsAktorik, false: nein, true: ja
	bool mGearRequested;///mFreigabeanforderungWaehlhebel, false: nein, true: ja
};

/**
 * Operator to init a WatchdogStatus from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, WatchdogStatus  & watchdogStatusMsg);

/**
 *
 * WatchdogBatteriestatus
 * Botschaft: 	WatchdogBatteriestatus	Sender:	Watchdog-Gateway
 * Identifier:	004 H
 * Länge:	8 Byte
 * Zykluszeit:	50 ms
 *
 */

class WatchdogBatteryStatus
{
public:
	static boost::uint32_t const ID = 0x004;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, WatchdogBatteryStatus & watchdogBatteriestatusMsg);

	WatchdogBatteryStatus()
		: mValid(false)
	{}

	/** */
	math::flt voltage() const {
		return mVoltage;
	}

	bool valid() const {
		return mValid;
	}

	std::string toString() const;
private:
	boost::uint8_t mZykluszaehler;
	math::flt mVoltage;///-32,767..32,768 V
	bool mValid;
};

/**
 * Operator to init a WatchdogBatteryStatus from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, WatchdogBatteryStatus  & watchdogBatteriestatusMsg);

/**
 *
 * BCU3IN_ACC (Regler)
 * CAN-Bus:	Regler-CAN
 * Identifier:	151 H
 * Länge:	8 Byte
 * Zykluszeit:	10 ms
 * Timeout: 30ms
 *
 */

class BrakeCommand
{
public:
	enum {
		ID = 0x151,
		CYCLE_TIME = 8
	};

	enum BlsState {
		BlsNotActive = 0,
		BlsActive = 1,
		Undefined = 2,
		Failure = 3
	};

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(BrakeCommand const & bremseReglerMsg, MsgType & can_msg);

	BrakeCommand()
		: mEnableBrakeAssist(false)
		, mBitPRCFunc(true)
		, mPressureDemand(0.0f)
		, mBrakeLightSwitch(BlsNotActive)
		, mMessageCounter(0)
	{}

	BrakeCommand(uint cycleCounter, math::flt pressureDemand)
		: mEnableBrakeAssist(false)
		, mBitPRCFunc(true)
		, mPressureDemand(pressureDemand)
		, mBrakeLightSwitch(BlsNotActive)
		, mMessageCounter(cycleCounter)
	{}

	/** */
	// void enableBrakeAssist(bool b) { mEnableBrakeAssist = b; } Fixed according to vw-docs

	void setPressureDemand(math::flt p) {
		mPressureDemand = p;
	}

	// void setBrakeLightSwitch(BlsState b) { mBrakeLightSwitch = b; } Fixed according to docs

	void setMessageCounter(uint m) {
		mMessageCounter = m;
	}

private:
	bool mEnableBrakeAssist;	/// BitBAFunc: false: disable BA function, true: enable BA function. (Anm. = false)
	bool mBitPRCFunc;			/// false: disable PRC function, true: enable PRC function. (Anm: Für Ansteuerung auf true setzen!)
	math::flt mPressureDemand;		/// bar
	BlsState mBrakeLightSwitch;	///see enum BlsState; 0:BLS not active, 1:BLS active, 2:undefined, 3:failure (Should be 0)
	boost::uint8_t mMessageCounter;
};

/**
 * Operator to init a msg_data_ind from a BrakeCommand.
 */
void operator>>(BrakeCommand  const & bremseReglerMsg, CanMessage & can_msg);

/**
 *
 * BCU3OUT_ACC1 (Status Bremsbooster)
 * CAN-Bus:	Aktorik-CAN (gespiegelt auf Regler-CAN)
 * Identifier:	321 H
 * Länge:	8 Byte
 * Zykluszeit:	28 ms
 */

class BrakeStatus1
{
public:
	static boost::uint32_t const ID = 0x321;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, BrakeStatus1 & statusBremsboosterMsg);

	BrakeStatus1()
		: mValid(false)
	{}

	BrakeStatus1(math::flt actualPressure)
		: mValid(false) {
		mActualPressure = actualPressure;
	}

	/** 0..600 bar */
	math::flt actualPressure() const {
		return mActualPressure;
	}

	/** 0..43 mm */
	math::flt travelSensor() const {
		return mTravelSensor;
	}

	/** 0..3950 mA */
	math::flt magnetCurrent() const {
		return mMagnetCurrent;
	}

	/** 0..100 % */
	uint magnetPwmRatio() const {
		return mMagnetPwmRatio;
	}

	/** 0..18 V  */
	math::flt magnetVoltage() const {
		return mMagnetVoltage;
	}

	/** false: BA is not firing, true: BA is firing. */
	bool brakeAssistActive() const {
		return mBrakeAssistActive;
	}

	bool valid() const {
		return mValid;
	}

	std::string toString() const;

private:
	math::flt mActualPressure;/// 0..600 bar
	math::flt mTravelSensor;///0..43 mm
	math::flt mMagnetCurrent;///0..3950 mA
	uint mMagnetPwmRatio;///0..100 %
	math::flt mMagnetVoltage;///0..18 V
	bool mBrakeAssistActive;/// false: BA is not firing, true: BA is firing
	bool mValid;
};

/**
 * Operator to init a WatchdogBatteryStatus from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, BrakeStatus1  & statusBremsboosterMsg);

/**
 *
 * BCU3OUT_ACC2 (Status Bremsbooster)
 * CAN-Bus:	Aktorik-CAN (gespiegelt auf Regler-CAN)
 * Identifier:	221 H
 * Länge:	8 Byte
 * Zykluszeit:	28 ms
 */

class BrakeStatus2
{

public:
	static boost::uint32_t const ID = 0x221;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, BrakeStatus2  & statusBremsboosterMsg);

	BrakeStatus2()
		: mValid(false)
	{}

	/** */
	bool brakeAssist() const {
		return mBrakeAssist;
	}

	/** */
	bool bitPRCFunc() const {
		return mBitPRCFunc;
	}

	/** */
	bool bcu3StatusError() const {
		return mBCU3StatusError;
	}

	/** */
	bool bcu3StatusVoltageLowHigh() const {
		return mBCU3StatusVoltageLowHigh;
	}

	/** */
	bool bcu3StatusBAWarnLamp() const {
		return mBCU3StatusBAWarnLamp;
	}

	/** */
	bool bcu3StatusPRCWarnLamp() const {
		return mBCU3StatusPRCWarnLamp;
	}

	/** */
	bool bcu3StatusMode() const {
		return mBCU3StatusMode;
	}

	/** */
	bool bcu3StatusReleaseSwitch() const {
		return mBCU3StatusReleaseSwitch;
	}

	/** */
	bool bcu3StatusPressureControlActive() const {
		return mBCU3StatusPressureControlActive;
	}

	/** */
	bool detectedFailureBCU3intern() const {
		return mDetectedFailureBCU3intern;
	}

	/** */
	bool detectedFailurePressureSensor() const {
		return mDetectedFailurePressureSensor;
	}

	/** */
	bool detectedFailureTravelSensor() const {
		return mDetectedFailureTravelSensor;
	}

	/** */
	bool detectedFailureReleaseSwitch() const {
		return mDetectedFailureReleaseSwitch;
	}

	/** */
	bool detectedFailureSolenoid() const {
		return mDetectedFailureSolenoid;
	}

	/** */
	bool detectedFailurePressControl() const {
		return mDetectedFailurePressControl;
	}

	/** */
	bool detectedFailureCan() const {
		return mDetectedFailureCAN;
	}

	/** */
	bool detectedFailurePreChargePump() const {
		return mDetectedFailurePreChargePump;
	}

	/** Second sensor */
	math::flt actualPressure() const {
		return mActualPressure;
	}

	/** */
	math::flt pressureDemand() const {
		return mPressureDemand;
	}

	/** */
	uint messageCounter() const {
		return mMessageCounter;
	}

	bool valid() const {
		return mValid;
	}

	std::string toString() const;

private:
	bool mBrakeAssist;/// false: disable BA function, true: enable BA function. (Anm. = Spiegel der Botschaft BrakeCommand (BCU3IN_ACC))
	bool mBitPRCFunc;/// false: disable PRC function, true: enable PRC function. (Anm. = Spiegel der Botschaft BrakeCommand (BCU3IN_ACC))

	bool mBCU3StatusError;
	bool mBCU3StatusVoltageLowHigh;
	bool mBCU3StatusBAWarnLamp;
	bool mBCU3StatusPRCWarnLamp;
	bool mBCU3StatusMode;
	bool mBCU3StatusReleaseSwitch;
	bool mBCU3StatusPressureControlActive;
	bool mDetectedFailureBCU3intern;
	bool mDetectedFailurePressureSensor;
	bool mDetectedFailureTravelSensor;
	bool mDetectedFailureReleaseSwitch;
	bool mDetectedFailureSolenoid;
	bool mDetectedFailurePressControl;
	bool mDetectedFailureCAN;
	bool mDetectedFailurePreChargePump;
	math::flt mActualPressure;///0..600 bar, actual measure pressure
	math::flt mPressureDemand;///0..127,5 bar, Accepted pressure demand
	uint mMessageCounter;/// 0..15
	bool mValid;
};

void operator>>(CanMessage  const & can_msg, BrakeStatus2  & statusBremsboosterMsg);

/**
 *
 * AktorikBox_AnlgAus (Regler)
 * CAN-Bus:	Regler-CAN
 * Identifier:	A3 H
 * Länge:	8 Byte
 * Zykluszeit:	20 ms
 * Timeout: 60ms
 */
class ThrottleCommand
{
public:
	static boost::uint32_t const ID = 0xa3;
	static boost::uint32_t const CYCLE_TIME = 20;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(ThrottleCommand const & actoricBoxThrottleAnalogOutMsg, MsgType & can_msg);

	ThrottleCommand(uint32_t cycleCounter, math::flt wantedVoltage)
		: mWantedVoltage(wantedVoltage)
		, mMessageCounter(cycleCounter)
	{}

protected:
	ThrottleCommand()
		: mWantedVoltage(0.0)
		, mMessageCounter(0)
	{}

private:
	math::flt mWantedVoltage;///0,75..4,00 V Normalbetrieb, 4,40 V Kickdown (doc: U_soll)
	boost::uint8_t mMessageCounter;/// 0..15 (doc: Zykluszaehler)
};

void operator>>(ThrottleCommand const & actoricBoxThrottleAnalogOutMsg, CanMessage  & can_msg);

/**
 *
 * AktorikBox_Spannung1 (Status Gaspedal)
 * CAN-Bus:	Aktorik-CAN (gespiegelt auf Regler-CAN)
 * Identifier:	A1 H
 * Länge:	8 Byte
 * Zykluszeit:	20 ms
 * Timeout: 60ms
 */
class ThrottleStatus
{
public:
	static boost::uint32_t const ID = 0xA1;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, ThrottleStatus & throttleStatus);

	ThrottleStatus()
		: mValid(false)
	{}

	ThrottleStatus(math::flt actualVoltage)
		: mValid(false) {
		mVoltageGaspedal1 = actualVoltage;
		mVoltageGaspedal2 = actualVoltage;
		mVoltageMotor1 = actualVoltage;
		mVoltageMotor2 = actualVoltage;
	}


	/** */
	boost::uint8_t messageCounter() const {
		return mMessageCounter;
	}

	/** */
	math::flt voltageGasPedal1() const {
		return mVoltageGaspedal1;
	}

	/** */
	math::flt voltageGasPedal2() const {
		return mVoltageGaspedal2;
	}

	/** */
	math::flt voltageMotor1() const {
		return mVoltageMotor1;
	}

	/** */
	math::flt voltageMotor2() const {
		return mVoltageMotor2;
	}

	bool valid() const {
		return mValid;
	}

	std::string toString() const;

private:
	boost::uint8_t mMessageCounter;/// 0..15 (doc: Zykluszaehler)
	math::flt mVoltageGaspedal1;///0.. 5,12 V, (doc: U1_Gaspedal , Spannung am Ausgang des Gaspedals)
	math::flt mVoltageGaspedal2;///0.. 5,12 V (doc: U2_Gaspedal)
	math::flt mVoltageMotor1;///0.. 5,12 V (doc: U1_Motor , Spannung am Eingang des Motorsteuergerätes)
	math::flt mVoltageMotor2;///0.. 5,12 V (doc: U2_Motor)
	bool mValid;
};

void operator>>(CanMessage  const & can_msg, ThrottleStatus  & throttleStatus);



/**
 *
 * Waehlhebel_Anforderung
 * CAN-Bus:	Regler-CAN
 * Identifier:	0x005 H
 * Length:	8 Byte
 * Cycletime:	10 ms //? doch 20ms
 * Timeout: 30ms
 */

class GearCommand
{
public:
	static boost::uint32_t const ID = 5;
	static boost::uint32_t const CYCLE_TIME = 10;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(GearCommand const & cmd, MsgType & msg);

	enum GearPosition {
		Position1	= 1,
		Position2	= 2,
		Position3	= 3,
		Position4	= 4,
		Drive		= 5,
		Neutral		= 6,
		Reverse		= 7,
		Park		= 8,
		NeutralSL	= 9, /// Neutral without Spotlight
		TipPlus		= 0xa,
		TipMinus	= 0xb,
		Sport		= 0xc,
		PositionL	= 0xd,
		Tiptronic	= 0xe,
		Error		= 0xf
	};


	/** convert legacy gear position (i.e. Spob) to passat gear position */
	static GearPosition legacyGearPosToPassatGearPos(int legacy) {
		switch (legacy) {
		case 1:
			return GearCommand::Park;

		case 2:
			return GearCommand::Reverse;

		default: // 0
		case 4:
			return GearCommand::Neutral;

		case 8:
			return GearCommand::Drive;

		case 16:
			return GearCommand::Sport;	//not for SpoB
		}

	}

	GearCommand(uint cycleCount, GearPosition position)
		: mCycleCount(cycleCount)
		, mGearPosition(position)
	{}

	/** set wanted gear position. */
	void setGearPosition(GearPosition g) {
		mGearPosition = g;
	}
protected:
	GearCommand()
		: mCycleCount(0)
		, mGearPosition(Error)
	{}


private:

	uint mCycleCount;
	GearPosition mGearPosition;
};

void operator>>(GearCommand const &, CanMessage const & msg);



/**
 *
 * AktorikBox_Funktionsanforderung
 * CAN-Bus:	Regler-CAN
 * Identifier:	A4 H
 * Länge:	8 Byte
 * Zykluszeit:	50 ms
 */
class FunctionCommand
{
public:
	static boost::uint32_t const ID = 0xa4;
	static boost::uint32_t const CYCLE_TIME = 50;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(FunctionCommand const & functionCommand, MsgType & can_msg);

	FunctionCommand(uint cycleCount)
		: mCycleCount(cycleCount)
		, mHorn(false)
		, mFlashLight(false)
		, mHighBeam(false)
		, mTurnSignalLeft(false)
		, mTurnSignalRight(false)
		, mWarningLights(false)
		, mDriverWindowUp(false)
		, mDriverWindowDown(false)
		, mCoDriverWindowUp(false)
		, mCoDriverWindowDown(false)
		, mWiperIntervalFront(false)
		, mWiperLevel1Front(false)
		, mWiperLevel2Front(false)
		, mWiperWaterFront(false)
		, mWiperBack(false)
		, mWiperWaterBack(false)
	{}

	/** Turn horn on or off. */
	void setHorn(bool h) {
		mHorn = h;
	}

	/** Turn flash light on or off. */
	void setFlashLight(bool h) {
		mFlashLight = h;
	}
	/** Turn High Beam on or off. */
	void setHighBeam(bool h) {
		mHighBeam = h;
	}
	/** Turn left turnsignal on or off. */
	void setTurnSignalLeft(bool h) {
		mTurnSignalLeft = h;
	}
	/** Turn right turnsignal on or off. */
	void setTurnSignalRight(bool h) {
		mTurnSignalRight = h;
	}
	/** Turn warning lights on or off. */
	void setWarningLights(bool h) {
		mWarningLights = h;
	}

	/** make driver window move up. */
	void setDriverWindowUp(bool h) {
		mDriverWindowUp = h;
	}
	/** make driver window move down. */
	void setDriverWindowDown(bool h) {
		mDriverWindowDown = h;
	}
	/** make Codriver window move up. */
	void setCoDriverWindowUp(bool h) {
		mCoDriverWindowUp = h;
	}
	/** make Codriver window move down. */
	void setCoDriverWindowDown(bool h) {
		mCoDriverWindowDown = h;
	}

	/** Turn front-wiper interval on or off. */
	void setWiperIntervalFront(bool h) {
		mWiperIntervalFront = h;
	}
	/** Set front-wiper interval to speedlevel 1. */
	void setWiperLevel1Front(bool h) {
		mWiperLevel1Front = h;
	}
	/** Set front-wiper interval to speedlevel 2. */
	void setWiperLevel2Front(bool h) {
		mWiperLevel2Front = h;
	}
	/** Turn front-wiper water on or off. */
	void setWiperWaterFront(bool h) {
		mWiperWaterFront = h;
	}
	/** Turn back-wiper on or off. */
	void setWiperBack(bool h) {
		mWiperBack = h;
	}
	/** Turn back-wiper water on or off. */
	void setWiperWaterBack(bool h) {
		mWiperWaterBack = h;
	}
protected:
	FunctionCommand()
		: mCycleCount(0)
		, mHorn(true)
	{}

private:
	boost::uint8_t mCycleCount;/// 0..15 (doc: Zykluszaehler)

	bool mHorn;//Signalhorn, false: off, true: on
	bool mFlashLight;//Lichthupe, false: off, true: on
	bool mHighBeam;//Fernlicht, false: off, true: on
	bool mTurnSignalLeft;//false: off, true: on
	bool mTurnSignalRight;//false: off, true: on
	bool mWarningLights;//Warnblinker, false: off, true: on
	bool mDriverWindowUp;//FensterFahrerHoch, false: off, true: on
	bool mDriverWindowDown;//FensterFahrerTief, false: off, true: on
	bool mCoDriverWindowUp;//FensterBeifahrerHoch, false: off, true: on
	bool mCoDriverWindowDown;//FensterBeifahrerTief, false: off, true: on
	bool mWiperIntervalFront;//WischerIntervall_vorne, false: off, true: on
	bool mWiperLevel1Front;//WischerStufe1_vorne, false: off, true: on
	bool mWiperLevel2Front;//WischerStufe2_vorne, false: off, true: on
	bool mWiperWaterFront;//WischWasch_vorne, false: off, true: on
	bool mWiperBack;//Wischer_hinten, false: off, true: on
	bool mWiperWaterBack;//WischWasch_hinten, false: off, true: on
};

void operator>>(FunctionCommand const & functionCommand, CanMessage  & can_msg);


class GearStatus
{
public:
	static boost::uint32_t const ID = 1096;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, GearStatus & gearStatus);

	enum ChosenPos {
		ErrorPos = 15,
		TiptronicManual = 14,
		Pos_L = 13,
		Pos_S_Automatic_Sport = 12,
		Pos_Z2 = 11,
		Pos_Z1 = 10,
		Pos_RSP_Manual_Sport = 9,
		Pos_P_Key_Lock_Release = 8,
		Pos_R = 7,
		Pos_N = 6,
		Pos_D_Automatic = 5,
		Pos_4 = 4,
		Pos_3 = 3,
		Pos_2 = 2,
		Pos_1 = 1,
		BetweenPos = 0
	};

	/** convert legacy gear position (i.e. Spob) to passat gear status */
	static ChosenPos legacyGearPosToPassatChosenPos(int legacy) {
		switch (legacy) {
		case 1:
			return Pos_P_Key_Lock_Release;

		case 2:
			return Pos_R;

		default: // 0
		case 4:
			return Pos_N;

		case 8:
			return Pos_D_Automatic;

		case 16:
			return Pos_S_Automatic_Sport;
		}
	}

	GearStatus()
		: mValid(false)
	{}

	GearStatus(ChosenPos pos)
		: mValid(false) {
		mChosenPos = pos;
	}

	static char const * const chosenPosString(ChosenPos pos) {
		switch (pos) {
		case BetweenPos:
			return "BetweenPos";

		case Pos_1:
			return "Pos_1";

		case Pos_2:
			return "Pos_2";

		case Pos_3:
			return "Pos_3";

		case Pos_4:
			return "Pos_4";

		case Pos_D_Automatic:
			return "Pos_D_Automatic";

		case Pos_N:
			return "Pos_N";

		case Pos_R:
			return "Pos_R";

		case Pos_P_Key_Lock_Release:
			return "Pos_P_Key_Lock_Release";

		case Pos_RSP_Manual_Sport:
			return "Pos_RSP_Manual_Sport";

		case Pos_Z1:
			return "Pos_Z1";

		case Pos_Z2:
			return "Pos_Z2";

		case Pos_S_Automatic_Sport:
			return "Pos_S_Automatic_Sport";

		case Pos_L:
			return "Pos_L";

		case TiptronicManual:
			return "TiptronicManual";

		case ErrorPos:
			return "ErrorPos";

		default:
			return "-";
		}
	}


	bool valid() const {
		return mValid;
	}

	uint cycleCounter() const {
		return mCycleCounter;
	}

	ChosenPos chosenPos() const {
		return mChosenPos;
	}

	std::string toString() const;

private:
	uint mCycleCounter;
	ChosenPos mChosenPos;
	bool mValid;

};

void operator>>(CanMessage const & can_msg, GearStatus & gearStatus);


/**
 *
 * AktorikBox_Parkhilfe
 * CAN-Bus:	Sensor-CAN
 * Identifier:	1355
 * Länge:	8 Byte
 * Zykluszeit:	??
 * Timeout: ??
 */
class ParkAssistData
{
public:
	static boost::uint32_t const ID = 1355;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, ParkAssistData & parkAssistData);

	ParkAssistData()
	{}

	enum SensorPosition {
		FrontLeft = 0,
		FrontRight = 1,
		RearLeft = 2,
		RearRight = 3,
		FrontMiddleLeft = 4,
		FrontMiddleRight = 5,
		RearMiddleLeft = 6,
		RearMiddleRight = 7
	};

	uint messageCounter() const {
		return mMessageCounter;
	}

	boost::uint8_t mData[8];	//min: 0, max: 254 cm

	std::string toString() const;

private:
	boost::uint8_t mMessageCounter;/// 0..15 (doc: Zykluszaehler)
};

void operator>>(CanMessage const & can_msg, ParkAssistData & parkAssistData);


/**
 *
 * SteerAssist3 (Lenkhilfe 3)
 * CAN-Bus:	Sensor-CAN
 * Identifier:	208
 * Länge:	6 Byte
 * Zykluszeit:	10 ms
 */

class SteerAssist3Status
{
public:
	static boost::uint32_t const ID = 208;


	static math::flt const MAXSTEERINGWHEELANGLE;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, SteerAssist3Status & steerAssist3Status);

	SteerAssist3Status()
		: mValid(false)
	{}

	SteerAssist3Status(math::flt steerAngle, math::flt steerMomentum) {
		mSteerAngle = steerAngle;
		mSteerMomentum = steerMomentum;
	}

	bool valid() const {
		return mValid;
	}

	boost::uint8_t messageCounter() const {
		return mMessageCounter;
	}

	bool steerMomentumValid() const {
		return mSteerMomentumValid;
	}

	math::flt steerMomentum() const {
		return mSteerMomentum;
	}

	bool steerAngleValid() const {
		return mSteerAngleValid;
	}

	math::flt steerAngle() const {
		return mSteerAngle;
	}

	math::flt normalizedSteerAngle() const {
		return (-1.0) * mSteerAngle / MAXSTEERINGWHEELANGLE;
	}

	std::string toString() const;
private:
	bool mValid;						// Checksum
	boost::uint8_t mMessageCounter;		// LH3_Zaehler

	math::flt mSteerMomentum; 				// LH3_LM, Lenkmoment, Faktor 0.147; min: 0, max: 15.0381 Nm
	bool mSteerMomentumValid;			// LH3_BLWValid, true: 1, false: 0

	math::flt mSteerAngle; 				// LH3_BLW Lenkwinkel, Faktor 0.15 min: 0; max: 614.25 Grad
	bool mSteerAngleValid;			// LH3_LMValid, true: 1, false: 0
	// 538 deg (left) and -535 deg (right)
};

/**
 * Operator to init a SteerAssist3Status from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, SteerAssist3Status  & steerAssist3Status);



/**
 *
 * SignalWipersStatus (LSM_1)  // Status für Blinker, Wischer und Hupe
 * CAN-Bus:	Sensor CAN
 * Identifier:	705
 * Länge:	6 Byte
 * Zykluszeit:	?? ms
 */

class SignalWipersStatus
{
public:
	static boost::uint32_t const ID = 705;
	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType  const & can_msg, SignalWipersStatus & signalWipersStatus);

	SignalWipersStatus()
		: mValid(false)
	{}

	enum TurnSignal {
		OFF = 0,
		LEFT = 1,
		RIGHT = 2,
		HAZARDS = 3
	};


	bool valid() const {
		return mValid;
	}

	bool turnLeft() const {
		return mTurnLeft;
	}

	bool turnRight() const {
		return mTurnRight;
	}

	bool flashLight() const {
		return mFlashLight;
	}

	bool highBeam() const {
		return mHighBeam;
	}

	bool horn() const {
		return mHorn;
	}

	bool wipeTip() const {
		return mWipeTip;
	}

	bool wipeInterval() const {
		return mWipeInterval;
	}

	bool wipeLevel1() const {
		return mWipeLevel1;
	}

	bool wipeLevel2() const {
		return mWipeLevel2;
	}

	bool washFront() const {
		return mWashFront;
	}

	bool washFrontMov() const {
		return mWashFrontMov;
	}

	bool wipeRearInterval() const {
		return mWipeRearInterval;
	}

	bool washRear() const {
		return mWashRear;
	}

	boost::uint8_t intervalLevel() const {
		return mIntervalLevel;
	}

	std::string toString() const;
private:
	bool mValid;						// Checksum

	bool mTurnLeft;			//Blinker links
	bool mTurnRight;		//Blinker rechts
	bool mFlashLight;		//Lichthupe
	bool mHighBeam;			//Fernlicht
	bool mHorn;				//Signalhorn
	bool mWipeTip;			//Tipwischen
	bool mWipeInterval;		//Interval
	bool mWipeLevel1;		//Wischen Stufe 1
	bool mWipeLevel2;		//Wischen Stufe 2
	bool mWashFront;		//Frontwaschen
	bool mWashFrontMov;		//Bew Frontwaschen
	bool mWipeRearInterval;	//Heckinterval
	bool mWashRear;			//Heckwaschen
	boost::uint8_t mIntervalLevel;	//Intervalstufen
};

/**
 * Operator to init a SignalWipersStatus from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, SignalWipersStatus  & signalWipersStatus);



/**
 *
 * Light1Status (Licht_1_alt)  // Status für Licht außer Blinker
 * CAN-Bus:	Sensor-CAN
 * Identifier:	1329
 * Länge:	4 Byte
 * Zykluszeit:	?? ms
 */

class Light1Status
{
public:
	static boost::uint32_t const ID = 1329;
	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, Light1Status & light1Status);

	Light1Status()
		: mValid(false)
	{}

	bool valid() const {
		return mValid;
	}

	bool parkingLight() const {
		return mParkingLight;
	}

	bool lowBeam() const {
		return mLowBeam;
	}

	bool warningLights() const {
		return mWarningLights;
	}

	uint cycleCounter() const {
		return mCycleCounter;
	}


	std::string toString() const;
private:
	bool mValid;						// Checksum

	bool mParkingLight;					// Standlicht
	bool mLowBeam;						// Abblendlicht
	bool mWarningLights;				// Warnblink
	uint mCycleCounter;					// Zähler

};

/**
 * Operator to init a SteerAssist3Status from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, Light1Status & light1Status);


/**
 *
 * Motor1 (Motor1)
 * CAN-Bus:	Sensor-CAN
 * Identifier:	640
 * Länge:	8 Byte
 * Zykluszeit:	?? ms
 */

class Motor1Status
{
public:
	static boost::uint32_t const ID = 640;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, Motor1Status & motor1Status);

	Motor1Status()
		: mValid(false)
		, mPedalValue(false)
	{}

	bool pedalStatus() const {
		return mPedalStatus;
	}

	bool kickDown() const {
		return mKickDown;
	}

	math::flt pedalValue() const {
		return mPedalValue;
	}

	math::flt revolutionSpeed() const {
		return mRevolutionSpeed;
	}

	bool valid() const {
		return mValid;
	}

	std::string toString() const;

private:
	bool mValid;
	bool mPedalStatus;				// MO1_Sta_Pedal
	bool mKickDown;					// MO1_Kickdown
	math::flt mPedalValue;				// MO1_Sta_Pedal
	math::flt mRevolutionSpeed;			// MO1_Drehzahl

};

/**
 * Operator to init a Motor1Status from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, Motor1Status  & motor1Status);


/*------------------------------------------------------------------------------------------------------------------*/

/**
 *
 * GraMainSwitch (mGRA_Neu)
 * CAN-Bus:     Sensor-CAN
 * Identifier:	906
 * Länge:       4 Byte
 * Zykluszeit:  ?? ms
 */

class GraMainSwitch
{
public:
	static boost::uint32_t const ID = 906;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, GraMainSwitch & graMainSwitch);

	GraMainSwitch()
		: mValid(false)
	{}

	enum TimeGap {
		ButtonNotPressed = 0, // Taste nicht betätigt
		DIST_minus_1     = 1, // ?
		DIST_plus_1      = 2, // ?
		Unassigned       = 3  // nicht belegt
	};

	static char const * const timeGapString(TimeGap tg) {
		switch (tg) {
		case ButtonNotPressed:
			return "ButtonNotPressed";

		case DIST_minus_1:
			return "DIST_minus_1";

		case DIST_plus_1:
			return "DIST_plus_1";

		case Unassigned:
			return "Unassigned";

		default:
			return "-";
		}
	}

	bool valid() const {				// Checksumme
		return mValid;
	}

	uint newCounter() const {			// GRA_Neu_Zaehler (4 bit)
		return mNewCounter;
	}

	TimeGap timegap() const {
		return TimeGap(mTimegap);    // GRA_Zeitluecke (2 bit)
	}


	bool mainSwitchSign() const {
		return mMainSwitchSign;    // GRA_Hauptschalt
	}

	bool cancel() const {
		return mCancel;    // GRA_Abbrechen
	}

	bool downShort() const {
		return mDownShort;    // GRA_Down_kurz
	}

	bool downLong() const {
		return mDownLong;    // GRA_Down_lang
	}

	bool upShort() const {
		return mUpShort;    // GRA_Up_kurz
	}

	bool upLong() const {
		return mUpLong;    // GRA_Up_lang
	}

	bool tipDown() const {
		return mTipDown;    // GRA_Tip_Down
	}

	bool tipUp() const {
		return mTipUp;    // GRA_Tip_Up
	}

	bool reset() const {
		return mReset;    // GRA_Neu_Setzen
	}

	bool recall() const {
		return mRecall;    // GRA_Recall
	}

	bool type() const {
		return mType;    // GRA_Typ_Hauptschalt
	}


	std::string toString() const;

private:
	bool mValid;						// Checksumme (8 bit)

	uint mNewCounter;					// GRA_Neu_Zaehler (4 bit)

	uint mTimegap;						// GRA_Zeitluecke (2 bit)

	bool mMainSwitchSign;				// GRA_Hauptschalt
	bool mCancel;						// GRA_Abbrechen
	bool mDownShort;					// GRA_Down_kurz
	bool mDownLong;						// GRA_Down_lang
	bool mUpShort;						// GRA_Up_kurz
	bool mUpLong;						// GRA_Up_lang
	bool mTipDown;						// GRA_Tip_Down
	bool mTipUp;						// GRA_Tip_Up
	bool mReset;				        // GRA_Neu_Setzen
	bool mRecall;						// GRA_Recall
	bool mType;							// GRA_Typ_Hauptschalt

};

/**
 * Operator to init a GraMainSwitch from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, GraMainSwitch  & graMainSwitch);


/**
 *
 * Airbag (mAirbag_1)
 * CAN-Bus:     Sensor-CAN
 * Identifier:	80
 * Länge:       4 Byte
 * Zykluszeit:  ?? ms
 */

class Airbag
{
public:
	static boost::uint32_t const ID = 80;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, Airbag & airbag);

	Airbag()
		: mValid(false)
	{}

	enum SeatBelt {
		Unavailable = 0,
		Error       = 1,
		NotFastened = 2,
		Fastened    = 3
	};

	static char const * const seatBeltString(SeatBelt sb) {
		switch (sb) {
		case Unavailable:
			return "Unavailable";

		case Error:
			return "Error";

		case NotFastened:
			return "NotFastened";

		case Fastened:
			return "Fastened";

		default:
			return "-";
		}
	}

	bool valid() const {				// Checksumme
		return mValid;
	}

	SeatBelt seatbelt() const {	// AB1_Gurt_Fa
		return SeatBelt(mSeatbelt);
	}

	uint counter() const {	// AB1_Zaehler
		return mCounter;
	}

	std::string toString() const;

private:
	bool mValid;						// Checksumme (8 bit)

	uint mSeatbelt;			// AB1_Gurt_Fa (2 bit)

	uint mCounter;			// AB1_Zaehler (4 bit)

};

/**
 * Operator to init a Airbag from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, Airbag  & airbag);



/**
 *
 * AbsEsp (mBremse_1) //....
 * CAN-Bus:     Sensor-CAN
 * Identifier:	416
 * Länge:       8 Byte
 * Zykluszeit:  ?? ms
 */

class AbsEsp
{
public:
	static boost::uint32_t const ID = 416;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, AbsEsp & absEsp);

	AbsEsp()
		: mValid(false)
	{}

	bool valid() const {				// Checksumme
		return mValid;
	}

	boost::uint8_t counter() const {	// BR1_Zaehler
		return mCounter;
	}

	bool espSign() const {			// BR1_ESP_Eingr
		return mEspSign;
	}

	bool absSign() const {
		return mAbsSign;    // BR1_ABS_Brems
	}

	bool backupSign() const {
		return mBackupSign;    // BR1_Ersatz_Kmh
	}

	math::flt speed() const {
		return mSpeed;    // BR1_Rad_kmh
	}

	std::string toString() const;

private:
	bool mValid;						// Checksumme (8 bit)

	boost::uint8_t mCounter;			// BR1_Zaehler (4 bit)

	bool mEspSign;						// BR1_ESP_Eingr
	bool mAbsSign;						// BR1_ABS_Brems
	bool mBackupSign;					// BR1_Ersatz_Kmh

	math::flt mSpeed;					// BR1_Rad_kmh
};

/**
 * Operator to init a AbsEsp from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, AbsEsp  & absEsp);
/*------------------------------------------------------------------------------------------------------------------*/


/**
 *
 * AbsEspFake (mBremse_1) //Fake for Hella Sensors....
 * CAN-Bus:     Sensor-CAN
 * Identifier:	417
 * Länge:       8 Byte
 * Zykluszeit:  10 ms
 */

class AbsEspFake
{
public:
	static boost::uint32_t const ID = 417;
	static boost::uint32_t const CYCLE_TIME = 10;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(AbsEspFake const & cmd, MsgType & msg);

	AbsEspFake()
		: mValid(false)
	{}

	void valid(bool c)  {				// Checksumme
		mValid = c;
	}

	void  setCounter(boost::uint8_t c)  {	// BR1_Zaehler
		mCounter = c;
	}

	void espSign(bool c)  {			// BR1_ESP_Eingr
		mEspSign = c;
	}

	void absSign(bool c) {
		mAbsSign = c;    // BR1_ABS_Brems
	}

	void backupSign(bool c)  {
		mBackupSign = c;    // BR1_Ersatz_Kmh
	}

	void setSpeed(math::flt c) {
		mSpeed = c;    // BR1_Rad_kmh
	}

private:
	bool mValid;						// Checksumme (8 bit)

	boost::uint8_t mCounter;
	bool mEspSign;						// BR1_ESP_Eingr
	bool mAbsSign;						// BR1_ABS_Brems
	bool mBackupSign;					// BR1_Ersatz_Kmh

	math::flt mSpeed;					// BR1_Rad_kmh
};

/**
 * Operator to init a AbsFake from a msg_data_ind.
 */
void operator>>(AbsEspFake const &, CanMessage const & msg);


/*------------------------------------------------------------------------------------------------------------------*/



/**
 *
 * PathPulse (mBremse_10) //Tacho- oder Wegimpuls und dazugehörige Qualitätsbits
 * CAN-Bus:	Sensor-CAN
 * Identifier:	928
 * Länge:	8 Byte
 * Zykluszeit:	?? ms
 */

class PathPulse
{
public:
	static boost::uint32_t const ID = 928;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, PathPulse  & pathPulse);

	PathPulse()
		: mValid(false)
	{}

	bool valid() const {			// B10_Checksumme
		return mValid;
	}

	boost::uint8_t counter() const {	// B10_Zaehler
		return mCounter;
	}

	bool qBitFrontL() const {		// B10_QB_Wegimp_VL
		return mQBitFrontL;
	}

	bool qBitFrontR() const {		// B10_QB_Wegimp_VR
		return mQBitFrontR;
	}

	bool qBitRearL() const {		// B10_QB_Wegimp_HL
		return mQBitRearL;
	}

	bool qBitRearR() const {		// B10_QB_Wegimp_HR
		return mQBitRearR;
	}


	uint16_t pulseFrontL() const {	// B10_Wegimp_VL
		return mPulseFrontL;
	}

	uint16_t pulseFrontR() const {	// B10_Wegimp_VR
		return mPulseFrontR;
	}

	uint16_t pulseRearL() const {	// B10_Wegimp_HL
		return mPulseRearL;
	}

	uint16_t pulseRearR() const {	// B10_Wegimp_HR
		return mPulseRearR;
	}

	std::string toString() const;

private:
	bool mValid;					// B10_Checksumme (8 bit)

	boost::uint8_t mCounter;		// B10_Zaehler (4 bit)

	bool mQBitFrontL;				// B10_QB_Wegimp_VL
	bool mQBitFrontR;				// B10_QB_Wegimp_VR
	bool mQBitRearL;				// B10_QB_Wegimp_HL
	bool mQBitRearR;				// B10_QB_Wegimp_HR

	uint16_t mPulseFrontL: 10; 			// B10_Wegimp_VL
	uint16_t mPulseFrontR: 10; 			// B10_Wegimp_VR
	uint16_t mPulseRearL: 10; 			// B10_Wegimp_HL
	uint16_t mPulseRearR: 10; 			// B10_Wegimp_HR

};

/**
 * Operator to init a PathPulse from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, PathPulse  & pathPulse);


/**
 *
 * WheelSpeeds (mBremse3)
 * CAN-Bus:	Sensor-CAN
 * Identifier:	1184
 * Länge:	8 Byte
 * Zykluszeit:	?? ms
 * in Meters per Second
 */

class WheelSpeeds
{
public:
	static boost::uint32_t const ID = 1184;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, WheelSpeeds & wheelSpeeds);

	WheelSpeeds()
		: mValid(false)
	{}

	WheelSpeeds(math::flt speedRearL, math::flt speedRearR, math::flt speedFrontL, math::flt speedFrontR)
		: mValid(true) {
		mSpeedRearL = speedRearL;
		mSpeedRearR = speedRearR;
		mSpeedFrontL = speedFrontL;
		mSpeedFrontR = speedFrontR;
	}

	~WheelSpeeds()
	{}

	bool signFrontL() const {
		return mSignFrontL;
	}

	bool signFrontR() const {
		return mSignFrontR;
	}

	bool signRearL() const {
		return mSignRearL;
	}

	bool signRearR() const {
		return mSignRearR;
	}

	math::flt speedFrontL() const {
		return mSpeedFrontL;
	}

	math::flt speedFrontR() const {
		return mSpeedFrontR;
	}

	math::flt speedRearL() const {
		return mSpeedRearL;
	}

	math::flt speedRearR() const {
		return mSpeedRearR;
	}

	math::flt speedAvg() const {
		return (mSpeedFrontL + mSpeedFrontR + mSpeedRearL + mSpeedRearR) * 0.25;
	}

	math::flt speedFrontAvg() const {
		return (mSpeedFrontL + mSpeedFrontR) * 0.5;
	}

	math::flt tangentialAccelerationFront() const {
		return 0.0;
	}

	math::flt tangentialAccelerationRear() const {
		return 0.0;
	}

	math::flt instantaniousCenterOfCurvatureFromRearWheelSpeed() const {	//Distance to the rear axis centre
		if ((mSpeedRearL == 0) || (mSpeedRearR == 0) ||
				((mSpeedRearL / mSpeedRearR > 0.99999)  && (mSpeedRearR / mSpeedRearL > 0.99999))) {
			return 1000000.0;
		}
		else {
			return 1.551 / (mSpeedRearL / mSpeedRearR - 1) + 1.551 / 2;
		}
	}

	math::flt instantaniousCenterOfCurvatureFromFrontWheelSpeed() const {	//Distance to the front axis centre, worse than rear wheel ICC
		if ((mSpeedFrontL == 0) || (mSpeedFrontR == 0) ||
				((mSpeedFrontL / mSpeedFrontR > 0.99999)  && (mSpeedFrontR / mSpeedFrontL > 0.99999))) {
			return 1000000.0;
		}
		else {
			math::flt const vL_Div_vR_squared = (mSpeedFrontL / mSpeedFrontR) * (mSpeedFrontL / mSpeedFrontR);
			math::flt const p = -2 * 1.551 / (vL_Div_vR_squared - 1);
			math::flt const q = 2.709 * 2.709 - 1.551 * 1.551 / ((vL_Div_vR_squared - 1));

			if (p * p / 4 - q < 0) {
				return (-p / 2 + 1.551 / 2);		// Hack, square root term should never become zero
			}
			else if (fabs(mSpeedFrontL) >= fabs(mSpeedFrontR)) {
				return (-p / 2 + sqrt(p * p / 4 - q) + 1.551 / 2);    // what contains the second solution?
			}
			else {
				return (-p / 2 - sqrt(p * p / 4 - q) + 1.551 / 2);
			}
		}
	}

	math::flt radiusRearAxisCenter() const {
		return instantaniousCenterOfCurvatureFromRearWheelSpeed();
	}

	math::flt radiusFrontAxisCenter() const {
		return sqrt(radiusRearAxisCenter() * radiusRearAxisCenter() + 2.709 * 2.709);
	}

	math::flt speedRearAxis() const {
		return (mSpeedRearL + mSpeedRearR) * 0.5;
	}

	math::flt speedFrontAxis() const {		//calculated from rear axis speed
		return fabs(radiusFrontAxisCenter() * speedRearAxis() / radiusRearAxisCenter());
	}

	math::flt absCentrifugalAccelerationRear() const {
		if (instantaniousCenterOfCurvatureFromRearWheelSpeed() == 0) {
			return -1;
		}
		else {
			return fabs(speedRearAxis() * speedRearAxis() / radiusRearAxisCenter());
		}
	}

	math::flt absCentrifugalAccelerationFront() const {			//calculated by rear icc - more accurate
		if (instantaniousCenterOfCurvatureFromRearWheelSpeed() == 0) {
			return -1;
		}
		else {
			return fabs(speedFrontAxis() * speedFrontAxis() / radiusFrontAxisCenter());
		}
	}

	bool allWheelsStopped() const {
		return ((mSpeedFrontL == 0.0) && (mSpeedFrontR == 0.0) && (mSpeedRearL == 0.0) && (mSpeedRearR == 0.0));
	}

	std::string toString() const;

	bool valid() const {
		return mValid;
	}

private:
	bool mSignFrontL;				// sign Wheelspeed FrontL
	bool mSignFrontR;				// sign Wheelspeed FrontR
	bool mSignRearL;				// sign Wheelspeed RearL
	bool mSignRearR;				// sign Wheelspeed RearR

	math::flt mSpeedFrontL;			// Wheelspeed FrontL
	math::flt mSpeedFrontR;			// Wheelspeed FrontR
	math::flt mSpeedRearL;				// Wheelspeed RearL
	math::flt mSpeedRearR;				// Wheelspeed RearR
	bool mValid;
};

/**
 * Operator to init a WheelSpeeds from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, WheelSpeeds  & wheelSpeeds);



/**
 *
 * WheelSpeedsFake (mBremse3)
 * CAN-Bus:	Sensor-CAN
 * Identifier:	1185
 * Länge:	8 Byte
 * Zykluszeit:	?? ms
 * in Meters per Second
 */

class WheelSpeedsFake
{
public:
	static boost::uint32_t const ID = 1184;
	static boost::uint32_t const CYCLE_TIME = 10;
	template<typename MsgType> friend void aa::modules::io::passat::operator>>(WheelSpeedsFake const & cmd, MsgType & msg);

	WheelSpeedsFake()
		: mValid(false)
	{}

	WheelSpeedsFake(math::flt speedRearL, math::flt speedRearR, math::flt speedFrontL, math::flt speedFrontR)
		: mValid(true) {
		mSpeedRearL = speedRearL;
		mSpeedRearR = speedRearR;
		mSpeedFrontL = speedFrontL;
		mSpeedFrontR = speedFrontR;
	}

	~WheelSpeedsFake()
	{}

	void setSignFrontL(bool g) {
		mSignFrontL = g;
	}

	void setSignFrontR(bool g) {
		mSignFrontR = g;
	}

	void setSignRearL(bool g) {
		mSignRearL = g;
	}

	void setSignRearR(bool g) {
		mSignRearR = g;
	}

	void setSpeedFrontL(math::flt g) {
		mSpeedFrontL = g;
	}

	void setSpeedFrontR(math::flt g) {
		mSpeedFrontR = g;
	}

	void setSpeedRearL(math::flt g) {
		mSpeedRearL = g;
	}

	void setSpeedRearR(math::flt g) {
		mSpeedRearR = g;
	}

	void setMessageCounter(uint m) {
		mMessageCounter = m;
	}

	bool valid() const {
		return mValid;
	}

private:
	boost::uint8_t mMessageCounter;

	bool mSignFrontL;				// sign Wheelspeed FrontL
	bool mSignFrontR;				// sign Wheelspeed FrontR
	bool mSignRearL;				// sign Wheelspeed RearL
	bool mSignRearR;				// sign Wheelspeed RearR

	math::flt mSpeedFrontL;			// Wheelspeed FrontL
	math::flt mSpeedFrontR;			// Wheelspeed FrontR
	math::flt mSpeedRearL;				// Wheelspeed RearL
	math::flt mSpeedRearR;				// Wheelspeed RearR
	bool mValid;

};

/**
 * Operator to init a WheelSpeedsFake from a msg_data_ind.
 */
void operator>>(WheelSpeedsFake const &, CanMessage const & msg);


/**
 *
 * SteeringWheelSpeed (mLW1)  accurate wheelangle, variable offset
 * CAN-Bus:	Sensor-CAN
 * Identifier:	194
 * Länge:	8 Byte
 * Zykluszeit:	?? ms
 */

class SteeringWheelSpeed
{
public:
	static boost::uint32_t const ID = 194;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, SteeringWheelSpeed & steeringWheelSpeed);

	SteeringWheelSpeed()
		: mValid(false)
	{}

	~SteeringWheelSpeed()
	{}


	bool valid() {
		return mValid;
	}

	math::flt steeringWheelAngle() const {
		return mSteeringWheelAngle;
	}

	bool steeringWheelAngleSign() const {
		return mSteeringWheelAngleSign;
	}

	math::flt steeringWheelSpeed() const {
		return mSteeringWheelSpeed;
	}

	bool steeringWheelSpeedSign() const {
		return mSteeringWheelSpeedSign;
	}

	boost::uint8_t messageId() const {
		return id;
	}

	boost::uint8_t counter() const {
		return mCounter;
	}


	std::string toString() const;

private:
	bool mValid;

	math::flt mSteeringWheelAngle;  //min max: -/+ 1433.55625 deg, accuracy: 0.04375 deg
	bool mSteeringWheelAngleSign;		// -1 or +1
	math::flt mSteeringWheelSpeed;  //min max: -/+ 1433.55625 deg/s, accuracy: 0.04375 deg/s
	bool mSteeringWheelSpeedSign;// -1 or +1
	boost::uint8_t id;			//id
	boost::uint8_t mCounter;	//counter (4 bit)
};

/**
 * Operator to init a WheelSpeeds from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, SteeringWheelSpeed  & steeringWheelSpeed);


/*********************************************/

/**
 *
 * TargetGear (mGetriebe_1)
 * CAN-Bus:     Sensor-CAN
 * Identifier:	1088
 * Länge:       8 Byte
 * Zykluszeit:  ?? ms
 */

class TargetGear
{
public:
	static boost::uint32_t const ID = 1088;
	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, TargetGear & targetGear);


	TargetGear()
		: mValid(false)
	{}

	~TargetGear()
	{}

	enum TargetPos {
		ErrorTarget = 15,
		CurrentGearNotDefined = 14,
		Free3 = 13,
		Free2 = 12,
		Free1 = 11,
		Free0 = 10,
		Gear7 = 9,
		Gear6 = 8,
		GearR = 7,
		Gear_1m = 6,
		Gear5 = 5,
		Gear4 = 4,
		Gear3 = 3,
		Gear2 = 2,
		Gear1 = 1,
		Gear_P_N_None = 0
	};

	static char const * const targetPosString(TargetPos target) {
		switch (target) {
		case Gear_P_N_None:
			return "Gear_P_N_None";

		case Gear1:
			return "Gear1";

		case Gear2:
			return "Gear2";

		case Gear3:
			return "Gear3";

		case Gear4:
			return "Gear4";

		case Gear5:
			return "Gear5";

		case Gear_1m:
			return "Gear_1m";

		case GearR:
			return "GearR";

		case Gear6:
			return "Gear6";

		case Gear7:
			return "Gear7";

		case Free0:
			return "Free0";

		case Free1:
			return "Free1";

		case Free2:
			return "Free2";

		case Free3:
			return "Free3";

		case CurrentGearNotDefined:
			return "CurrentGearNotDefined";

		case ErrorTarget:
			return "ErrorTarget";

		default:
			return "-";
		}
	}

	enum SelectedPos {
		ErrorSelected = 15,
		TiptronicManual = 14,
		Pos_L = 13,
		Pos_S_Automatic_Sport = 12,
		Pos_Z2 = 11,
		Pos_Z1 = 10,
		Pos_RSP_Manual_Sport = 9,
		Pos_P_Key_Lock_Release = 8,
		Pos_R = 7,
		Pos_N = 6,
		Pos_D_Automatic = 5,
		Pos_4 = 4,
		Pos_3 = 3,
		Pos_2 = 2,
		Pos_1 = 1,
		BetweenPos = 0
	};

	static char const * const selectedPosString(SelectedPos selected) {
		switch (selected) {
		case BetweenPos:
			return "BetweenPos";

		case Pos_1:
			return "Pos_1";

		case Pos_2:
			return "Pos_2";

		case Pos_3:
			return "Pos_3";

		case Pos_4:
			return "Pos_4";

		case Pos_D_Automatic:
			return "Pos_D_Automatic";

		case Pos_N:
			return "Pos_N";

		case Pos_R:
			return "Pos_R";

		case Pos_P_Key_Lock_Release:
			return "Pos_P_Key_Lock_Release";

		case Pos_RSP_Manual_Sport:
			return "Pos_RSP_Manual_Sport";

		case Pos_Z1:
			return "Pos_Z1";

		case Pos_Z2:
			return "Pos_Z2";

		case Pos_S_Automatic_Sport:
			return "Pos_S_Automatic_Sport";

		case Pos_L:
			return "Pos_L";

		case TiptronicManual:
			return "TiptronicManual";

		case ErrorSelected:
			return "ErrorSelected";

		default:
			return "-";
		}
	}


	bool valid() const {				// Checksumme
		return mValid;
	}

	TargetPos targetPos() const {	// Zielgang
		return mTargetPos;
	}

	SelectedPos selectedPos() const { // Ausgewählter Gang
		return mSelectedPos;
	}

	uint counter() const {			// Zähler
		return mCounter;
	}

	std::string toString() const;

private:
	bool mValid;						// Checksumme (8 bit)
	uint mCounter;						// GE1_Zaehler (4 bit)

	TargetPos mTargetPos;				// GE1_Zielgang
	SelectedPos mSelectedPos;			// GE1_Wahl_Pos

};

/**
 * Operator to init a TargetGear from a msg_data_ind.
 */
void operator>>(CanMessage const & can_msg, TargetGear & targetGear);


/**
 *
 * Acceleration  (mBremse_8)
 * CAN-Bus:     Sensor-CAN
 * Identifier:	428
 * Länge:       8 Byte
 * Zykluszeit:  ?? ms
 */

class Acceleration
{
public:
	static boost::uint32_t const ID = 428;
	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, Acceleration & acceleration);

	Acceleration()
		: mValid(false)
	{}

	~Acceleration()
	{}

	bool valid() const {						// Checksumme
		return mValid;
	}


	uint counter() const {					// Zaehler
		return mCounter;
	}

	math::flt  logitudinalAcc() const {		// BR8_Laengsbeschl
		return mLogitudinalAcc;
	}

	math::flt currentAcc() const {			// BR8_Istbeschl
		return mCurrentAcc;
	}

	bool accSign() const {				// BR8_QB_LBeschl
		return mAccSign;
	}

	std::string toString() const;

private:
	bool mValid;						// Checksumme (8 bit)

	uint mCounter;						// Zähler

	math::flt mLogitudinalAcc;				// BR8_Laengsbeschl
	math::flt mCurrentAcc;					// BR8_Istbeschl

	bool mAccSign;						// BR8_QB_LBeschl




};

/**
 * Operator to init a Acceleration from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, Acceleration  & acceleration);


/**
 *
 * YawAndBrakePressure  (mBremse_5)
 * CAN-Bus:     Sensor-CAN
 * Identifier:	1192
 * Länge:       8 Byte
 * Zykluszeit:  ?? ms
 */

class YawAndBrakePressure
{
public:
	static boost::uint32_t const ID = 1192;

	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, YawAndBrakePressure & yawAndBrakePressure);

	YawAndBrakePressure()
		: mValid(false)
	{}

	~YawAndBrakePressure()
	{}

	bool valid() const {						// Checksumme
		return mValid;
	}


	uint counter() const {					// Zaehler
		return mCounter;
	}


	math::flt yawRate() const {
		return mYawRate;    // BR5_Giergeschw
	}

	math::flt brakePressure() const {
		return mBrakePressure;    // BR5_Bremsdruck
	}


	bool yawRateValid() const {
		return mYawRateValid;    // BR5_Sta_Gierrate
	}

	bool brakePressureValid() const {
		return mPressureValid;    // BR5_Druckgueltig
	}

	std::string toString() const;

private:
	bool mValid;						// Checksumme (8 bit)

	uint mCounter;						// Zähler

	math::flt mYawRate;				// BR5_Giergeschw
	math::flt mBrakePressure;				// BR5_Bremsdruck

	bool mPressureValid;				// BR5_Druckgueltig
	bool mYawRateValid;				// BR5_Sta_Gierrate


};

/**
 * Operator to init a YawAndBrakePressure from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, YawAndBrakePressure  & yawAndBrakePressure);


/**
 *
 * AccelerationAndTeeth  (mBremse_2)
 * CAN-Bus:     Sensor-CAN
 * Identifier:	1440
 * Länge:       8 Byte
 * Zykluszeit:  ?? ms
 */

class AccelerationAndTeeth
{
public:
	static boost::uint32_t const ID = 1440;
	template<typename MsgType> friend void aa::modules::io::passat::operator>>(MsgType const & can_msg, AccelerationAndTeeth & accelerationAndTeeth);

	AccelerationAndTeeth()
		: mValid(false)
	{}

	~AccelerationAndTeeth()
	{}

	bool valid() const {						// Checksumme
		return mValid;
	}


	boost::uint8_t counter() const {			// Zaehler
		return mCounter;
	}


	boost::uint8_t nrOfTeeth() const {
		return mNrOfTeeth;    // BR2_Zahnzahl
	}

	math::flt lateralAcceleration() const {
		return mLateralAcceleration;    // BR2_Querbeschl
	}


	bool mux() const {
		return mMux;    // BR2_Querb_Zahn
	}

	bool lateralAccValid() const {
		return mLateralAccValid;    // BR2_QB_Querbeschl
	}

	std::string toString() const;

private:
	bool mValid;						// Checksumme (8 bit)

	boost::uint8_t mCounter;			// Zähler

	//Package with MuX=1 never occurred!
	boost::uint8_t mNrOfTeeth;			// BR2_Zahnzahl

	math::flt mLateralAcceleration;		// BR2_Querbeschl

	bool mMux;							// BR2_Querb_Zahn
	bool mLateralAccValid;				// BR2_QB_Querbeschl


};

/**
 * Operator to init a AccelerationAndTeeth from a msg_data_ind.
 */
void operator>>(CanMessage  const & can_msg, AccelerationAndTeeth  & accelerationAndTeeth);

/********************************************/

//helper functions
static inline ::math::flt normalisedSpeedToThrottle(::math::flt normalizedSpeed, ::math::flt minThrottleVoltage, ::math::flt maxThrottleVoltage)
{
	using namespace math;
	math::flt const clampedSpeed = std::min(math::flt(1), std::max(math::flt(0), normalizedSpeed));
	math::flt const throttleRange = maxThrottleVoltage - minThrottleVoltage;
	math::flt throttleVoltage = minThrottleVoltage + (clampedSpeed * throttleRange);
	return throttleVoltage;
}

static inline ::math::flt normalisedSpeedToBrake(::math::flt normalizedSpeed, ::math::flt minBrakePressure, ::math::flt maxBrakePressure)
{
	using namespace math;
	math::flt const clampedSpeed = -(std::min(math::flt(0), std::max(math::flt(-1), normalizedSpeed)));
	math::flt const brakeRange = maxBrakePressure - minBrakePressure;
	math::flt brakePressure = minBrakePressure + (clampedSpeed * brakeRange);
	return brakePressure;
}

static inline ::math::flt throttleBrakeToNormalizedSpeed(::math::flt throttleVoltage, ::math::flt brakePressure, ::math::flt minThrottleVoltage, ::math::flt maxThrottleVoltage, ::math::flt minBrakePressure, ::math::flt maxBrakePressure)
{
	using namespace math;
	math::flt const throttleRange = maxThrottleVoltage - minThrottleVoltage;
	math::flt const brakeRange = maxBrakePressure - minBrakePressure;

	math::flt const normThrottle = (throttleVoltage - minThrottleVoltage) / throttleRange;
	math::flt const normBrake = -((brakePressure - minBrakePressure) / brakeRange);
	math::flt const voltageTolerance = 0.05;	//since the standing gas varies from 0.75 V (specified) to 0.80 V


	if (throttleVoltage > minThrottleVoltage + voltageTolerance && brakePressure > minBrakePressure) {
		//std::cout << "[PassatCanMessage] throttleBrake normalization error: throttleVoltage=" << throttleVoltage << " || normThrottle " << normThrottle << " && brakePressure=" << brakePressure << std::endl;
		return normBrake;
	}
	else if (throttleVoltage <= minThrottleVoltage + voltageTolerance) {
		return normBrake;
	}
	else if (brakePressure <= minBrakePressure) {
		return normThrottle;
	}
	else {
		//should never get here
		//assert(false); but it did (06/29/11)
		return normBrake;
	}
}

}
}
}
}

#if !defined(NO_TIMED_DATA)

#include <core/TimedData.h>
#include <util/Ports.h>

#include <boost/preprocessor/list/for_each.hpp>

#define PASSAT_CAN_MESSAGES_TYPES (WatchdogBatteryStatus, (BrakeCommand, (WatchdogStatus, (BrakeStatus1, (BrakeStatus2, (ThrottleCommand, (ThrottleStatus, (GearCommand, (FunctionCommand, (GearStatus, (ParkAssistData, (SteerAssist3Status, (SignalWipersStatus, (Light1Status, (Motor1Status, (GraMainSwitch, (Airbag, (AbsEsp, (AbsEspFake, (PathPulse, (WheelSpeeds, (SteeringWheelSpeed, (TargetGear, (Acceleration, (YawAndBrakePressure, (AccelerationAndTeeth, BOOST_PP_NIL))))))))))))))))))))))))))


#define PASSAT_CAN_MESSAGE_DECLARE(r, Namespace, Elem)\
	namespace aa { namespace modules { namespace io { namespace passat { typedef TimedData<Elem> Timed##Elem; } } } } \
	extern template class RTT::InputPort<aa::modules::io::passat::Timed##Elem>; \
	extern template class RTT::OutputPort<aa::modules::io::passat::Timed##Elem>;

#define PASSAT_CAN_MESSAGE_INSTANTIATE(r, data, elem)\
	template class RTT::InputPort<aa::modules::io::passat::Timed##elem>; \
	template class RTT::OutputPort<aa::modules::io::passat::Timed##elem>;

BOOST_PP_LIST_FOR_EACH(PASSAT_CAN_MESSAGE_DECLARE, _, PASSAT_CAN_MESSAGES_TYPES);

#endif

