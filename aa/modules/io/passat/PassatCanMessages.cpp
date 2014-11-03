#include "PassatCanMessages.h"
#include <sstream>
#include <iostream>
#include <util/StringHelper.h>
#include "PassatCanMessages.ipp"

using namespace math;

namespace aa
{
namespace modules
{
namespace io
{
namespace passat
{
using ::modules::io::anagate::CanMessage;
math::flt const SteerAssist3Status::MAXSTEERINGWHEELANGLE = 530;

template void operator >> <CanMessage>(GearCommand const &, CanMessage &);
template void operator >> <CanMessage>(WheelSpeedsFake const &, CanMessage &);
template void operator >> <CanMessage>(AbsEspFake const &, CanMessage &);

void operator>>(WatchdogCommand const & watchdogCommand, CanMessage & msg)
{
	operator>> <CanMessage>(watchdogCommand, msg);
}


void operator>>(CanMessage const & msg, WatchdogStatus & watchdogStatusMsg)
{
	operator>> <CanMessage>(msg, watchdogStatusMsg);
}

void operator>>(CanMessage const & msg, WatchdogBatteryStatus & watchdogBatteriestatusMsg)
{
	operator>> <CanMessage>(msg, watchdogBatteriestatusMsg);
}

void operator>>(BrakeCommand const & cmd, CanMessage & msg)
{
	operator>> <CanMessage>(cmd, msg);
}

void operator>>(CanMessage const & msg, BrakeStatus1 & statusBremsboosterMsg)
{
	operator>> <CanMessage>(msg, statusBremsboosterMsg);
}

void operator>>(CanMessage const & msg, BrakeStatus2 & status2BremsboosterMsg)
{
	operator>> <CanMessage>(msg, status2BremsboosterMsg);
}

void operator>>(ThrottleCommand const & cmd, CanMessage & msg)
{
	operator>> <CanMessage>(cmd, msg);
}

void operator>>(CanMessage const & msg, ThrottleStatus & throttleStatus)
{
	operator>> <CanMessage>(msg, throttleStatus);
}

void operator>>(GearCommand const & cmd, CanMessage & msg)
{
	operator>> <CanMessage>(cmd, msg);
}

void operator>>(FunctionCommand const & cmd, CanMessage & msg)
{
	operator>> <CanMessage>(cmd, msg);
}

void operator>>(AbsEspFake const & cmd, CanMessage & msg)
{
	operator>> <CanMessage>(cmd, msg);
}

void operator>>(WheelSpeedsFake const & cmd, CanMessage & msg)
{
	operator>> <CanMessage>(cmd, msg);
}


void operator>>(CanMessage const & msg, GearStatus & gearStatus)
{
	operator>> <CanMessage>(msg, gearStatus);
}

void operator>>(CanMessage const & msg, ParkAssistData & parkAssistData)
{
	operator>> <CanMessage>(msg, parkAssistData);
}

void operator>>(CanMessage const & msg, SteerAssist3Status & steerAssist3Status)
{
	operator>> <CanMessage>(msg, steerAssist3Status);
}


void operator>>(CanMessage const & msg, SignalWipersStatus & signalWipersStatus)
{
	operator>> <CanMessage>(msg, signalWipersStatus);
}

void operator>>(CanMessage const & msg, Light1Status & light1Status)
{
	operator>> <CanMessage>(msg, light1Status);
}


void operator>>(CanMessage const & msg, Motor1Status & motor1Status)
{
	operator>> <CanMessage>(msg, motor1Status);
}

void operator>>(CanMessage const & msg, GraMainSwitch & graMainSwitch)
{
	operator>> <CanMessage>(msg, graMainSwitch);
}

void operator>>(CanMessage const & msg, Airbag & airbag)
{
	operator>> <CanMessage>(msg, airbag);
}

void operator>>(CanMessage const & msg, AbsEsp & absEsp)
{
	operator>> <CanMessage>(msg, absEsp);
}


void operator>>(CanMessage const & msg, PathPulse & pathPulse)
{
	operator>> <CanMessage>(msg, pathPulse);
}


void operator>>(CanMessage const & msg, WheelSpeeds & wheelSpeeds)
{
	operator>> <CanMessage>(msg, wheelSpeeds);
}


void operator>>(CanMessage const & msg, SteeringWheelSpeed & steeringWheelSpeed)
{
	operator>> <CanMessage>(msg, steeringWheelSpeed);
}

void operator>>(CanMessage const & msg, TargetGear & targetGear)
{
	operator>> <CanMessage>(msg, targetGear);
}

void operator>>(CanMessage const & msg, Acceleration & acceleration)
{
	operator>> <CanMessage>(msg, acceleration);
}


void operator>>(CanMessage const & msg, YawAndBrakePressure & yawAndBrakePressure)
{
	operator>> <CanMessage>(msg, yawAndBrakePressure);
}

void operator>>(CanMessage const & msg, AccelerationAndTeeth & accelerationAndTeeth)
{
	operator>> <CanMessage>(msg, accelerationAndTeeth);
}

/*********************************** To String Methods ****************************************/

std::string WatchdogStatus::toString() const
{
	std::stringstream s;
	s << "WatchdogStatus - Message (" << messageCounter() << ")\n";
	s << "\tWatchdogState:     \t";

	if (watchdogState() == WatchdogStatus::Off) {
		s << "Off";
	}
	else if (watchdogState() == WatchdogStatus::S1Off) {
		s << "Off(S1)";
	}
	else if (watchdogState() == WatchdogStatus::S2Off) {
		s << "Off(S2)";
	}
	else if (watchdogState() == WatchdogStatus::VoltageOn) {
		s << "VoltageOn";
	}
	else if (watchdogState() == WatchdogStatus::VoltageOn_Error) {
		s << "VoltageOn_Error";
	}
	else if (watchdogState() == WatchdogStatus::OperationReady) {
		s << "OperationReady";
	}
	else if (watchdogState() == WatchdogStatus::ActoricReady) {
		s << "ActoricReady";
	}
	else {
		s << "UNKNOWN!!! (we dont know this value: " << watchdogState() << ")";
	}

	s << "\tPowerOff:\t" << util::toYesNo(poweredDown());

	if (poweredDown()) {
		s << " Reason: ";

		if (powerOffReason() == WatchdogStatus::NoPowerOff) {
			s << "NoPowerOff";
		}
		else if (powerOffReason() == WatchdogStatus::S1) {
			s << "S1";
		}
		else if (powerOffReason() == WatchdogStatus::S2) {
			s << "S2";
		}
		else if (powerOffReason() == WatchdogStatus::Voltageproblem) {
			s << "Voltageproblem";
		}
		else if (powerOffReason() == WatchdogStatus::ErrorWatchdogCommand) {
			s << "ErrorWatchdogCommand";
		}
		else if (powerOffReason() == WatchdogStatus::ErrorControllerBrake) {
			s << "ErrorControllerBrake";
		}
		else if (powerOffReason() == WatchdogStatus::ErrorControllerGaspedal) {
			s << "ErrorControllerGaspedal";
		}
		else if (powerOffReason() == WatchdogStatus::MalfunctionBrake) {
			s << "MalfunctionBrake";
		}
		else if (powerOffReason() == WatchdogStatus::MalfunctionGaspedal) {
			s << "MalfunctionGaspedal";
		}
		else if (powerOffReason() == WatchdogStatus::Deactivation) {
			s << "Deactivation";
		}
		else if (powerOffReason() == WatchdogStatus::DriverIntervention) {
			s << "DriverIntervention";
		}
		else if (powerOffReason() == WatchdogStatus::MalfunctionActoricBox) {
			s << "MalfunctionActoricBox";
		}
		else if (powerOffReason() == WatchdogStatus::NotEnabled) {
			s << "NotEnabled";
		}
		else if (powerOffReason() == WatchdogStatus::ErrorControllerGear) {
			s << "ErrorControllerGear";
		}
		else if (powerOffReason() == WatchdogStatus::MalfunctionGear) {
			s << "MalfunctionGear";
		}
		else if (powerOffReason() == WatchdogStatus::Unknown) {
			s << "Unknown (code=255)";
		}
		else {
			s << "undefined (we dont know this value: " << powerOffReason() << ')';
		}
	}

	s << '\n';
	s << "\tVoltage:       \t" << util::toOkError(voltage()) << '\n';
	s << "\tWatchdogCommand:\t: " << util::toOkError(watchdogCommand()) << '\n';
	s << '\t' << "        \tWähl\tBremse\tGas\tFunc\tActoricBox" << '\n';
	s << '\t' << "Control:\t" << util::toOkError(controllerGear()) << '\t' << util::toOkError(controllerBrake()) << '\t' << util::toOkError(controllerThrottle()) << '\t' << util::toOkError(controllerFunction()) << "\t--\n";
	s << '\t' << "Status :\t" << util::toOkError(statusGear()) << '\t' << util::toOkError(statusBrake()) << '\t' << util::toOkError(statusThrottle()) << '\t' << util::toOkError(statusFunction()) << "\t" << util::toOkError(statusActoricBox()) << '\n';

	s << "\tGas/Brake (cur/req):\t" << util::toYesNo(gasBrakeEnabled()) << '/' << util::toYesNo(enableRequestGasBrake()) << '\n';
	s << "\tGear      (cur/req):\t" << util::toYesNo(gearEnabled()) << '/' << util::toYesNo(enableRequestGear()) << '\n';
	s << "\tDriver:\tbrakes " << util::toYesNo(driverBrakes()) << "\taccelerates: " << util::toYesNo(driverAccelerates()) << "\tshifts gear " << util::toYesNo(driverShiftsGear());

	return s.str();
}



std::string WatchdogBatteryStatus::toString() const
{
	std::stringstream s;
	s << "WatchdogBatteryStatus - Message\n";
	s << "\tvoltage:\t" << voltage() << " V";
	return s.str();
}


std::string BrakeStatus1::toString() const
{
	std::stringstream s;
	s << "Bremsbooster - Status(1) - Message\n";
	s << "\tPressureSensor1actual:\t" << actualPressure() << " bar\n";
	s << "\tTravelSensor:\t" << travelSensor() << " mm\n";
	s << "\tMagnetCurrent:\t" << magnetCurrent() << " mA\n";
	s << "\tMagnetPWMRatio:\t" << magnetPwmRatio() << " %\n";
	s << "\tMagnetVoltage:\t" << magnetVoltage() << " V\n";
	s << "\tBA is firing:\t" << util::toYesNo(brakeAssistActive());
	return s.str();
}

std::string BrakeStatus2::toString() const
{
	std::stringstream s;
	s << "Bremsbooster - Status(2) - Message\n";
	s << "\tBA function:\t" << util::toEnableDisable(brakeAssist()) << '\n';
	s << "\tPRC function:\t" << util::toEnableDisable(bitPRCFunc()) << '\n';
	s << "\tBCU3Status:\n";
	s << "\t\tError:" << util::toYesNo(bcu3StatusError()) << '\n';
	s << "\t\tVoltageLowHigh:" << util::toYesNo(bcu3StatusVoltageLowHigh()) << '\n';
	s << "\t\tBAWarnLamp:" << util::toYesNo(bcu3StatusBAWarnLamp()) << '\n';
	s << "\t\tPRCWarnLamp:" << util::toYesNo(bcu3StatusPRCWarnLamp()) << '\n';
	s << "\t\tMode:" << util::toYesNo(bcu3StatusMode()) << '\n';
	s << "\t\tReleaseSwitch:" << util::toYesNo(bcu3StatusReleaseSwitch()) << '\n';
	s << "\t\tPressureControlActive:" << util::toYesNo(bcu3StatusPressureControlActive()) << '\n';
	s << "\tDetected failure:\n";
	s << "\t\tBCU3intern:" << util::toOkError(detectedFailureBCU3intern()) << '\n';
	s << "\t\tPressureSensor:" << util::toOkError(detectedFailurePressureSensor()) << '\n';
	s << "\t\tTravelSensor:" << util::toOkError(detectedFailureTravelSensor()) << '\n';
	s << "\t\tReleaseSwitch:" << util::toOkError(detectedFailureReleaseSwitch()) << '\n';
	s << "\t\tSolenoid:" << util::toOkError(detectedFailureSolenoid()) << '\n';
	s << "\t\tPressControl:" << util::toOkError(detectedFailurePressControl()) << '\n';
	s << "\t\tCAN:" << util::toOkError(detectedFailureCan()) << '\n';
	s << "\t\tPreChargePump:" << util::toOkError(detectedFailurePreChargePump()) << '\n';
	s << "\tActual measure pressure: \t" << actualPressure() << " bar\n";
	s << "\tAccepted pressure demand: \t" << pressureDemand() << " bar";

	return s.str();
}

std::string ThrottleStatus::toString() const
{
	std::stringstream s;
	s << "Throttle - Status - Message\n";
	s << "\tvoltage1 GasPedal:\t" << voltageGasPedal1() << " V (Spannung am Ausgang des Gaspedals)\n";
	s << "\tvoltage2 GasPedal:\t" << voltageGasPedal2() << " V\n";
	s << "\tvoltage1 Motor:\t" << voltageMotor1() << " V (Spannung am Eingang des Motorsteuergerätes)\n";
	s << "\tvoltage2 Motor:\t" << voltageMotor2() << " V";

	return s.str();
}

std::string GearStatus::toString() const
{
	std::stringstream s;
	s	<< "GearStatus (" << cycleCounter() << ")\t  chosenPos " <<  chosenPos();
	return s.str();
}

std::string ParkAssistData::toString() const
{
	std::stringstream s;
	s	<< "ParkAssistData (" << messageCounter() << ")\t";
	s	<< " FrontLeft " << mData[FrontLeft] << " FrontRight " << mData[FrontRight];
	s 	<< " RearLeft " << mData[RearLeft] << " RearRight " << mData[RearRight];
	s	<< "\n FrontMiddleLeft " << mData[FrontMiddleLeft] << " FrontMiddleRight " << mData[FrontMiddleRight];
	s	<< " RearMiddleLeft " << mData[RearMiddleLeft] << " RearMiddleRight " << mData[RearMiddleRight];
	return s.str();
}

std::string SteerAssist3Status::toString() const
{
	std::stringstream s;
	s	<< "SteerAssist3Status (" << messageCounter() << ")\tvalid: " << util::toYesNo(valid());
	s	<< "\t Steermomentum " << steerMomentum() << " Valid " << util::toYesNo(steerMomentumValid());
	s	<< "\t SteerAngle " << steerAngle() << " Valid " << util::toYesNo(steerAngleValid());
	return s.str();
}

std::string SignalWipersStatus::toString() const
{
	std::stringstream s;
	s	<< "SignalWipersStatus ";
	s	<< "turnleft: " << util::toYesNo(turnLeft()) << "\tturnRight: " << util::toYesNo(turnRight());
	return s.str();
}

std::string Light1Status::toString() const
{
	std::stringstream s;
	s	<< "Light1Status ";
	s	<< "cycleCounter \tparkingLight \tlowBeam \t warningLights " << std::endl;
	s   <<  cycleCounter() << " \t" << util::toYesNo(parkingLight()) << " \t" << util::toYesNo(lowBeam()) << " \t" << util::toYesNo(warningLights());
	return s.str();
}

std::string Motor1Status::toString() const
{
	std::stringstream s;
	s	<< "Motor1Status ";
	s	<< "\tpedalStatus (no = no Problems) " << util::toYesNo(pedalStatus()) << "\tkickDown " << kickDown();
	s	<< "\tpedalValue " << pedalStatus() << "\trevolutionSpeed " << revolutionSpeed();
	return s.str();
}

std::string GraMainSwitch::toString() const
{
	std::stringstream s;
	s	<< "GraMainSwitch ";
	s	<< "\tNewCounter" << newCounter(); 									// GRA_Neu_Zaehler (4 bit)
	s	<< "\tTimegap" << timegap();											// GRA_Zeitluecke (2 bit)
	s	<< "\tMainSwitchSign" << util::toEnableDisable(mainSwitchSign())
		<< "\tType" 			 << util::toYesNo(type()) ;						// GRA_Hauptschalt, GRA_Typ_Hauptschalt
	s	<< "\tDownShort" << util::toYesNo(downShort())
		<< "\tDownLong"  << util::toYesNo(downLong());						// GRA_Down_kurz, GRA_Down_lang
	s	<< "\tUpShort" << util::toYesNo(upShort())
		<< "\tUpLong" << util::toYesNo(upLong());							// GRA_Up_kurz, GRA_Up_lang
	s	<< "\tTipDown" << util::toYesNo(tipDown())
		<< "\tTipUp"  << util::toYesNo(tipUp());							// GRA_Tip_Down, GRA_Tip_Up
	s	<< "\tCancel"  << util::toYesNo(cancel())
		<< "\tReset"  << util::toYesNo(reset())
		<< "\tRecall" << util::toYesNo(recall());							// GRA_Abbrechen, GRA_Neu_Setzen, GRA_Recall
	return s.str();

}

std::string Airbag::toString() const
{
	std::stringstream s;
	s	<< "Airbag ";
	s	<< "\tCounter" << counter();
	s	<< "\tSeatbelt" << seatbelt(); // AB1_Gurt_Fa: 3="seatbelt plugged in"; 2="not plugged in"; 1="Error"; 0="not available";
	return s.str();
}

std::string AbsEsp::toString() const
{
	std::stringstream s;
	s	<< "AbsEsp ";
	s	<< "\tCounter"  << counter();						// BR1_Zaehler
	s	<< "\tmEspSign" << util::toYesNo(espSign());			// BR1_ESP_Eingr
	s	<< "\tmAbsSign" << util::toYesNo(absSign());			// BR1_ABS_Brems
	s	<< "\tmBackupSign" << util::toYesNo(backupSign());	// BR1_Ersatz_Kmh
	s	<< "\tmSpeed" << speed();							// BR1_Rad_kmh
	return s.str();
}

std::string PathPulse::toString() const
{
	std::stringstream s;
	s	<< "PathPulse ";
	s	<< "\tCounter" << counter();
	s   << "\tQuality Bits, VL:" << util::toYesNo(qBitFrontL()) << "  VR:" << util::toYesNo(qBitFrontR()) << "  HL:" << util::toYesNo(qBitRearL()) << "  HR:" << util::toYesNo(qBitRearR());
	s   << "\tPath Pulse Values, VL:" << pulseFrontL() << "  VR:" << pulseFrontR() << "  HL:" << pulseRearL() << "  HR:" << pulseRearR();
	return s.str();
}



std::string WheelSpeeds::toString() const
{
	std::stringstream s;
	s	<< "WheelSpeeds ";
	s	<< "\tFrontL " << speedFrontL() << "\tFrontR " << speedFrontR();
	s	<< "\tRearL  " << speedRearL()  << "\tRearR  " << speedRearR();
	return s.str();
}

std::string SteeringWheelSpeed::toString() const
{
	std::stringstream s;
	s	<< "SteeringWheelSpeeds: ";
	s	<< "\tSpeed: " << steeringWheelSpeed() << "\tAngle " << steeringWheelAngle();
	return s.str();
}

std::string TargetGear::toString() const
{
	std::stringstream s;
	s	<< "TargetGear ";
	s	<< "\tCounter" << counter();
	s	<< "\ttargetPos:   " << targetPosString(targetPos())     << " ; " << targetPos();
	s	<< "\tselectedPos: " << selectedPosString(selectedPos()) << " ; " << selectedPos();
	return s.str();
}

std::string Acceleration::toString() const
{
	std::stringstream s;
	s	<< "Acceleration ";
	s	<< "\tAccSign: " << accSign();
	s	<< "\tLogitudinalAcc: "   << logitudinalAcc() << "\tCurrentAcc: " << currentAcc();
	return s.str();
}

std::string YawAndBrakePressure::toString() const
{
	std::stringstream s;
	s	<< "BrakeStatus5 ";
	//Giergeschw 16383 "Fehler" 16382 "ABS_only_System" 16381 "ASR_System"
	s	<< "\tVelicity Valid: " << util::toYesNo(yawRateValid()) << "\tYaw Velocity: " << yawRate();
	// Bremsdruck 4095 "Fehler"
	s	<< "\tBrake Valid" << util::toYesNo(brakePressureValid()) << "\tBrakePressure: "   << brakePressure();
	return s.str();
}

std::string AccelerationAndTeeth::toString() const
{
	std::stringstream s;
	s	<< "AccAndTeeth ";
	s	<< "Lateral Acceleration is Valid: " << util::toYesNo(lateralAccValid());

	if (mux()) {
		s	<< "mNrOfTeeth: "	<< ((uint) nrOfTeeth());
	}
	else {
		s	<< "mLateralAcc: "	<< lateralAcceleration();
	}

	return s.str();
}

}
}
}
}


#include <boost/preprocessor/list/first_n.hpp>
#define SUBSET BOOST_PP_LIST_FIRST_N(10, PASSAT_CAN_MESSAGES_TYPES)
// FIXME: Disabled instantiation as it is currently requires too much memory to compile: Split it to different files
BOOST_PP_LIST_FOR_EACH(PASSAT_CAN_MESSAGE_INSTANTIATE, _, SUBSET)

