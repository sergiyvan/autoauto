#pragma once

#include <core/TimedData.h>
#include <QDataStream>
#include <boost/cstdint.hpp>
#include <math/AutoMath.h>
#include <math/Types.h>

#include <modules/io/anagate/AnaGateMessages.h>
#include <aa/modules/io/passat/PassatCanMessages.h>

#include <modules/models/carstate/CarState.h>
#include "PassatCarState.h"

namespace aa
{
namespace modules
{
namespace models
{

namespace carstate
{

// Aevit -> simple CarState


// Passat -> simple::modules::models::carstate::CarState
void operator>>(aa::modules::io::passat::TimedWatchdogStatus const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

//	cs.lastMessageId = data.mId();
	cs.autonomousControl = data.watchdogState() == aa::modules::io::passat::TimedWatchdogStatus::ActoricReady;
}

void operator>>(aa::modules::io::passat::TimedWatchdogBatteryStatus const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

//	cs.lastMessageId = data.mId();
}

void operator>>(aa::modules::io::passat::TimedBrakeStatus1 const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

//	cs.lastMessageId = data.mId();
}

void operator>>(aa::modules::io::passat::TimedBrakeStatus2 const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

//	cs.lastMessageId = data.mId();
}

void operator>>(aa::modules::io::passat::TimedThrottleStatus const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

//	cs.lastMessageId = data.mId();
}

void operator>>(aa::modules::io::passat::TimedGearStatus const & data,::modules::models::carstate::CarState & cs)
{
	using aa::modules::io::passat::TimedGearStatus;

	if (!data.valid()) {
		return;
	}

//	cs.lastMessageId = data.mId();
	switch (data.chosenPos()) {
	case TimedGearStatus::ErrorPos:
		cs.gearPosition =::modules::models::carstate::CarState::GEAR_ERROR;
		break;

	case TimedGearStatus::BetweenPos:
		cs.gearPosition =::modules::models::carstate::CarState::GEAR_BETWEEN;
		break;

	case TimedGearStatus::Pos_P_Key_Lock_Release:
		cs.gearPosition =::modules::models::carstate::CarState::GEAR_PARK;
		break;

	case TimedGearStatus::Pos_R:
		cs.gearPosition =::modules::models::carstate::CarState::GEAR_REVERSE;
		break;

	case TimedGearStatus::Pos_N:
		cs.gearPosition =::modules::models::carstate::CarState::GEAR_NEUTRAL;
		break;

	case TimedGearStatus::TiptronicManual:
	case TimedGearStatus::Pos_L:
	case TimedGearStatus::Pos_S_Automatic_Sport:
	case TimedGearStatus::Pos_Z2:
	case TimedGearStatus::Pos_Z1:
	case TimedGearStatus::Pos_RSP_Manual_Sport:
	case TimedGearStatus::Pos_D_Automatic:
	case TimedGearStatus::Pos_4:
	case TimedGearStatus::Pos_3:
	case TimedGearStatus::Pos_2:
	case TimedGearStatus::Pos_1:
		cs.gearPosition =::modules::models::carstate::CarState::GEAR_DRIVE;
		break;

	default:
		cs.gearPosition =::modules::models::carstate::CarState::GEAR_ERROR;
		break;
	}
}

void operator>>(aa::modules::io::passat::TimedSteerAssist3Status const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

//	cs.lastMessageId = data.mId();
	cs.wheelPosition = math::rangeCut<math::flt>(-1.0, data.normalizedSteerAngle(), 1.0);
}

void operator>>(aa::modules::io::passat::TimedSignalWipersStatus const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

	if (data.turnLeft()) {
		cs.turnSignal = 1;
	}
	else if (data.turnRight()) {
		cs.turnSignal = 2;
	}
	else {
		cs.turnSignal = 0;
	}

	cs.siren = data.horn();
}

void operator>>(aa::modules::io::passat::TimedWheelSpeeds const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}
}

void operator>>(aa::modules::io::passat::TimedPathPulse const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}
}

void operator>>(aa::modules::io::passat::TimedLight1Status const & data,::modules::models::carstate::CarState & cs)
{
	if (!data.valid()) {
		return;
	}

	if (data.warningLights()) {
		cs.turnSignal = 3;
	}

	cs.headlights = data.lowBeam() ? 1 : 0;
}

void operator>>(aa::modules::io::passat::TimedMotor1Status const & data,::modules::models::carstate::CarState & cs)
{
	/*
	if (!data.valid())
		return;
	*/
}





// Passat -> modules::models::carstate::PassatCarState

void operator>>(aa::modules::io::passat::TimedWatchdogStatus const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.watchdogStatus = data;
}

void operator>>(aa::modules::io::passat::TimedWatchdogBatteryStatus const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.watchdogBatteryStatus = data;
}

void operator>>(aa::modules::io::passat::TimedBrakeStatus1 const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.brakeStatus1 = data;
}

void operator>>(aa::modules::io::passat::TimedBrakeStatus2 const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.brakeStatus2 = data;
}

void operator>>(aa::modules::io::passat::TimedThrottleStatus const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.throttleStatus = data;
}

void operator>>(aa::modules::io::passat::TimedGearStatus const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.gearStatus = data;
}

void operator>>(aa::modules::io::passat::TimedSteerAssist3Status const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.steerAssist3Status = data;
}

void operator>>(aa::modules::io::passat::TimedSignalWipersStatus const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.signalWipersStatus = data;
}

void operator>>(aa::modules::io::passat::TimedWheelSpeeds const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.wheelSpeeds = data;
}

void operator>>(aa::modules::io::passat::TimedPathPulse const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.pathPulse = data;
}

void operator>>(aa::modules::io::passat::TimedLight1Status const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.light1Status = data;
}

void operator>>(aa::modules::io::passat::TimedMotor1Status const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.motor1Status = data;
}

void operator>>(aa::modules::io::passat::TimedAbsEsp const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.absEsp = data;
}

void operator>>(aa::modules::io::passat::TimedYawAndBrakePressure const & data, modules::models::carstate::PassatCarState & cs)
{
	if (!data.valid()) {
		return;
	}

	cs.yawAndBrakePressure = data;
}


//mimaxcountStates

struct minmaxcountGasState {
	minmaxcountGasState()
		: gas_deviceIsActive_truecount(0)
		, gas_deviceIsActive_falsecount(0)
		, gas_indError_truecount(0)
		, gas_indError_falsecount(0)
		, gas_sharedError_truecount(0)
		, gas_sharedError_falsecount(0)
		, gas_isRunning_truecount(0)
		, gas_isRunning_falsecount(0)
		, gas_inputData_min(0xffff)
		, gas_inputData_max(0)
		, gas_requestIsInactive_truecount(0)
		, gas_requestIsInactive_falsecount(0)
		, gas_isPBrakeRequest_truecount(0)
		, gas_isPBrakeRequest_falsecount(0)
		, gas_isClutchRequest_truecount(0)
		, gas_isClutchRequest_falsecount(0)
		, gas_isEvalConnected_truecount(0)
		, gas_isEvalConnected_falsecount(0)
		, gas_isEvalActive_truecount(0)
		, gas_isEvalActive_falsecount(0)
		, gas_encoder_min(0xffff)
		, gas_encoder_max(0)
		, gas_command_counter()
		, gas_command_data_min()
		, gas_command_data_max()
	{}

	quint16 	gas_deviceIsActive_truecount;
	quint16 	gas_deviceIsActive_falsecount;

	//bool 		gas_indError
	quint16 	gas_indError_truecount;
	quint16 	gas_indError_falsecount;

	//bool 		gas_sharedError;
	quint16 	gas_sharedError_truecount;
	quint16 	gas_sharedError_falsecount;

//	bool 		gas_isRunning;
	quint16 	gas_isRunning_truecount;
	quint16 	gas_isRunning_falsecount;

//	quint16 	gas_inputData;
	quint16 	gas_inputData_min;
	quint16 	gas_inputData_max;

// 	bool 		gas_requestIsInactive;
	quint16 	gas_requestIsInactive_truecount;
	quint16 	gas_requestIsInactive_falsecount;

// 	bool 		gas_isPBrakeRequest;
	quint16 	gas_isPBrakeRequest_truecount;
	quint16 	gas_isPBrakeRequest_falsecount;

// 	bool 		gas_isClutchRequest;
	quint16 	gas_isClutchRequest_truecount;
	quint16 	gas_isClutchRequest_falsecount;

// 	bool 		gas_isEvalConnected;
	quint16 	gas_isEvalConnected_truecount;
	quint16 	gas_isEvalConnected_falsecount;

// 	bool 		gas_isEvalActive;
	quint16 	gas_isEvalActive_truecount;
	quint16 	gas_isEvalActive_falsecount;

// 	quint16 	gas_encoder;
	quint16 	gas_encoder_min;
	quint16 	gas_encoder_max;

// 	uchar 		gas_command;
	std::map <uchar, quint16>	gas_command_counter;//how often which command?

// 	quint16 	gas_data;
	std::map <uchar, quint16>	gas_command_data_min;//min gas_data for each command
	std::map <uchar, quint16>	gas_command_data_max;//max gas_data for each command
};

struct minmaxcountWheelState {
	minmaxcountWheelState()
		: wheel_deviceIsActive_truecount(0)
		, wheel_deviceIsActive_falsecount(0)
		, wheel_indError_truecount(0)
		, wheel_indError_falsecount(0)
		, wheel_sharedError_truecount(0)
		, wheel_sharedError_falsecount(0)
		, wheel_isRunning_truecount(0)
		, wheel_isRunning_falsecount(0)
		, wheel_inputData_min(0xffff)
		, wheel_inputData_max(0)
		, wheel_encoder_min(0xffff)
		, wheel_encoder_max(0)
		, wheel_requestIsInactive_truecount(0)
		, wheel_requestIsInactive_falsecount(0)
		, wheel_isPBrakeRequest_truecount(0)
		, wheel_isPBrakeRequest_falsecount(0)
		, wheel_isClutchRequest_truecount(0)
		, wheel_isClutchRequest_falsecount(0)
		, wheel_isEvalConnected_truecount(0)
		, wheel_isEvalConnected_falsecount(0)
		, wheel_isEvalActive_truecount(0)
		, wheel_isEvalActive_falsecount(0)
		, wheel_command_counter()
		, wheel_command_data_min()
		, wheel_command_data_max()
	{}

	quint16 	wheel_deviceIsActive_truecount;
	quint16 	wheel_deviceIsActive_falsecount;

	//bool 		wheel_indError
	quint16 	wheel_indError_truecount;
	quint16 	wheel_indError_falsecount;

	//bool 		wheel_sharedError;
	quint16 	wheel_sharedError_truecount;
	quint16 	wheel_sharedError_falsecount;

//	bool 		wheel_isRunning;
	quint16 	wheel_isRunning_truecount;
	quint16 	wheel_isRunning_falsecount;

//	quint16 	wheel_inputData;
	quint16 	wheel_inputData_min;
	quint16 	wheel_inputData_max;

// 	quint16 	wheel_encoder;
	quint16 	wheel_encoder_min;
	quint16 	wheel_encoder_max;

// 	bool 		wheel_requestIsInactive;
	quint16 	wheel_requestIsInactive_truecount;
	quint16 	wheel_requestIsInactive_falsecount;

// 	bool 		wheel_isPBrakeRequest;
	quint16 	wheel_isPBrakeRequest_truecount;
	quint16 	wheel_isPBrakeRequest_falsecount;

// 	bool 		wheel_isClutchRequest;
	quint16 	wheel_isClutchRequest_truecount;
	quint16 	wheel_isClutchRequest_falsecount;

// 	bool 		wheel_isEvalConnected;
	quint16 	wheel_isEvalConnected_truecount;
	quint16 	wheel_isEvalConnected_falsecount;

// 	bool 		wheel_isEvalActive;
	quint16 	wheel_isEvalActive_truecount;
	quint16 	wheel_isEvalActive_falsecount;

// 	uchar 		wheel_command;
	std::map <uchar, quint16>	wheel_command_counter;//how often which command?

// 	quint16 	wheel_data;
	std::map <uchar, quint16>	wheel_command_data_min;//min wheel_data for each command
	std::map <uchar, quint16>	wheel_command_data_max;//max wheel_data for each command
};

struct minmaxcountInterfaceState {
	minmaxcountInterfaceState()
		: interface_isPresent_truecount(0)
		, interface_isPresent_falsecount(0)
		, interface_isBooted_truecount(0)
		, interface_isBooted_falsecount(0)
		, interface_isValidCoilPulse_truecount(0)
		, interface_isValidCoilPulse_falsecount(0)
		, interface_brakelightActive_truecount(0)
		, interface_brakelightActive_falsecount(0)
		, interface_parkinglightActive_truecount(0)
		, interface_parkinglightActive_falsecount(0)
		, interface_steeringServoEngaged_truecount(0)
		, interface_steeringServoEngaged_falsecount(0)
		, interface_remoteOffSwitchActive_truecount(0)
		, interface_remoteOffSwitchActive_falsecount(0)
		, interface_ignitionActive_truecount(0)
		, interface_ignitionActive_falsecount(0)
		, interface_throttleClutchEngaged_truecount(0)
		, interface_throttleClutchEngaged_falsecount(0)
		, interface_speed_min(0xffff)
		, interface_speed_max(0)
		, interface_bat1_voltage_min(0xffff)
		, interface_bat1_voltage_max(0)
		, interface_bat2_voltage_min(0xffff)
		, interface_bat2_voltage_max(0)
		, interface_command_counter()
		, interface_command_data_min()
		, interface_command_data_max()
	{}

	quint16 	interface_isPresent_truecount;
	quint16 	interface_isPresent_falsecount;

	quint16 	interface_isBooted_truecount;
	quint16 	interface_isBooted_falsecount;

	quint16 	interface_isValidCoilPulse_truecount;
	quint16 	interface_isValidCoilPulse_falsecount;

	quint16 	interface_brakelightActive_truecount;
	quint16 	interface_brakelightActive_falsecount;

	quint16 	interface_parkinglightActive_truecount;
	quint16 	interface_parkinglightActive_falsecount;

	quint16 	interface_steeringServoEngaged_truecount;
	quint16 	interface_steeringServoEngaged_falsecount;

	quint16 	interface_remoteOffSwitchActive_truecount;
	quint16 	interface_remoteOffSwitchActive_falsecount;

	quint16 	interface_ignitionActive_truecount;
	quint16 	interface_ignitionActive_falsecount;

	quint16 	interface_throttleClutchEngaged_truecount;
	quint16 	interface_throttleClutchEngaged_falsecount;

//	quint16 	interface_inputData;
	quint16 	interface_speed_min;
	quint16 	interface_speed_max;
	quint16 	interface_bat1_voltage_min;
	quint16 	interface_bat1_voltage_max;
	quint16 	interface_bat2_voltage_min;
	quint16 	interface_bat2_voltage_max;

// 	uchar 		interface_command;
	std::map <uchar, quint16>	interface_command_counter;//how often which command?

// 	quint16 	interface_data;
	std::map <uchar, quint16>	interface_command_data_min;//min interface_data for each command
	std::map <uchar, quint16>	interface_command_data_max;//max interface_data for each command
};

struct minmaxcountIcenterState {
	minmaxcountIcenterState()
		: icenter_isPresent_truecount(0)
		, icenter_isPresent_falsecount(0)

		, icenter_speed_min(0xffff)
		, icenter_speed_max(0)

		, icenter_command_counter()
		, icenter_command_data_min()
		, icenter_command_data_max()
	{}

	quint16 	icenter_isPresent_truecount;
	quint16 	icenter_isPresent_falsecount;


//	quint16 	icenter_inputData;
	quint16 	icenter_speed_min;
	quint16 	icenter_speed_max;

// 	uchar 		icenter_command;
	std::map <uchar, quint16>	icenter_command_counter;//how often which command?

// 	quint16 	icenter_data;
	std::map <uchar, quint16>	icenter_command_data_min;//min icenter_data for each command
	std::map <uchar, quint16>	icenter_command_data_max;//max icenter_data for each command
};


}

}

}


}

