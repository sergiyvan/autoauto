#pragma once

#include <math/AutoMath.h>

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

enum {
	BITMASK_NORMAL  	= 0,
	BITMASK_GROUND 		= 1, 	// Bit1. Ground point
	BITMASK_DIRT 		= 2, 	// Bit2. Dirt (on the scanner window e.g.)
	BITMASK_CLUTTER		= 4, 	// Bit3. Rain, Snow, Spray
	BITMASK_TRANSPARENT = 8, 	// Bit4. Transparent scan point (all points that have double reflections)
	BITMASK_INVALID 	= 16,  	// Bit5. Invalid Scan Point (stems from ALasca Data)
};

// From lowest to highest
enum Layer {
	LAYER_RED			= 0,
	LAYER_BLUE			= 1,
	LAYER_GREEN			= 2,
	LAYER_YELLOW		= 3
};

// From first to last
enum Echo {
	ECHO1 = 0,
	ECHO2 = 1,
	ECHO3 = 2
};

namespace ecudata
{
typedef ::math::flt flt;
typedef ::math::Vec3 Vec3;

const unsigned int SCANNER_ID_FL = 212;
const unsigned int SCANNER_ID_FC = 213;
const unsigned int SCANNER_ID_FR = 211;
const unsigned int SCANNER_ID_RR = 210;
const unsigned int SCANNER_ID_RC = 215;
const unsigned int SCANNER_ID_RL = 214;

const flt X_IMU_OFFSET = -2.709; //HACK wo ist der ursprung der ecu (fritz)?
const flt Y_IMU_OFFSET = 0.0;
const flt Z_IMU_OFFSET = 0.0;
const Vec3 POSITION_IMU_OFFSET(X_IMU_OFFSET, Y_IMU_OFFSET, Z_IMU_OFFSET);
const flt ROLL_IMU_OFFSET 	= 0.0;
const flt PITCH_IMU_OFFSET 	= 0.0;
const flt YAW_IMU_OFFSET 	= 0.0;
const Vec3 ROTATION_IMU_OFFSET(ROLL_IMU_OFFSET, PITCH_IMU_OFFSET, YAW_IMU_OFFSET);

/*
* Binary Masks
*/
const int BITMASK_GROUND 	= 1;
const int BITMASK_DIRT 		= 2;
const int BITMASK_CLUTTER	= 4;


/**
*	Converts the flag given in Lux Ecu Scan Point format to the common
*	Ibeo Scan Point flag format
*/
unsigned int convertFlagToCommon(unsigned int flag);

}

namespace luxdata
{
typedef ::math::flt flt;
typedef ::math::Vec3 Vec3;

// From lowest to highest
enum Lux8Layer {
	LUX8LAYER_RED			= 0,
	LUX8LAYER_ORANGE		= 1,
	LUX8LAYER_YELLOW		= 2,
	LUX8LAYER_LIGHT_GREEN	= 3,
	LUX8LAYER_DARK_GREEN	= 4,
	LUX8LAYER_LIGHT_BLUE	= 5,
	LUX8LAYER_DARK_BLUE		= 6,
	LUX8LAYER_VIOLET		= 7
};

/**
* Technical data of the Ibeo Lux
*/

const unsigned int NUM_LAYERS = 4;

const flt SIGMA_DISTANCE = 0.07; // [m]
const flt SIGMA_ANGLE = 0.02 * math::D2R;

/*
* 	Mounting data of the Lux on SPOB (relative to IMU, the car starts looking in y-direction)
*/

const flt POSITION_X	= 4.40;
const flt POSITION_Y	= 0.02;
const flt POSITION_Z	= 0.05;

const Vec3 POSITION(POSITION_X, POSITION_Y, POSITION_Z);

const flt ROLL 	= 0.0;
const flt PITCH	= 0.0;
const flt YAW	= 0.0;

const Vec3 ROTATION(ROLL, PITCH, YAW);

/*
* The vertical angle of each emitted ray in degree
* 	RedRay    = layer0
*	BlueRay   = layer1
*	GreenRay  = layer2
*	YellowRay = layer3
*/

const flt DEGREE_RED_RAY    = -1.2; // [deg]
const flt DEGREE_BLUE_RAY   = -0.4; // [deg]
const flt DEGREE_GREEN_RAY  =  0.4; // [deg]
const flt DEGREE_YELLOW_RAY =  1.2; // [deg]

/*
* The vertical angle of each emitted ray in degree
* 	layer 0 to layer 7
*/

const flt DEGREE_LUX8_RED_RAY			= -2.8; // [deg]
const flt DEGREE_LUX8_ORANGE_RAY		= -2.0; // [deg]
const flt DEGREE_LUX8_YELLOW_RAY		= -1.2; // [deg]
const flt DEGREE_LUX8_LIGHT_GREEN_RAY	= -0.4; // [deg]
const flt DEGREE_LUX8_DARK_GREEN_RAY    = +0.4; // [deg]
const flt DEGREE_LUX8_LIGHT_BLUE_RAY    = +1.2; // [deg]
const flt DEGREE_LUX8_DARK_BLUE_RAY		= +2.0; // [deg]
const flt DEGREE_LUX8_VIOLET_RAY		= +2.8; // [deg]


/*
* Vertical angle resolution. The scans on the different layers are interlaced.
* This means that the scan of layer0/1 is alternated with that of layer2/3.
* In this context, an angle resolution of 0.25 degree means, that two successive
* scanpoints (one on layer0/1 and one on layer2/3) have an angle of 0.25, whereas
* two neighboured scans on the same layer have an angle of 2*0.25 (0.5)
*/

const flt SCAN_FREQUENCY = 12.5; // [Hz]
// const flt SCAN_FREQUENCY = 25.0; // [Hz]
// const flt SCAN_FREQUENCY = 50.0; // [Hz]

/*
* Binary Masks
*/

const int TRANSPARENT_MASK 	= 1;
const int CLUTTER_MASK 		= 2;
const int GROUND_MASK 		= 4;
const int DIRT_MASK 		= 8;
const int OTHER_MASK 		= 16;

/*
* IS_CONSTANT_ANGLE_RESOLUTION = true:
* The angle resolution ist constant on the whole scan area
* IS_CONSTANT_ANGLE_RESOLUTION = false:
* The angle resolution ist constant on some defined sectors:
* central area: -10 to +10
* middle area : -30 to +30
//		lateral area: -60 to +50
const bool  IS_CONSTANT_ANGLE_RESOLUTION = true;


// This data depends on the SCAN_FREQUENCY (12.5Hz: 0.25 | 25Hz: 0.25 | 50Hz: 0.5)
const flt CONSTANT_ANGLE_RESOLUTION         = 0.25;  // [deg]


// The variable angle resolution is only available at a scan frequency of 12.5 Hz
const flt VARIABLE_ANGLE_RESOLUTION_CENTRAL = 0.125; // [deg]
const flt VARIABLE_ANGLE_RESOLUTION_MIDDLE  = 0.25;  // [deg]
const flt VARIABLE_ANGLE_RESOLUTION_LATERAL = 0.5;   // [deg]
*/

/*
* Not all layers are available in the whole scan area.
*	Central Working Area (-50 to +35): layer 0 - 3
*	Lateral Working Area (-60 to -50 and +35 to +50): layer 0 - 1
*/
const flt CENTRAL_WORKING_ANGLE_LEFT  = +35.0 * math::D2R;
const flt CENTRAL_WORKING_ANGLE_RIGHT = -50.0 * math::D2R;
const flt LATERAL_WORKING_ANGLE_LEFT  = +50.0 * math::D2R;
const flt LATERAL_WORKING_ANGLE_RIGHT = -60.0 * math::D2R;

/**
*	Converts the flag given in Lux Scan Point format to the common
*	Ibeo Scan Point flag format
*/
unsigned int convertFlagToCommon(unsigned int flag);

}

namespace alascadata
{
typedef ::math::flt flt;
typedef ::math::Vec3 Vec3;
const flt SIGMA_DISTANCE = 0.10; // [m]
const flt SIGMA_ANGLE = 0.04 * math::D2R;

/**
* 	Mounting data of the Alasca on SPOB (relative to IMU, the car starts looking in y-direction)
*/

const flt POSITION_X	= 4.40; //0.02;
const flt POSITION_Y	= 0.02; //4.40;
const flt POSITION_Z	= 0.05;

const Vec3 POSITION(POSITION_X, POSITION_Y, POSITION_Z);

const flt ROLL 	= 0.0;
const flt PITCH	= 0.0;
const flt YAW	= 0.0;

const Vec3 ROTATION(ROLL, PITCH, YAW);

/**
* 	Technical data of the Ibeo Alasca
*/

const flt MIN_ANGLE = -100.0 * math::D2R; // [rad]
const flt MAX_ANGLE = +100.0 * math::D2R; // [rad]

// The vertical angle of each emitted ray in degree
// RedRay    = layer0
// BlueRay   = layer1
// GreenRay  = layer2
// YellowRay = layer3
const flt DEGREE_RED_RAY    = -1.2; // [deg]
const flt DEGREE_BLUE_RAY   = -0.4; // [deg]
const flt DEGREE_GREEN_RAY  =  0.4; // [deg]
const flt DEGREE_YELLOW_RAY =  1.2; // [deg]

const flt SCAN_FREQUENCY              = 12.5; // [Hz]

}

}
}
}
}