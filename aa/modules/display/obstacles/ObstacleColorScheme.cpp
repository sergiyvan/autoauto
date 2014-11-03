/**
*  \author Sebastian Hempel
*  \brief Color schemes for obstacles
*/

#include "ObstacleColorScheme.h"

using namespace aa::modules::display::obstacles;
using namespace ::math;


const Vec3 ObstacleColorScheme::DEFAULT_BOUNDING_BOX_COLOR =
    Vec3(0.0f, 1.0f, 1.0f);

const Vec3 ObstacleColorScheme::DEFAULT_SOLID_TOP_COLOR =
    Vec3(0.24f, 0.76f, 0.15f);

const Vec3 ObstacleColorScheme::DEFAULT_SOLID_BOTTOM_COLOR =
    Vec3(0.2f, 0.3f, 0.8f);

const Vec3 ObstacleColorScheme::DEFAULT_ORIENTATION_FIRST_COLOR =
    Vec3(0.0f, 1.0f, 0.0f);

const Vec3 ObstacleColorScheme::DEFAULT_ORIENTATION_SECOND_COLOR =
    Vec3(0.0f, 0.0f, 1.0f);

const Vec3 ObstacleColorScheme::DEFAULT_ID_COLOR =
    Vec3(1.0f, 1.0f, 1.0f);

const Vec3 ObstacleColorScheme::DEFAULT_CENTER_COLOR =
    Vec3(0.0f, 1.0f, 0.0f);

const Vec3 ObstacleColorScheme::DEFAULT_CONTOUR_COLOR =
    Vec3(1.0f, 1.0f, 0.0f);

ObstacleColorScheme::ObstacleColorScheme()
	: mDefaultScheme(true)
	, mSensors(SENSOR_UNKNOWN)
	, mOptions(0)
{
	init();
}


ObstacleColorScheme::ObstacleColorScheme(bool boxes, bool solid, bool ids,
        bool center, bool orientations, bool contour)
	: mDefaultScheme(false)
	, mSensors(SENSOR_UNKNOWN)
	, mOptions(0)
{
	setOption(RENDER_BOUNDING_BOX, boxes);
	setOption(RENDER_SOLID_BOX, solid);
	setOption(RENDER_ID, ids);
	setOption(RENDER_CENTER, center);
	setOption(RENDER_ORIENTATION, orientations);
	setOption(RENDER_CONTOUR, contour);
}


ObstacleColorScheme::ObstacleColorScheme(unsigned int sensors)
	: mDefaultScheme(false)
	, mOptions(0)
	, mSensors(sensors)
{
	init();
}


void ObstacleColorScheme::init()
{
	setOption(RENDER_BOUNDING_BOX, true);
	setOption(RENDER_CENTER, true);
}


bool ObstacleColorScheme::check(OBSTACLE_RENDER_OPTION option)
{
	return (mOptions & option) != 0;
}


Vec3 ObstacleColorScheme::boundingBoxColor(bool tracked,
        unsigned int id)
{
	return ObstacleColorScheme::DEFAULT_BOUNDING_BOX_COLOR;
}


Vec3 ObstacleColorScheme::solidTopColor(bool tracked,
                                        unsigned int id)
{
	return makeColor(id);
	//return ObstacleColorScheme::DEFAULT_SOLID_TOP_COLOR;
}


Vec3 ObstacleColorScheme::solidBottomColor(bool tracked,
        unsigned int id)
{
	return makeColor(id);
	//return ObstacleColorScheme::DEFAULT_SOLID_BOTTOM_COLOR;
}


Vec3 ObstacleColorScheme::orientationFirstColor(bool tracked,
        unsigned int id)
{
	return ObstacleColorScheme::DEFAULT_ORIENTATION_FIRST_COLOR;
}


Vec3 ObstacleColorScheme::orientationSecondColor(bool tracked,
        unsigned int id)
{
	return ObstacleColorScheme::DEFAULT_ORIENTATION_SECOND_COLOR;
}


Vec3 ObstacleColorScheme::idColor(bool tracked,
                                  unsigned int id)
{
	return ObstacleColorScheme::DEFAULT_ID_COLOR;
}


Vec3 ObstacleColorScheme::centerColor(bool tracked,
                                      unsigned int id)
{
	return ObstacleColorScheme::DEFAULT_CENTER_COLOR;
}

Vec3 ObstacleColorScheme::contourColor(bool tracked,
                                       unsigned int id)
{
	return ObstacleColorScheme::DEFAULT_CONTOUR_COLOR;
}

ColorConfiguration ObstacleColorScheme::make(const aa::data::obstacle::Obstacle & obstacle)
{
	return make(false, obstacle.id());
}

ColorConfiguration ObstacleColorScheme::make(bool tracked,
        unsigned int id)
{
	ColorConfiguration result;

	setOptions(&result);

	//set colors
	result.mBoundingBoxColor = boundingBoxColor(tracked, id);
	result.mSolidTopColor = solidTopColor(tracked, id);
	result.mSolidBottomColor = solidBottomColor(tracked, id);
	result.mOrientationFirstColor = orientationFirstColor(tracked, id);
	result.mOrientationSecondColor = orientationSecondColor(tracked, id);
	result.mIdColor = idColor(tracked, id);
	result.mCenterColor = centerColor(tracked, id);
	result.mContourColor = contourColor(tracked, id);
	result.mContourWidth = 1.0;

	return result;
}


void ObstacleColorScheme::setOptions(ColorConfiguration * config)
{
	config->mRenderBoundingBox = checkBoundingBox();
	config->mRenderSolid = checkSolidBox();
	config->mRenderOrientation = checkOrientation();
	config->mRenderId = checkId();
	config->mRenderCenter = checkCenter();
}


void ObstacleColorScheme::setOption(unsigned int options, bool value)
{
	if (value) {
		mOptions |= options;
	}
	else {
		mOptions &= (~options);
	}
}


Vec3 ObstacleColorScheme::makeColor(unsigned int id)
{
	flt fMax = 255.0f;
	unsigned int hr = hash(id),
	             hg = hash(hr),
	             hb = hash(hg),
	             iMax = fMax + 1;

	return Vec3((hr % iMax) / fMax,
	            (hg % iMax) / fMax,
	            (hb % iMax) / fMax);
}


unsigned int ObstacleColorScheme::hash(unsigned int key)
{
	key = (key + 0x7ed55d16) + (key << 12);
	key = (key ^ 0xc761c23c) ^ (key >> 19);
	key = (key + 0x165667b1) + (key << 5);
	key = (key + 0xd3a2646c) ^ (key << 9);
	key = (key + 0xfd7046c5) + (key << 3);
	key = (key ^ 0xb55a4f09) ^ (key >> 16);

	return key;

}
