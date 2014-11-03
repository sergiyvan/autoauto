/**
 *  \author Sebastian Hempel
 *  \brief Color schemes for obstacles
 */

#pragma once
#include <qmutex.h>
#include <aa/data/obstacle/Obstacle.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace obstacles
{

/**
 *  \brief render options
 */
enum OBSTACLE_RENDER_OPTION {
	RENDER_BOUNDING_BOX = 1,
	RENDER_SOLID_BOX = 2,
	RENDER_ID = 4,
	RENDER_CENTER = 8,
	RENDER_ORIENTATION = 16,
	RENDER_CONTOUR = 32
};


enum SENSOR {
	SENSOR_UNKNOWN = 0,
	SENSOR_VELODYNE = 1,
	SENSOR_ALASCA = 2,
	SENSOR_LUX = 4				//SENSOR_LUX_0, ... SENSOR_LUX_k ?
};


/**
 *  \brief a color configuration for an obstacle
 */
struct ColorConfiguration {
	typedef ::math::Vec3 Vec3;
	//colors
	Vec3 mBoundingBoxColor;
	Vec3 mSolidTopColor;
	Vec3 mSolidBottomColor;
	Vec3 mOrientationFirstColor;
	Vec3 mOrientationSecondColor;
	Vec3 mIdColor;
	Vec3 mCenterColor;
	Vec3 mContourColor;

	//options
	bool mRenderBoundingBox;
	bool mRenderSolid;
	bool mRenderOrientation;
	bool mRenderId;
	bool mRenderCenter;
	bool mRenderContourPoints;
	math::flt mContourWidth;
};


/**
*  \brief Color schemes for obstacles
 */
class ObstacleColorScheme
{
public:
	typedef ::math::Vec3 Vec3;
	/**
	 *  \brief default scheme
	 */
	ObstacleColorScheme();


	/**DataObstacle
	 *  \param boxes render bounding boxes?
	 *  \param solid render solid boxes?
	 *  \param ids render ids?
	 *  \param obstacles render center?
	 *  \param orientations render orientations?
	 */
	ObstacleColorScheme(bool boxes, bool solid, bool ids,
	                    bool center, bool orientations, bool contour);


	/**
	 *  \brief creates scheme by used sensors
	 *  \param sensors bitmask of used sensors
	 */
	ObstacleColorScheme(unsigned int sensors);

	bool checkBoundingBox()
	{
		return check(RENDER_BOUNDING_BOX);
	}
	bool checkSolidBox()
	{
		return check(RENDER_SOLID_BOX);
	}
	bool checkId()
	{
		return check(RENDER_ID);
	}
	bool checkCenter()
	{
		return check(RENDER_CENTER);
	}
	bool checkOrientation()
	{
		return check(RENDER_ORIENTATION);
	}

	/**
	 *  \brief creates color configuration of a tracked obstacle
	 *  \param obstacle obstacle to create configuration for
	 *  \return configuration for the given obstacle
	 */
	ColorConfiguration make(const aa::data::obstacle::Obstacle & obstacle);

	/**
	 *  \brief is this a default scheme or is it modified?
	 *  \return true if default false otherwise
	 */
	bool isDefaultScheme()
	{
		return mDefaultScheme;
	}


protected:
	/**
	 *  render options
	 */
	unsigned int mOptions;


	/**
	 *  used sensors
	 */
	unsigned int mSensors;


	/**
	 *  is this the default scheme?
	 */
	bool mDefaultScheme;


	/**
	 *  \brief checks an option
	 *  \param option option to check
	 *  \return true if option is set false otherwise
	 */
	bool check(OBSTACLE_RENDER_OPTION option);


	/**
	 *  default initialisation
	 */
	void init();


	virtual Vec3 boundingBoxColor(bool tracked, unsigned int id);
	virtual Vec3 solidTopColor(bool tracked, unsigned int id);
	virtual Vec3 solidBottomColor(bool tracked, unsigned int id);
	virtual Vec3 orientationFirstColor(bool tracked,
	                                   unsigned int id);
	virtual Vec3 orientationSecondColor(bool tracked,
	                                    unsigned int id);
	virtual Vec3 idColor(bool tracked, unsigned int id);
	virtual Vec3 centerColor(bool tracked, unsigned int id);
	virtual Vec3 contourColor(bool tracked, unsigned int id);

	/**
	 *  \brief sets render options
	 */
	void setOptions(ColorConfiguration * config);


	/**
	 *  \brief setting and disabling of options
	 *  \param options options to set
	 *  \param value value of option
	 */
	void setOption(unsigned int options, bool value);

	/**
	 *  \brief creates color configuration
	 *  \param tracked is obstacle tracked?
	 *  \param id id of the obstacle
	 *  \return configuration
	 */
	ColorConfiguration make(bool tracked,
	                        unsigned int id);


	//properties of the current obstacle

	//default colors
	static const Vec3 DEFAULT_BOUNDING_BOX_COLOR,
	       DEFAULT_SOLID_TOP_COLOR,
	       DEFAULT_SOLID_BOTTOM_COLOR,
	       DEFAULT_ORIENTATION_FIRST_COLOR,
	       DEFAULT_ORIENTATION_SECOND_COLOR,
	       DEFAULT_ID_COLOR,
	       DEFAULT_CENTER_COLOR,
	       DEFAULT_CONTOUR_COLOR;


	/**
	 *  \brief creates a color based on the id of an obstacle
	 *  \param id id of obsacle
	 *  \return color of obstacle
	 */
	Vec3 makeColor(unsigned int id);


	/**
	 *  \brief a simple hash function
	 *         http://www.concentric.net/~Ttwang/tech/inthash.htm
	 *  \param key key to hash
	 *  \return hashed value
	 */
	unsigned int hash(unsigned int key);
};

}
}
}
}
