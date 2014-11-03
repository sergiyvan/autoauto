#pragma once
#include <math/Types.h>
#include <boost/array.hpp>

/** Wurde von mir erweitert, ist aber kompatibel mit den Werten aus der Ibeo ECU
 * - Arwed
 */
enum ObjectClass {
	UNKNOWN_SMALL = 0,	// pole, pedestrian or bike or other small static or dynamic obstacle
	PERSON = 1,
	CAR = 2,
	TRUCK = 3,
	BIKE = 4,
	UNKNOWN_BIG = 5,		// either small car or truck
	SMALL_CAR = 6,
	UNKNOWN_MEDIUM = 7,	// car or bike or static obstacle
	ENVIRONMENT = 8,		// tree, house, fence, or big static structures
	UNKNOWN_HUGE = 9,		// dynamic object, bigger than truck
	UNCLASSIFIED = 15		// no estimate yet
};

/** Hindernisse im Datenformat von Ibeo (der Alasca ECU).
 * TODO: Kann eigentlich komplett geändert werden, wird nicht in Logs verwendet
 */
class ObstacleData
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	enum Location {
		LOCATION_UNKOWN = 0,
		ON_LANE = 1,
		NEAR_LANE = 2,
		OFF_LANE = 3,
		IN_ZONE = 4
	};

	enum Movement {
		MOVEMENT_UNKNOWN = 0,
		BEING_OBSERVED,
		STATIC,
		DYNAMIC
	};
// 	uint uMovementClassification;

	enum SIZES {
		MAX_CONTOURPOINTS = 200, //80, // 24
		MAX_OBJECTS = 96
	};


	typedef boost::array<Vec2, MAX_CONTOURPOINTS> ContourPoints;
	typedef boost::array<Vec3, 4> BasePoints;

	ObstacleData()
		: m_noOfObjects(0) {
		/*		uMovementClassification = NOT_KNOWN;*/
	}

	virtual ~ObstacleData()
	{}

	uint getNoOfObjects() const {
		return m_noOfObjects;
	}

	uint getNoOfPoints(uint objectIndex) const {
		return m_noOfPoints[objectIndex];
	}

	ObjectClass getObjectClass(uint objectIndex) const {
		return m_objectClass[objectIndex];
	}

	Location getLocation(uint objectIndex) const {
		return m_objectLocation[objectIndex];
	}

	uint getObjectId(uint objectIndex) const {
		return m_objectId[objectIndex];
	}

	flt getObjectConfidence(uint objectIndex) const {
		return m_objectConfidence[objectIndex];
	}

	ContourPoints const & getPoints(uint objectIndex) const {
		return m_contourPoints[objectIndex];
	}

	BasePoints const & getBasePoints(uint objectIndex) const {
		return m_basePoints[objectIndex];
	}

	Vec2 const & getVelocity(uint objectIndex) const {
		return m_velocity[objectIndex];
	}

	Vec2 const & getAcceleration(uint objectIndex) const {
		return m_acceleration[objectIndex];
	}

	flt getHeight(uint objectIndex) const {
		return m_height[objectIndex];
	}

	/** Zur Schnittpunktberechnung im Behavior: Trajektorien */
	Vec2 const & getCenterOfGravity(uint objectIndex) const {
		return m_centerOfGravity[objectIndex];
	}

	/** Zur Schnittpunktberechnung im Behavior: Vorabcheck auf Kollisionsgefahr */
	flt getRadius(uint objectIndex) const {
		return m_radius[objectIndex];
	}

	/** Gibt an, ob sich ein Objekt im engeren Sichtwinkelbereich (voreingestellt: 150deg) befindet */
	bool inNarrowFOV(uint objectIndex) const;

	/** set number of objects contained
	 * \param	init_obstacles	set to true if the objects should be initialized with class and location UNKNOWN and zero
	 * velocity/acceleration
	 */
	virtual void setNoOfObjects(uint count, bool init_obstacles = true);
	void setObjectClass(uint objectIndex, ObjectClass objectClass);
	void setObjectId(uint objectIndex, uint objectId);

	void setContourPoints(uint objectIndex, Vec2 const * const contourpoints, uint nContourPoints);
	void setBasePoints(uint objectIndex, Vec3 const * const basepoints);
// 	void setPoints(uint objectIndex, Vec2 const * const points, uint noOfPoints);
// 	void setPoints(uint objectIndex, Vec2 const * const points, Vec3 const * const base, uint noOfPoints);

	void setVelocity(uint objectIndex, const Vec2 & velocity);
	void setAcceleration(uint objectIndex, const Vec2 & accel);
	void setHeight(uint objectIndex, flt height);
	void setLocation(uint objectIndex, Location location);
	void setObjectConfidence(uint objectIndex, flt conf);
	void setInNarrowFOV(uint objectIndex, bool inNarrowFOV);

	void setMovementClass(uint uObjectId, uint uMovementClass);
	uint getMovementClass(uint uObjectId);

	uint getSensorSources(uint objectIndex);
	void setSensorSources(uint objectIndex, uint sensorSources);
	bool isBuiltFromUnknownSource(uint objectIndex) const;
	bool isBuiltFromIbeo(uint objectIndex) const;
	bool isBuiltFromVelodyne(uint objectIndex) const;
	bool isBuiltFromCamera(uint objectIndex) const;

private:
	void calculateCenterOfGravity(uint objectIndex);
	void calculateRadius(uint objectIndex);
	uint m_noOfObjects;
	uint m_timstamp;
	boost::array<uint, MAX_OBJECTS> m_objectId;

	/** values < 0.5 will be filtered out
	 * this is not a solution for eternity
	 */
	boost::array<flt, MAX_OBJECTS> m_objectConfidence;
	boost::array<bool, MAX_OBJECTS> m_inNarrowFOV;

	boost::array<uint, MAX_OBJECTS> m_objectMovementClass;

	boost::array<ObjectClass, MAX_OBJECTS> m_objectClass;
	boost::array<Location, MAX_OBJECTS> m_objectLocation;

	boost::array<uint, MAX_OBJECTS> m_noOfPoints;
	boost::array< ContourPoints, MAX_OBJECTS > m_contourPoints;
	boost::array< BasePoints, MAX_OBJECTS > m_basePoints;

	boost::array<Vec2, MAX_OBJECTS> m_velocity;
	boost::array<Vec2, MAX_OBJECTS> m_acceleration;
	boost::array<Vec2, MAX_OBJECTS > m_centerOfGravity;
	boost::array<flt, MAX_OBJECTS > m_radius;
	boost::array<flt, MAX_OBJECTS> m_height;

	/// Describes which sensors were used to build this obstacle
	boost::array<unsigned int, MAX_OBJECTS> m_sensorSources;
};

#include <rtt/Port.hpp>
#include <util/PooledObjectTemplate.h>
TIMEDPOOLEDOBJECT(ObstacleData, 16);

namespace RTT
{
extern template class InputPort<TimedObstacleData_ptr>;
extern template class OutputPort<TimedObstacleData_ptr>;
}
