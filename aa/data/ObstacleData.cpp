#include "ObstacleData.h"
#include "obstacle/Obstacle.h"

using namespace aa::data::obstacle::util;

void ObstacleData::setNoOfObjects(uint count, bool init_obstacles)
{
	m_noOfObjects = count;

	if (!init_obstacles) {
		return;
	}

	for (uint i = 0; i < count && i < MAX_OBJECTS; i++) {
		m_noOfPoints[i] = 0;
		m_velocity[i][0] = 0.;
		m_velocity[i][1] = 0.;
		m_acceleration[i][0] = 0.;
		m_acceleration[i][1] = 0.;
		m_objectClass[i] = UNCLASSIFIED;
		m_objectLocation[i] = LOCATION_UNKOWN;
		m_inNarrowFOV[i] = true;
		m_sensorSources[i] = SENSOR_TYPE_UNKNOWN;
	}
}

/*void ObstacleData::setNoOfPoints(uint objectIndex, uint count)
{
	m_noOfPoints[objectIndex] = count;
}*/

void ObstacleData::setObjectClass(uint objectIndex, ObjectClass objectClass)
{
	m_objectClass[objectIndex] = objectClass;
}


void ObstacleData::setMovementClass(uint uObjectId, uint uMovementClass)
{
	for (uint i = 0; i < getNoOfObjects(); i++) {
		if (getObjectId(i) == uObjectId) {
			m_objectMovementClass[i] = uMovementClass;
		}
	}
}

uint ObstacleData::getMovementClass(uint uObjectId)
{
	for (uint i = 0; i < getNoOfObjects(); i++) {
		if (getObjectId(i) == uObjectId) {
			return m_objectMovementClass[i];
		}
	}

	return MOVEMENT_UNKNOWN;
}

void ObstacleData::setSensorSources(uint objectIndex, uint sensorSources)
{
	m_sensorSources[objectIndex] = sensorSources;
}

uint ObstacleData::getSensorSources(uint objectIndex)
{
	return m_sensorSources[objectIndex];
}

bool ObstacleData::isBuiltFromUnknownSource(uint objectIndex) const
{
	return m_sensorSources[objectIndex] == SENSOR_TYPE_UNKNOWN;
}

bool ObstacleData::isBuiltFromIbeo(uint objectIndex) const
{
	return m_sensorSources[objectIndex] & SENSOR_TYPE_IBEO;
}

bool ObstacleData::isBuiltFromVelodyne(uint objectIndex) const
{
	return m_sensorSources[objectIndex] & SENSOR_TYPE_VELODYNE;
}

bool ObstacleData::isBuiltFromCamera(uint objectIndex) const
{
	return m_sensorSources[objectIndex] & SENSOR_TYPE_CAMERA;
}

void ObstacleData::setLocation(uint objectIndex, ObstacleData::Location objectClass)
{
	m_objectLocation[objectIndex] = objectClass;
}

void ObstacleData::setObjectId(uint objectIndex, uint objectId)
{
	m_objectId[objectIndex] = objectId;
}

void ObstacleData::setObjectConfidence(uint objectIndex, flt conf)
{
	m_objectConfidence[objectIndex] = conf;
}

// void ObstacleData::setPoints(uint objectIndex, Vec2 const * const points, uint noOfPoints)
// {
// 	for (uint i=0; i<noOfPoints && i < MAX_CONTOURPOINTS; i++){
// 		m_contourPoints[objectIndex][i]=points[i];
// 		m_basePoints[objectIndex][i] = Vec3(points[i][0], points[i][1], 0.0);
// 	}
//
// 	m_noOfPoints[objectIndex] = noOfPoints;
//
// 	calculateCenterOfGravity(objectIndex);
// 	calculateRadius(objectIndex);
//
// }
//
// void ObstacleData::setPoints(uint objectIndex,
// 							 Vec2 const * const points,
// 							 Vec3 const * const base,
// 							 uint noOfPoints)
// {
// 	assert(noOfPoints == 4);
//
// 	for (uint i=0; i<noOfPoints && i < MAX_CONTOURPOINTS; i++){
// 		m_contourPoints[objectIndex][i]=points[i];
// 		m_basePoints[objectIndex][i] = base[i];
// 		std::cout << i << ": " << base[i][0] << "," << base[i][1] << "," << base[i][2] << std::endl;
// 	}
//
// 	m_noOfPoints[objectIndex] = noOfPoints;
//
// 	calculateCenterOfGravity(objectIndex);
// 	calculateRadius(objectIndex);
//
// }

void ObstacleData::setContourPoints(uint objectIndex,
									Vec2 const * const contourpoints,
									uint nContourPoints)
{
	if (nContourPoints > MAX_CONTOURPOINTS) {
		RTT::Logger::log()
				<< RTT::Logger::Warning
				<< "Number of cotourpoints is too big, some points will be dropped."
				<< RTT::Logger::endl;
	}

	for (uint i = 0; i < nContourPoints && i < MAX_CONTOURPOINTS; i++) {
		m_contourPoints[objectIndex][i] = contourpoints[i];
	}

	m_noOfPoints[objectIndex] = nContourPoints;

	calculateCenterOfGravity(objectIndex);
	calculateRadius(objectIndex);

}

void ObstacleData::setBasePoints(uint objectIndex,
								 Vec3 const * const basepoints)
{
	for (uint i = 0; i < 4; i++) {
		m_basePoints[objectIndex][i] = basepoints[i];
	}
}

void ObstacleData::setVelocity(uint objectIndex, const Vec2 & velocity)
{
	m_velocity[objectIndex] = velocity;
}

void ObstacleData::setAcceleration(uint objectIndex, const Vec2 & accel)
{
	m_acceleration[objectIndex] = accel;
}

void ObstacleData::setHeight(uint objectIndex, flt height)
{
	m_height[objectIndex] = height;
}

void ObstacleData::setInNarrowFOV(uint _objectIndex, bool _inNarrowFOV)
{
	m_inNarrowFOV[_objectIndex] = _inNarrowFOV;
}


void ObstacleData::calculateCenterOfGravity(uint objectIndex)
{
	m_centerOfGravity[objectIndex].fill(0);

	for (uint i = 0; i < m_noOfPoints[objectIndex]; i++) {
		m_centerOfGravity[objectIndex] += m_contourPoints[objectIndex][i];
	}

	m_centerOfGravity[objectIndex] /= (flt)m_noOfPoints[objectIndex];
}

void ObstacleData::calculateRadius(uint objectIndex)
{
	Vec2 const & center = getCenterOfGravity(objectIndex);

	flt maxDist = 0.f;


	for (uint i = 0; i < m_noOfPoints[objectIndex]; i++) {
		flt dist = (m_contourPoints[objectIndex][i] - center).squaredNorm();

		if (dist > maxDist) {
			maxDist = dist;
		}
	}

	m_radius[objectIndex] = sqrt(maxDist);
}

bool ObstacleData::inNarrowFOV(uint objectIndex) const
{
	return m_inNarrowFOV[objectIndex];
}

namespace RTT
{
template class InputPort<TimedObstacleData_ptr>;
template class OutputPort<TimedObstacleData_ptr>;
}
