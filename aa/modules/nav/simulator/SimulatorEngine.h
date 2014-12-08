#pragma once

#include <vector>
#include <math/Types.h>

#include <boost/noncopyable.hpp>

// #include <data/ObstacleData.h>
#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <modules/models/carstate/CarState.h>
#include <modules/models/carstate/AuxDevicesData.h>

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class SimpleBackend;
class Simulant;

class SimulatorEngine
	: boost::noncopyable
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec3 Vec3;
	typedef MATH_TYPES_VEC(flt, 4) Vec4;
	typedef ::math::Quaternion Quaternion;

	SimulatorEngine();
	~SimulatorEngine();

	flt getCurrentFrameTime() const;
	void update();

	// Interface
	// register object with nosimulation characteristics, you can change position and it will be pushed, but nothing else
	// control funtion does nothing on those. fixed objects are immovable
	unsigned int registerObject(bool fixed, Vec3 const & pos, Vec3 const & size = Vec3(1, 1, 1),
#if defined(USE_EIGEN)
								Quaternion const & orientation = Quaternion(0, 0, 0, 1),
#else
								Quaternion const & orientation = Quaternion(0, 0, 1, 0),
#endif
								std::string const & model = "",
								Vec3 const & modelscale = Vec3(1, 1, 1), Vec4 const & mat = Vec4(0.5, 0.5, 0.5, 0.2), std::string const & name = "");

	// a fully simulated car, only here the settings for gear and so on do anything usefull
	unsigned int registerAuto(Vec3 const & pos, Quaternion const & orientation = Quaternion(0, 0, 1, 0));

	void destroyObject(unsigned int objectId);

	void pause(bool p = true);

	// set control commands for the car
	void control(unsigned int id, int gear, flt speed, flt brake, flt steer);
	void setPosition(unsigned int id, Vec3 const & pos, bool force = false);
	void setOrientation(unsigned int id, Quaternion const & ori, bool force = false);
	void setVelocity(unsigned int id, Vec3 const & vel, bool force = false);
	void setSteer(unsigned int id, flt const & steer, bool force = false);
	void setAcceleration(unsigned int id, Vec3 const & acc, bool force = false);
	void setAuxDevicesData(unsigned int id, AuxDevicesData const & auxData);

	Vec3 getPosition(unsigned int id) const;
	Vec3 getMovementDirection(unsigned int id) const;
	Quaternion getOrientation(unsigned int id) const;
	Vec3 getVelocity(unsigned int id) const;
	Vec3 getAcceleration(unsigned int id) const;
	flt getSpeed(unsigned int id) const;
	flt getSteer(unsigned int id) const;
	int getGear(unsigned int id) const;
	flt getYawRate(unsigned int id) const;
	std::string getInfos(unsigned int id) const;

	void generateObstacles(unsigned int id);

	void setTrafficLightState();

	void draw();

// 	TimedObstacleData_ptr getObstacles(unsigned int id) const;
	TimedBaseObstacleBundle_ptr getObstacles(unsigned int id) const;

//	static RTT::Property<flt>&	mFrontShaftDistance;
//	static RTT::Property<flt>&	mRearShaftDistance;
//	static RTT::Property<flt>&	mShaftDistance;
//	static RTT::Property<flt>&	mTireDistance;
//	static RTT::Property<flt>&	mMaxSpeedChange;
//	static RTT::Property<flt>&	mMaxBreakChange;
//	static RTT::Property<flt>&	mMaxSteerChange;
//	static RTT::Property<flt>&	mMaxSpeed;
//	static RTT::Property<flt>&	mMaxSteer;
//	static RTT::Property<flt>&	mMaxReverseSpeed;
//	static RTT::Property<flt>&	mTireStiffness;
//	static RTT::Property<flt>&	mTireRadius;
	typedef std::vector<Simulant *> objects_collection_type;
	objects_collection_type objects;

    bool mGenerateHiddenObstaclePoints;

protected:
// 	RTT::OutputPort<std::vector<boost::tuple<Mat4x4, std::string, Vec4> > > mObjects;

private:
	flt currentFrameTime;
	TimeStamp lastTimeStamp;
	SimpleBackend * backend;
};

} // namespace modules
} // namespace nav
} // namespace simulator
}
