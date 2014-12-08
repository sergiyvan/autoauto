#pragma once
#include <util/RtTaskContext.h>
// #include <data/ObstacleData.h>
#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <aa/modules/io/passat/PassatCanMessages.h>

#include <modules/models/egostate/EgoStateSetter.h>
#include <modules/models/carstate/CarState.h>
#include <aa/modules/models/carstate/PassatCarState.h>
#include <modules/models/carstate/AuxDevicesData.h>

#include <aa/modules/models/rndf/RndfGraph.h>

#include "WheelData.h"

typedef TimedData< PlainDataHolder< ::math::flt> > TimedDouble;

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class Simulator
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Vec4 Vec4;
	typedef ::math::Mat4x4 Mat4x4;

	explicit Simulator(std::string const & name);
	virtual ~Simulator();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void errorHook();

	void initTrafficLights();

	bool setTrafficLightState(int trafficLightID, int state);

	bool setTrafficLights(std::string ids, std::string states);

protected:


    RTT::OutputPort< ::modules::models::carstate::TimedCarState> mCarStateOut;
    RTT::OutputPort< aa::modules::models::carstate::TimedPassatCarState> mPassatCarStateOut;

	RTT::OutputPort<std::vector<std::pair<Vec3, Vec3> > > mVectorsOut;
	RTT::OutputPort<std::vector<boost::tuple<Mat4x4, std::string, Vec4> > > mObjectsOut;
	RTT::OutputPort<TimedBaseObstacleBundle_ptr> mObstaclesOut;

    RTT::InputPort< ::modules::models::egostate::EgoStateSetter > mEgoStateSetterIn;

	RTT::InputPort<TimedDouble> mSpeedIn;
	RTT::InputPort<flt> mSteerIn;
	RTT::InputPort<int> mGearIn;
	RTT::InputPort<AuxDevicesData> mAuxDevicesIn;

	RTT::InputPort<bool> mResetNewScenarioIn;
	RTT::InputPort<std::string> mSetToWayPointIn;


    RTT::Property<bool> mPause;
    RTT::Property<bool> mGenerateHiddenObstaclePoints;

	void tokenize(const std::string & str, std::vector<std::string>& tokens, const std::string & delimiters);
	void setPosAndDirAtWaypoint(std::string const & wp);
	void setPosDirVec(std::string const & v);
	void setPosition(flt x, flt y, flt z);
	void setDirection(flt roll, flt yaw, flt pitch);
	void setVelocity(flt x, flt y, flt z);
	void setSteer(flt a);
	void reset();
	bool togglePause();

private:
	int mSimulationObjectId;

    std::vector< aa::modules::models::rndf::vertex_descr> trafficlights;
    boost::property_map< aa::modules::models::rndf::aGraph, aa::modules::models::rndf::vertex_data_t>::type vertexDataMap;
};

} // namespace modules
} // namespace nav
} // namespace simulator
}
