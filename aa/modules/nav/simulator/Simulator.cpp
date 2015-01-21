#include "Simulator.h"

#include <iostream>
#include <set>
#include <algorithm>
#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/trim.hpp>


#include <math/AutoMath.h>
#include <math/Geodetic.h>
#include <util/TaskContextFactory.h>
#include <data/VehicleData.h>
#include <patterns/Singleton.h>
#include "SimulatorEngine.h"
#include "Simulator.h"

#include <aa/modules/io/passat/PassatCanMessages.h>

#include <data/VehicleData.h>

using namespace std;
using namespace RTT;
using namespace boost;
using namespace ::math;
using namespace util;
using namespace aa::modules::nav::simulator;
using namespace aa::modules::io::passat;
using namespace modules::models::carstate;
using namespace aa::modules::models::rndf;

REGISTERTASKCONTEXT(Simulator);



typedef patterns::Singleton<SimulatorEngine> theSimulator;

/**
 * \brief sets up propertys, ports and methods for orocos
 * \param name
 */
Simulator::Simulator(string const & name)
	: util::RtTaskContext(name)

	, mCarStateOut("CarStateOut")
	, mPassatCarStateOut("PassatCarStateOut")

	, mVectorsOut("VectorsOut")
	, mObjectsOut("ObjectsOut")
    , mObstaclesOut("ObstaclesOut")
    , mOdometricDataOut("OdometricDataOut")

	, mEgoStateSetterIn("EgoStateSetter")
	, mSpeedIn("SpeedIn")
	, mSteerIn("SteerIn")
	, mGearIn("GearIn")
	, mAuxDevicesIn("AuxDevicesIn")

	, mResetNewScenarioIn("ResetNewScenarioIn")
	, mSetToWayPointIn("SetToWayPointIn")

    , mPause("Pause", "Pause the simulator", false)
    , mGenerateHiddenObstaclePoints("GenerateHiddenObstaclePoints", "if true the simulator generates full obstacles contours", false)
    , mObstaclePosStdDev("ObstaclePosStdDev", "standard deviation of outgoing obstacle positions (fake sensor noise)", 0.0)
    , mObstacleVelStdDev("ObstacleVelStdDev", "standard deviation of outgoing obstacle velocities (fake sensor noise)", 0.0)
    , mOdometryDistStdDev("OdometryDistStdDev", "standard deviation of outgoing odometry distance (fake sensor noise)", 0.0)
    , mOdometryAngleStdDev("OdometryAngleStdDev", "standard deviation of outgoing odometry distance (fake sensor noise)", 0.0)

	, mSimulationObjectId(theSimulator::instance().registerAuto(Vec3(0, 0, 0),
#if defined(USE_EIGEN)
						  Quaternion(0, 0, 0, 1)
#else
						  Quaternion(0, 0, 1, 0)
#endif
															   ))
    , mLastEgoPos(0,0,0)
    , mLastEgoAngle(0)
{
	setDirection(0.0, 0.0, 0.0);

	addPort(mCarStateOut);
	addPort(mPassatCarStateOut);

	addPort(mVectorsOut);
	addPort(mObjectsOut);
    addPort(mObstaclesOut);
    addPort(mOdometricDataOut);

	addPort(mEgoStateSetterIn);

	addPort(mSpeedIn);
	addPort(mSteerIn);
	addPort(mGearIn);
	addPort(mAuxDevicesIn);

	addPort(mResetNewScenarioIn);
	addPort(mSetToWayPointIn);


    addProperty(mPause);
    addProperty(mGenerateHiddenObstaclePoints);
    addProperty(mObstaclePosStdDev);
    addProperty(mObstacleVelStdDev);
    addProperty(mOdometryDistStdDev);
    addProperty(mOdometryAngleStdDev);

    // Method Factory Interface.
	addOperation("initTrafficLights", &Simulator::initTrafficLights, this, RTT::ClientThread).doc("Search for all Trafficlights on the RNDF");
	addOperation("setTrafficLightState", &Simulator::setTrafficLightState, this, RTT::ClientThread).doc("Set state of one certain trafficlight").arg("trafficLightID", "").arg("state", "");
	addOperation("setTrafficLights", &Simulator::setTrafficLights, this, RTT::ClientThread).doc("Sets a group of Trafficlights (e.g. useful at crossings)").arg("ids", "IDs").arg("states", "states of trafficlights");

	addOperation("setPosAndDirAtWaypoint", &Simulator::setPosAndDirAtWaypoint, this, RTT::ClientThread).doc("Sets the position and orientation of the car at a specific waypoint").arg("waypoint", "waypoint");

	addOperation("setPosDir", &Simulator::setPosDirVec, this, RTT::ClientThread).doc("Sets the position and orientation of the car").arg("v", "xyzrpy vec");
	addOperation("setPosition", &Simulator::setPosition, this, RTT::ClientThread).doc("Sets the position of the car").arg("x", "X").arg("y", "Y").arg("z", "Z");
	addOperation("setDirection", &Simulator::setDirection, this, RTT::ClientThread).doc("Sets the orientation of the car").arg("r", "roll").arg("p", "pitch").arg("y", "yaw");

	addOperation("setVelocity", &Simulator::setVelocity, this, RTT::ClientThread).doc("Sets the velocity of the car").arg("x", "Vel_X").arg("y", "Vel_Y").arg("z", "Vel_Z");
	addOperation("setSteer", &Simulator::setSteer, this, RTT::ClientThread).doc("Sets the steer angle at the wheel").arg("a", "A");
	addOperation("reset", &Simulator::reset, this, RTT::ClientThread).doc("reset car.");
	addOperation("togglePause", &Simulator::togglePause, this, RTT::ClientThread).doc("toggles the pause state");
	/*
		addOperation("setToWaypoint", &Simulator::setToWaypoint, this, RTT::ClientThread).doc("set To Waypoint.").arg("Waypoint ID", "Waypoint ID");
		addOperation("setSpeed", &Simulator::setSpeed, this, RTT::ClientThread).doc("set Speed.").arg("Speed value", "Speed value");
		addOperation("setSteer", &Simulator::setSteer, this, RTT::ClientThread).doc("set Steering.").arg("Steering value", "Steering value");
		addOperation("setGear", &Simulator::setGear, this, RTT::ClientThread).doc("set Gear.").arg("Gear value", "Gear value");

		addOperation("addObstacle", &Simulator::addObstacle, this, RTT::ClientThread).doc("add Obstacle.").arg("POS", "POS").arg("SIZE", "SIZE").arg("DIR", "DIR");
	// 	addOperation("addObstacle2", &Simulator::addObstacle2, this, RTT::ClientThread).doc("add Obstacle.").arg("X", "X").arg("Y", "Y").arg("Z", "Z").arg("SX", "SX").arg("SY", "SY").arg("SZ", "SZ").arg("DX", "DX").arg("DY", "DY").arg("DZ", "DZ");
		addOperation("changeObstacle", &Simulator::changeObstacle, this, RTT::ClientThread).doc("change Obstacle.").arg("ID", "ID").arg("POS", "POS").arg("SIZE", "SIZE").arg("DIR", "DIR");
	*/

}

Simulator::~Simulator()
{}

void Simulator::tokenize(const string & str, vector<string>& tokens, const string & delimiters)
{
	// Skip delimiters at beginning.
	string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	string::size_type pos     = str.find_first_of(delimiters, lastPos);

	while (string::npos != pos || string::npos != lastPos) {
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

/**
 *
 */
void Simulator::initTrafficLights()
{
	RNDFGraph const & mRndfGraph = patterns::Singleton<RNDFGraph>::instance();
	aGraph & graph = const_cast<aGraph &>(mRndfGraph.getBoostGraph());

	vertexDataMap = get(vertex_data_t(), graph);

	int i = 0;
	int j = 0;

	BOOST_FOREACH(vertex_descr v1, vertices(graph)) {
		VertexData & v1Data = vertexDataMap[v1];

		if (v1Data.vertexType.isSet(VertexData::TRAFFIC_LIGHT | VertexData::TRAFFIC_LIGHT_TWO_PHASES)) {
//			v1Data.metaData = VertexData::UNKNOWN;
			trafficlights.push_back(v1);
			i++;
		}

		j++;
	}

	Logger::log(Logger::Debug) << "Found " << i << " trafficlight(s) on " << j << " vertices!" << Logger::endl;
}

/**
 *
 * @param trafficLightID
 * @param state
 * @return
 */
bool Simulator::setTrafficLightState(int trafficLightID, int state)
{

	VertexData::TrafficSignalState tState = (VertexData::TrafficSignalState) state;

	if (trafficLightID < 0 || trafficLightID > trafficlights.size()) {
		Logger::log(Logger::Error) << "TrafficLightID is out of range." << Logger::endl;

		return false;
	}

	VertexData & v = vertexDataMap[trafficlights.at(trafficLightID)];
	v.metaData = tState;

	Logger::log(Logger::Debug) << "Trafficlight (id: " << trafficLightID << ") "
							   "was set to state: " << state  << "[0: Unknown, 1: Red, 2:RedYellow, 3:Yellow, 4:Green]" << Logger::endl;
	return true;
}

/**
 *
 * @param ids
 * @return
 */
bool Simulator::setTrafficLights(std::string ids, std::string states)
{
	vector<string> tIDs;
	vector<string> tStates;

	// parse inputs
	tokenize(ids, tIDs, ",");
	tokenize(states, tStates, ",");

	if (tStates.size() < 1 || tStates.size() != tIDs.size()) {
		return false;
	}

	for (int i = 0; i < tStates.size(); ++i) {
		setTrafficLightState(atoi(tIDs.at(i).c_str()), atoi(tStates.at(i).c_str()));
	}

	return true;
}


/**
 * resets simulated object to position of a certain waypoint.
 * set direction to driving direction of the chosen lane
 * @param waypoint
 */
void Simulator::setPosAndDirAtWaypoint(std::string const & waypoint)
{
	Logger::In in("Simulator");

	RNDFGraph & rndfgraph = patterns::Singleton<RNDFGraph>::instance();
	aGraph const & rGraph = rndfgraph.getBoostGraph();
	boost::property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);

	vector<string> wpSegments;
	tokenize(waypoint, wpSegments, ".");

	// handle incorrect waypoint structure
	if (wpSegments.size() != 3) {
		Logger::log() << Logger::Error << "The given waypoint signature is invalid. RNDF structure is Segment.Lane.WayPoint" << Logger::endl;
		return;
	}

	Logger::log() << Logger::Debug << "Setting auto at waypoint " << atoi(wpSegments[0].c_str()) << "." << atoi(wpSegments[1].c_str()) << "." << atoi(wpSegments[2].c_str()) << Logger::endl;
	vertex_descr wp = rndfgraph.getVertex(atoi(wpSegments[0].c_str()), atoi(wpSegments[1].c_str()), atoi(wpSegments[2].c_str()));

	if (boost::graph_traits<aGraph>::null_vertex() == wp) {
		Logger::log() << Logger::Error << "Found no waypoint named " << waypoint << " in RNDF!" << Logger::endl;
		return;
	}

	Vec2 wppos = vertexDataMap[wp].pos;
	setPosition(wppos(0), wppos(1), 0.f);

	Vec2 dir;
	flt yaw = 0.f;

	if (!rndfgraph.trackDir(dir, wp)) {
		Logger::log() << Logger::Error << "Couldn't get track direction at waypoint " << waypoint << " in RNDF!" << Logger::endl;
	}
	else {
		yaw = atan2(dir(1), dir(0));
	}

	setDirection(0, 0, yaw);
	RTT::log(RTT::Debug) << "Set pose to " << wppos << " @ " << yaw * ::math::R2D << "°" << Logger::endl;
}


void Simulator::setPosDirVec(std::string const & v)
{
	Logger::In in("Simulator");

	vector<string> v_tokenized;
	vector<flt> v_coords;
	tokenizer< char_separator<char> > tok(v, char_separator<char>(","));

	v_tokenized.assign(tok.begin(), tok.end());


	for (vector<string>::iterator it = v_tokenized.begin(); it != v_tokenized.end(); ++it) {
		trim(*it);
		v_coords.push_back(strtod(it->c_str(), NULL));
	}

	if (v_coords.size() != 6) {
		Logger::log() << Logger::Error << "The given position/direction vector is invalid." << Logger::endl;
		return;
	}

	setPosition(v_coords[0], v_coords[1], v_coords[2]);
	setDirection(v_coords[3], v_coords[4], v_coords[5]);
}

void Simulator::setPosition(flt x, flt y, flt z)
{
	Logger::In in("Simulator");
	theSimulator::instance().setPosition(mSimulationObjectId, Vec3(flt(x), flt(y), flt(z)), true);
}

void Simulator::setDirection(flt roll, flt pitch, flt yaw)
{
	Logger::In in("Simulator");

	if (0.0 == roll && 0.0 == pitch && 0.0 == yaw) {
		yaw = 1e-6;
	}

#if defined(USE_EIGEN)
	theSimulator::instance().setOrientation(mSimulationObjectId, fromEulerXyz(roll, pitch, yaw), true);
#else
	theSimulator::instance().setOrientation(mSimulationObjectId, Quaternion(flt(roll), flt(pitch), flt(yaw)), true);
#endif
}

void Simulator::setVelocity(flt x, flt y, flt z)
{
	Logger::In in("Simulator");
	theSimulator::instance().setVelocity(mSimulationObjectId, Vec3(flt(x), flt(y), flt(z)), true);
}

void Simulator::setSteer(flt a)
{
	Logger::In in("Simulator");
	theSimulator::instance().setSteer(mSimulationObjectId, a, true);
}

void Simulator::reset()
{
	Logger::In in("Simulator");

	setPosition(0.0, 0.0, 0.0);
	setDirection(0.0, 0.0, 0.0);
	setVelocity(0.0, 0.0, 0.0);
	setSteer(0.0);
    mLastEgoPos = Vec3(0,0,0);
    mLastEgoAngle = 0.0;
}


/**
 * \brief create some basic information for the vehicle model and setup ODE
 */
bool Simulator::startHook()
{
	Logger::In in("Simulator");
    REQUIRED_PORTS((mSpeedIn)(mSteerIn)(mGearIn));

	return true;
}

/**
 * \brief get port inputs (Steer, Speed) and compute the new state of the carmodel. Finally set output ports.
 */
void Simulator::updateHook()
{
	Logger::In in("Simulator");

	bool resetNewScenario = false;
	mResetNewScenarioIn.read(resetNewScenario);

	if (resetNewScenario) {
		reset();
		std::string setToWayPoint;
		mSetToWayPointIn.read(setToWayPoint);
		setPosAndDirAtWaypoint(setToWayPoint);
		//setPosAndDirAtWaypoint()
	}

    if (mPause.get()) {
        theSimulator::instance().pause();
        return;
    }

    if (mSimulationObjectId < 0) {
		return;
	}

    theSimulator::instance().mGenerateHiddenObstaclePoints=mGenerateHiddenObstaclePoints.get();

	TimeStamp now;
	now.stamp();

	flt acceleration(0.0f);
// 	bool reverse = false;

    flt const maxSteer = ::data::theVehicleData::instance().getPropertyType<flt>("maxSteer")->value();

	TimedDouble speedIn;
	int gear = CarState::GEAR_PARK;
	flt steerIn = 0.0;

	if (RTT::NoData == mSpeedIn.read(speedIn)
			|| RTT::NoData == mGearIn.read(gear)
			|| RTT::NoData == mSteerIn.read(steerIn)
            || std::isnan(speedIn.data)
            || std::isnan(steerIn)) {
		// RTT::log(RTT::Debug) << "Not all ports have data" << RTT::endlog();
		theSimulator::instance().control(mSimulationObjectId, CarState::GEAR_PARK, 0.0, 0.0, 0.0);
		return;
	}

	// std::cout << "simul got speed: " << tspeed.data << std::endl;

	if (!CarState::isValidGear(gear)) {
		RTT::log(RTT::Error) << "Received invalid gear: " << gear << RTT::endlog();
		gear = CarState::GEAR_PARK;
	}

	// std::cout << "simul got gear: " << gear << std::endl;

	steerIn = -maxSteer * steerIn;
	theSimulator::instance().control(mSimulationObjectId, gear, max(flt(0), speedIn.data), -min(flt(0), speedIn.data), steerIn);

	AuxDevicesData aux;

	if (RTT::NewData == mAuxDevicesIn.read(aux)) {
		mAuxDevicesIn.read(aux);
		theSimulator::instance().setAuxDevicesData(mSimulationObjectId, aux);
	}

    theSimulator::instance().mObstaclePosStdDev=mObstaclePosStdDev.get();
    theSimulator::instance().mObstacleVelStdDev=mObstaclePosStdDev.get();
    theSimulator::instance().update();

	Vec3 const carPos3D = theSimulator::instance().getPosition(mSimulationObjectId);
	Vec3 const carMove3D = theSimulator::instance().getMovementDirection(mSimulationObjectId);
	flt const carYawRate = theSimulator::instance().getYawRate(mSimulationObjectId);
	flt const carSpeed = theSimulator::instance().getSpeed(mSimulationObjectId);

// 	std::cout << "simul has speed: " << carSpeed << std::endl;

	if (mEgoStateSetterIn.connected()) {
		//FIXME: now it is limited to roll=0
		using namespace ::math;

		// We simulate a little pitch when breaking and accelerating
// 		flt pitchAngle = (acceleration * 0.02f) * std::min(1.0f, std::abs(theSimulator::instance().getSpeed(mSimulationObjectId)));
		Quaternion q = theSimulator::instance().getOrientation(mSimulationObjectId);
		Affine3 localToGlobal = Affine3::TranslationType(carPos3D) * q.toRotationMatrix();
        TimedEgoState egoState(now, ::modules::models::egostate::EgoState(
								   localToGlobal, carMove3D, Vec3(acceleration, 0.f, 0.f), toEulerXyz(q), Vec3(0.0f, 0.0f, -carYawRate), carSpeed
							   ));

        ::modules::models::egostate::EgoStateSetter egoStateSetter;
		mEgoStateSetterIn.read(egoStateSetter);
		egoStateSetter.setCurrentState(egoState);
	}


// 	timedObstacleData->adoptTimeStamp(now);
// 	TimedObstacleData_ptr timedObstacleData = theSimulator::instance().getObstacles(mSimulationObjectId);
	TimedBaseObstacleBundle_ptr timedObstacleData = theSimulator::instance().getObstacles(mSimulationObjectId);
	mObstaclesOut.write(timedObstacleData);

// 	mObjectsOut.write( blocks );
//	mVectorsOut.write( vectors );



	//create general carstate
	TimedCarState carstate = TimedCarState();
	carstate.wheelPositionInput = steerIn;
	carstate.wheelPosition = -theSimulator::instance().getSteer(mSimulationObjectId) / maxSteer;
	carstate.gasPositionInput = speedIn.data;
	carstate.gasPosition = speedIn.data;			//zero delay
	carstate.gearPosition = theSimulator::instance().getGear(mSimulationObjectId);

	if (mAuxDevicesIn.connected()) {
		carstate.adoptAuxDevices(aux);
	}




	//create passat car state
    aa::modules::models::carstate::TimedPassatCarState passatcarstate = aa::modules::models::carstate::TimedPassatCarState();
	passatcarstate.adoptCarState(carstate);

	flt speed = (flt)theSimulator::instance().getSpeed(mSimulationObjectId);
	flt steer = carstate.wheelPosition * -33.894 * D2R;

	flt wheelSpeedRearL;
	flt wheelSpeedRearR;
	flt wheelSpeedFrontL;
	flt wheelSpeedFrontR;

	if (steer == 0) {
		wheelSpeedRearL = wheelSpeedRearR = wheelSpeedFrontL = wheelSpeedFrontR = speed;
	}
	else {
		if (steer < 0) {
			double distIccToCenterRear = 2.709 / tan(-steer);
			wheelSpeedRearR = (distIccToCenterRear - 1.551 / 2) / distIccToCenterRear * speed;
			wheelSpeedRearL = (distIccToCenterRear + 1.551 / 2) / distIccToCenterRear * speed;
			wheelSpeedFrontR = sqrt((distIccToCenterRear - 1.551 / 2) * (distIccToCenterRear - 1.551 / 2) + 2.709 * 2.709) / distIccToCenterRear * speed;
			wheelSpeedFrontL = sqrt((distIccToCenterRear + 1.551 / 2) * (distIccToCenterRear + 1.551 / 2) + 2.709 * 2.709) / distIccToCenterRear * speed;
		}
		else {
			double distIccToCenterRear = 2.709 / tan(steer);
			wheelSpeedRearR = (distIccToCenterRear + 1.551 / 2) / distIccToCenterRear * speed;
			wheelSpeedRearL = (distIccToCenterRear - 1.551 / 2) / distIccToCenterRear * speed;
			wheelSpeedFrontR = sqrt((distIccToCenterRear + 1.551 / 2) * (distIccToCenterRear + 1.551 / 2) + 2.709 * 2.709) / distIccToCenterRear * speed;
			wheelSpeedFrontL = sqrt((distIccToCenterRear - 1.551 / 2) * (distIccToCenterRear - 1.551 / 2) + 2.709 * 2.709) / distIccToCenterRear * speed;
		}
	}

	passatcarstate.wheelSpeeds = TimedWheelSpeeds(now, WheelSpeeds(wheelSpeedRearL, wheelSpeedRearR, wheelSpeedFrontL, wheelSpeedFrontR));
	passatcarstate.steerAssist3Status = TimedSteerAssist3Status(now, SteerAssist3Status((flt)(-carstate.wheelPosition * 530.0), (flt)0.0));
	passatcarstate.gearStatus = TimedGearStatus(now, GearStatus(GearStatus::legacyGearPosToPassatChosenPos(carstate.gearPosition)));

	if (speedIn.data < 0) {
		passatcarstate.brakeStatus1 = TimedBrakeStatus1(now, BrakeStatus1(-127.5 * speedIn.data));
		passatcarstate.throttleStatus = TimedThrottleStatus(now, ThrottleStatus(speedIn.data * 0.0));
	}
	else {
		passatcarstate.brakeStatus1 = TimedBrakeStatus1(now, BrakeStatus1(0.0));
		passatcarstate.throttleStatus = TimedThrottleStatus(now, ThrottleStatus(speedIn.data * (4.0 - 0.75) + 0.75));
	}

	//write out car states
	mCarStateOut.write(carstate);
	mPassatCarStateOut.write(passatcarstate);


    //calc and write out (fake) odometric data (with gaussian noise)
    Vec3 egoPos = theSimulator::instance().getPosition(mSimulationObjectId);
    flt distanceTravelled = (egoPos-mLastEgoPos).norm();
    mLastEgoPos = egoPos;

    Quaternion q = theSimulator::instance().getOrientation(mSimulationObjectId);
    Vec3 egoOrientation = toEulerXyz(q);
    flt egoAngle = egoOrientation[2];
    flt angleDiff = egoAngle - mLastEgoAngle;
    mLastEgoAngle = egoAngle;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<flt> posd(distanceTravelled,mOdometryDistStdDev);
    distanceTravelled = posd(gen);
    std::normal_distribution<flt> angled(angleDiff,mOdometryAngleStdDev);
    angleDiff = angled(gen);

    data::TimedOdometricData od(now, data::OdometricData(distanceTravelled,angleDiff));
    mOdometricDataOut.write(od);
}

void Simulator::stopHook()
{
}

void Simulator::errorHook()
{
}


bool Simulator::togglePause()
{
	mPause = !mPause.get();
	return mPause.get();
}




