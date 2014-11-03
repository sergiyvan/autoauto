#include "DisplayCarData.h"

#include <rtt/Logger.hpp>
#include <set>
#include <algorithm>
#include <tr1/unordered_map>
#include <osg/Vec3>
#include <osg/LightSource>
#include <osg/Light>
#include <osg/Geode>
#include <osg/Material>
#include <osg/CameraNode>
#include <osg/CameraView>
#include <osg/Texture2D>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgUtil/SceneView>
#include <osgDB/ReadFile>
#include <boost/random.hpp>
#include <boost/filesystem/convenience.hpp>

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <aa/modules/models/rndf/io/RndfData.h>
#include <aa/modules/models/rndf/io/RndfReader.h>
#include <gui/LookAtCameraView.h>
#include <gui/GlDispatch.h>
#include <data/VehicleData.h>
#include <math/AutoMath.h>
#define VERBOSE
#include <util/DLog.h>


using namespace std;
using namespace ::math;
using RTT::Logger;
using namespace modules::models::carstate;
//for test

namespace aa
{
namespace modules
{
namespace display
{
namespace carview
{


REGISTERTASKCONTEXT(DisplayCarData);


class DisplayCarData::impl
	: public osg::NodeCallback
{
private:
	impl(impl const &);
	impl & operator=(impl const &);

public:
	impl()
		: carGear(13)
		, test1(0)
		, test2(0)
		, test3(0)
		, test4(0)
		, testStep(1)
		, mIgnoreZ("IgnoreZ", "if true, car is always drawn at z=0", false)
/*		, p_printGear("printGear", "print inserted gear", false)
		, p_printCarPos("printCarPos", "print car position (in global coordinates) over car", false)
		, p_printForwardDir("printForwardDir", "print forward direction vector", false)
		, p_printVelocity("printVelocity", "print velocity vector (in global coordinates)", false)
		, p_printAcceleration("printAcceleration", "print x/y/z acceleration vector (global coordinates)", false)
		, p_printAngularRates("printAngularRates", "print angular turn rates (roll/pitch/yaw)", false)
		, p_printSpeed("printSpeed", "print vehicle speed (in forward direction) and x/y translation", false)
		, p_printRpy("printRpy", "print vehicle roll/pitch/yaw angle (relative to global coordinate axis)", false)
*/	{}

	~impl() {}

	/** we use this method to update our main node for this painter
		*  We check if there are new objects that we need to put into the graph
		*  Then we update the position and text of all objects
		*/
	virtual void operator()(osg::Node * node, osg::NodeVisitor * nv)
	{
		mutex.lock();

		//First we update our car
		if (ourCar.valid()) {


#if defined(USE_EIGEN)
			osg::Matrix m(egoState.localToGlobal().matrix().data());
#else
			osg::Matrix m(egoState.localToGlobal().matrix().transpose().data());
#endif

//{for test  now comment
			/*			Quaternion q;
						osg::Matrixd m ;
						BOOST_FOREACH(Simulant * sim, theSimulator::instance().objects) {
							if ( sim->typ==1){
							q = theSimulator::instance().getOrientation(sim->id);
							flt const angle = 2.0 * std::acos(q.w()) * math::R2D;
							Vec3 const axis = q.vec() / q.vec().norm();
							m= osg::Matrixd::rotate(angle * math::D2R, osg::Vec3f(axis(0), axis(1), axis(2)));
							m(3,0)=sim->pos()(0);
							m(3,1)=sim->pos()(1);
							m(3,2)=sim->pos()(2);
							}
						}
			*/
//for test}

			if (mIgnoreZ) {
				m(3, 2) = 0.0;
			}

#if !defined(NDEBUG)
			// Check, if the matrix is correctly passed (row-major vs. column-major)
			Vec3 carPos = egoState.localToGlobal().translation();
			osg::Vec3 v = m.getTrans();

			for (uint i = 0; i < 3; ++i) {
				flt diff = std::abs(v[i] - carPos[i]);

				if (diff >= 0.1) {
					Logger::log(Logger::Error) << "------------------------------------------> abs(v[i] - carPos[i]) = " << diff << Logger::endl;
					Logger::log(Logger::Error) << "v[i] = " << v[i] << Logger::endl;
					Logger::log(Logger::Error) << "carPos[i] = " << carPos[i] << Logger::endl;
				}
				else {
//				 		Logger::log(Logger::Error) << "abs(v[i] - carPos[i]) = " << diff << Logger::endl;
				}

				//assert( diff < 0.1);
			}

#endif


			ourCar->setMatrix(m);

			//{debug
			/*	TimeStamp now;
				now.stamp();
				//PosInDispalycarData<<now<<endl;
				PosInDispalycarData<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
				PosInDispalycarData<<RTT::TimeService::ticks2nsecs(RTT::TimeService::Instance()->getTicks())<<endl;
				PosInDispalycarData<<m(3,0)<<"  "<<m(3,1)<<"  "<< m(3,2)<<endl;
				PosInDispalycarData<<"***********************************************"<<endl;
				PosInDispalycarData<<m(0,0)<<"  "<<m(0,1)<<"  "<<m(0,2)<<" "<<m(0,3)<<endl;
				PosInDispalycarData<<m(1,0)<<"  "<<m(1,1)<<"  "<<m(1,2)<<" "<<m(1,3)<<endl;
				PosInDispalycarData<<m(2,0)<<"  "<<m(2,1)<<"  "<<m(2,2)<<" "<<m(2,3)<<endl;
				PosInDispalycarData<<m(3,0)<<"  "<<m(3,1)<<"  "<<m(3,2)<<" "<<m(3,3)<<endl;

				cout<<"=================================================================================="<<endl;
				cout<<"PosInDisplayCarData"<<endl;
				cout<<RTT::TimeService::ticks2nsecs(RTT::TimeService::Instance()->getTicks())<<endl;
				cout<<m(3,0)<<"  "<<m(3,1)<<"  "<< m(3,2)<<endl;
				*/
			//debug}

			// check wheelcount and add wheels if there arent enough
			if (wheelNodes.empty()) {
				for (uint w = 0; w < 4; w++) {
					wheelNodes.push_back(new osg::MatrixTransform());
					wheelNodes.back()->addChild(meshes["wheel"]);
					carToImu->addChild(wheelNodes.back());
				}
			}

			// update all the wheels
			flt const tireRadius = ::data::theVehicleData::instance().getPropertyType<flt>("tireRadius")->value();
			flt const front = ::data::theVehicleData::instance().getPropertyType<flt>("frontShaftDistance")->rvalue();
			flt const rear = ::data::theVehicleData::instance().getPropertyType<flt>("rearShaftDistance")->rvalue();
			flt const dist = ::data::theVehicleData::instance().getPropertyType<flt>("tireDistance")->rvalue() * 0.5f;
			flt const ground = ::data::theVehicleData::instance().getPropertyType<flt>("groundDistance")->rvalue();
			flt const maxSteer = ::data::theVehicleData::instance().getPropertyType<flt>("maxSteer")->rvalue();
			flt const steerArc = carState.wheelPosition * -maxSteer;
			m.makeRotate(math::PI * 0.5f + steerArc, 0, 0, 1);
			m.setTrans(front, dist, tireRadius - ground);
			wheelNodes[0]->setMatrix(m);
			m.setTrans(front, -dist, tireRadius - ground);
			wheelNodes[1]->setMatrix(m);
			m.makeRotate(math::PI * 0.5f, 0, 0, 1);
			m.setTrans(rear, dist, tireRadius - ground);
			wheelNodes[2]->setMatrix(m);
			m.setTrans(rear, -dist, tireRadius - ground);
			wheelNodes[3]->setMatrix(m);



			string carString("");
			carString += carInfos;
			/*
			if (p_printGear.get())
			{
				switch (carGear)
				{
					case 0: carString = "CHANGING GEARS";break;
					case 1: carString = "PARK";break;
					case 2: carString = "REVERSE";break;
					case 4: carString = "NORMAL";break;
					case 8: carString = "DRIVE";break;
					case 16: carString = "DRIVE 2";break;
					default: carString = "UNDEFINED GEAR";break;
				}
			}

			// add egostate information
			char egoStateString[30];
			if (p_printCarPos.get())
			{

				const Vec3 & var = egoState.position();
				snprintf(egoStateString, 30, "\npos:(%2.1f/%2.1f/%2.1f)", var[0], var[1], var[2]);
				carString += egoStateString;
			}
			if (p_printForwardDir.get())
			{
				const Vec3 & var = egoState.forwardDirection();
				snprintf(egoStateString, 30, "\ndir:(%2.1f/%2.1f/%2.1f)", var[0], var[1], var[2]);
				carString += egoStateString;
			}
			if (p_printVelocity.get())
			{
				const Vec3 & var = egoState.velocity();
				snprintf(egoStateString, 30, "\nv:(%2.1f/%2.1f/%2.1f)", var[0], var[1], var[2]);
				carString += egoStateString;
			}
			if (p_printAcceleration.get())
			{
				const Vec3 & var = egoState.acceleration();
				snprintf(egoStateString, 30, "\nacc:(%2.1f/%2.1f/%2.1f)", var[0], var[1], var[2]);
				carString += egoStateString;
			}
			if (p_printAngularRates.get())
			{
				const Vec3 & var = egoState.angularRates();
				snprintf(egoStateString, 30, "\nvphi:(%2.1f/%2.1f/%2.1f)", var[0], var[1], var[2]);
				carString += egoStateString;
			}
			if (p_printSpeed.get())
			{
				flt v = egoState.vehicleSpeed();
				const Vec3 & angularRates = egoState.angularRates();
				flt turnRate = angularRates[2];
				snprintf(egoStateString, 30, "\nspeed/vphi:(%2.1f/%2.1f)");
				carString += egoStateString;
			}
			if (p_printRpy.get())
			{
				const Vec3 & var = egoState.rollPitchYaw();
				snprintf(egoStateString, 30, "\nvrpy:(%2.1f/%2.1f/%2.1f)", var[0], var[1], var[2]);
				carString += egoStateString;
			}
			*/
			carText->setText(carString);

		}

		mutex.unlock();

		traverse(node, nv);
	}

	/*// Properties to alter car text string
	RTT::Property<bool> p_printGear;
	RTT::Property<bool> p_printCarPos;
	RTT::Property<bool> p_printForwardDir;
	RTT::Property<bool> p_printVelocity;
	RTT::Property<bool> p_printAcceleration;
	RTT::Property<bool> p_printAngularRates;
	RTT::Property<bool> p_printSpeed;
	RTT::Property<bool> p_printRpy;
	*/
	RTT::Property<bool> mIgnoreZ;

	QMutex mutex;
	tr1::unordered_map<std::string, osg::Node *> meshes;
	TimedEgoState egoState;
	::modules::models::carstate::TimedCarState carState;
	std::string carInfos;
	int carGear;

	bool changeShader;
	bool drawDepthView;
	bool drawGpsErrorHalo;
	flt test1, test2, test3, test4, testStep;

	std::vector<osg::MatrixTransform *> wheelNodes;
	std::vector<std::pair<Vec3, Vec3> > vectors;
	std::vector<boost::tuple<Mat4x4, std::string, Vec4> > globals;

	osg::ref_ptr<osg::MatrixTransform> ourCar;
	osg::ref_ptr<osg::MatrixTransform> carToImu;
	osg::ref_ptr<osgText::Text> carText;
	SceneNodePtr sceneNode;
	osg::ref_ptr<osg::Texture2D> texture;

	// camera which'll be activated by pressing Key C
	osg::ref_ptr<gui::LookAtCameraView> cam;

	osg::ref_ptr<osg::PositionAttitudeTransform> gpsErrorHalo;



};


DisplayCarData::DisplayCarData(string const & name)
	: Painter3DTask(name)
	// Members
	, inited(false)
	// Properties
	, mDrawOverlays("DrawOverlays", "DrawOverlays", 0)
	, mDrawDepthView("DrawDepthView", "DrawDepthView", false)
	, mDrawGpsErrorHalo("DrawGpsErrorHalo", "draw a halo over the car to visualize the gps error", true)
	, mDrawAuxDevices("DrawAuxDevices", "draw auxiliary devices (headlight,wiper,siren,turnlights)", true)
	, mCarModelFilename("CarModelFileName", "CarModelFileName", "3dmodels/car.ac")
// Ports
	, mCarGear("CarGear")
	, mVectors("Vectors")
	, mGlobals("Globals")
	, mInfos("Infos")
	, mEgoState("EgoState")
	, mCarState("CarState")
	, mAuxDevices("AuxDevices")

	, mTurnSignalCounter(0)
	, pimpl(new impl())



{
	addProperty(mDrawOverlays);
	addProperty(mDrawDepthView);
	addProperty(mDrawGpsErrorHalo);
	addProperty(mDrawAuxDevices);
	addProperty(mCarModelFilename);
	addProperty(pimpl->mIgnoreZ);


	addEventPort(mEgoState);
	addPort(mCarState);
	addPort(mCarGear);
	addPort(mVectors);
	addPort(mGlobals);
	addPort(mInfos);
	addPort(mAuxDevices);


	addOperation("setCamAttitude", &DisplayCarData::setCamAttitude, this, RTT::ClientThread).doc("setCamAttitude").arg("roll", "new roll of cam").arg("pitch", "new pitch of cam").arg("yaw", "new yaw of cam");

	addOperation("setCamViewDistance", &DisplayCarData::setCamViewDistance, this, RTT::ClientThread).doc("setCamViewDistance").arg("dist", "new distance of cam");

}

DisplayCarData::~DisplayCarData()
{
	stop();
}


bool DisplayCarData::startHook()
{
	OPTIONAL_PORT(mEgoState);

	return RTT::TaskContext::startHook();
}

void DisplayCarData::updateHook()
{
	Logger::In in("DisplayCarData");
	pimpl->mutex.lock();

	mEgoState.read(pimpl->egoState);

	mCarState.read(pimpl->carState);



	mCarGear.read(pimpl->carGear);

	mVectors.read(pimpl->vectors);

	mInfos.read(pimpl->carInfos);


	pimpl->drawDepthView = mDrawDepthView.get();

#if 0
	static int updateCounter = 0;
	updateCounter++;
	Logger::log() << Logger::Debug << "Current Frame: " << updateCounter << Logger::endl;
#endif

	pimpl->mutex.unlock();
}

void DisplayCarData::stopHook()
{

}

#if 0

TODO Needs to be reimplemented for GUIEventHandler
bool DisplayCarData::keyReleaseEvent(QKeyEvent * e)
{
	if (e->key() == Qt::Key_1 && e->modifiers() == Qt::ControlModifier) {
		pimpl->test1 -= pimpl->testStep;
	}
	else if (e->key() == Qt::Key_1) {
		pimpl->test1 += pimpl->testStep;
	}
	else if (e->key() == Qt::Key_2 && e->modifiers() == Qt::ControlModifier) {
		pimpl->test2 -= pimpl->testStep;
	}
	else if (e->key() == Qt::Key_2) {
		pimpl->test2 += pimpl->testStep;
	}
	else if (e->key() == Qt::Key_3 && e->modifiers() == Qt::ControlModifier) {
		pimpl->test3 -= pimpl->testStep;
	}
	else if (e->key() == Qt::Key_3) {
		pimpl->test3 += pimpl->testStep;
	}
	else if (e->key() == Qt::Key_4 && e->modifiers() == Qt::ControlModifier) {
		pimpl->test4 -= pimpl->testStep;
	}
	else if (e->key() == Qt::Key_4) {
		pimpl->test4 += pimpl->testStep;
	}
	else if (e->key() == Qt::Key_9 && e->modifiers() == Qt::ControlModifier) {
		pimpl->testStep = max(0.05f, pimpl->testStep - 0.05f);
	}
	else if (e->key() == Qt::Key_9) {
		pimpl->testStep += 0.05f;
	}
	else if (e->key() == Qt::Key_0) {
		std::cout << "TestVars: " << pimpl->test1 << ", " << pimpl->test2 << ", " << pimpl->test3 << ", " << pimpl->test4 << " - " << pimpl->testStep << std::endl;
	}

	return false;
}


/** We override the input listener method to show mouse hoovering infos for objects
*/
bool DisplayCarData::mouseMoveEvent(bool valid, flt x, flt y, QMouseEvent * e)
{
	if (valid) {

	}

	//we want that the viewer to still handle those events so we return false
	return false;
}
#endif

void DisplayCarData::init3D(SceneNodePtr sceneNode)
{

	pimpl->sceneNode = sceneNode;
	sceneNode->addDescription("DisplayCarData");


	osg::NotifySeverity oldNotifyLevel = osg::getNotifyLevel();
	osg::setNotifyLevel(osg::ALWAYS);

	pimpl->meshes["carModel"] = osgDB::readNodeFile(mCarModelFilename.get());
	pimpl->meshes["wheel"] = osgDB::readNodeFile("3dmodels/wheel.ac");


	// temporarily, for truck
	if ((::data::theVehicleData::instance().getPropertyType<std::string>("carName")->value() == "truck") || (::data::theVehicleData::instance().getPropertyType<std::string>("carName")->value() == "ras")) {
		// create scale matrix
		osg::MatrixTransform * transform = new osg::MatrixTransform;
		double targetScale = ::data::theVehicleData::instance().getPropertyType<double>("modelScale")->value();

		transform->setMatrix(osg::Matrix::scale(targetScale, targetScale, targetScale));
		transform->setDataVariance(osg::Object::STATIC);
		transform->addChild(osgDB::readNodeFile(mCarModelFilename.get()));
		std::cout << "Adding node from file " << mCarModelFilename.get() << std::endl;

		// rescale normals
		osg::StateSet * state = transform->getOrCreateStateSet();
		state->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

		// create rotate matrix
		osg::MatrixTransform * transform2 = new osg::MatrixTransform;

		osg::Vec3f  axisx(0, 0, 1);
		osg::Vec3f  axisy(0, 1, 0);
		osg::Vec3f  axisz(1, 0, 0);

		double yaw = ::data::theVehicleData::instance().getPropertyType<double>("yaw")->value();
		double pitch = ::data::theVehicleData::instance().getPropertyType<double>("pitch")->value();
		double roll = ::data::theVehicleData::instance().getPropertyType<double>("roll")->value();
		osg::Quat quaternion(yaw * math::D2R, axisx,
		                     pitch * math::D2R, axisy,
		                     roll * math::D2R, axisz);
		transform2->setMatrix(osg::Matrix::rotate(quaternion));
		transform2->setDataVariance(osg::Object::STATIC);
		transform2->addChild(transform);

		// create translate matrix
		osg::MatrixTransform * transform3 = new osg::MatrixTransform;
		double x = ::data::theVehicleData::instance().getPropertyType<double>("x")->value();
		double y = ::data::theVehicleData::instance().getPropertyType<double>("y")->value();
		double z = ::data::theVehicleData::instance().getPropertyType<double>("z")->value();
		transform3->setMatrix(osg::Matrix::translate(osg::Vec3d(x, y, z)));
		transform3->setDataVariance(osg::Object::STATIC);
		transform3->addChild(transform2);

		pimpl->meshes["carModel"] = transform3;

		if (::data::theVehicleData::instance().getPropertyType<std::string>("carName")->value() == "ras") {
			pimpl->meshes["wheel"] = 0;
		}

	}


	osg::setNotifyLevel(oldNotifyLevel);


	flt const imuPitch = (flt)::data::theVehicleData::instance().getPropertyType<flt>("imuPitch")->value();

	pimpl->carToImu = osg::ref_ptr<osg::MatrixTransform>(new osg::MatrixTransform(osg::Matrix::rotate(imuPitch, 0, 1, 0)));
	pimpl->ourCar = osg::ref_ptr<osg::MatrixTransform>(new osg::MatrixTransform());
	pimpl->ourCar->setName("Car");
	pimpl->carToImu->addChild(pimpl->meshes["carModel"]);
	pimpl->ourCar->addChild(pimpl->carToImu);

	// We add our pimpl as callback for updates to the car node
	pimpl->ourCar->setUpdateCallback(pimpl.get());

	pimpl->cam = osg::ref_ptr<gui::LookAtCameraView>(new gui::LookAtCameraView()) ;
	pimpl->cam->setName("Follow Birds-Eye Perspective");
	pimpl->cam->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
	pimpl->cam->setViewDistance(20.0f);

	pimpl->ourCar->addChild(pimpl->cam.get());


	// changes view perspective of the car, shoulder-perspective
	pimpl->cam = osg::ref_ptr<gui::LookAtCameraView>(new gui::LookAtCameraView()) ;
	pimpl->cam->setName("Shoulder Perspective");
	pimpl->cam->setPosition(osg::Vec3d(14.0, 0.0, 3.0));
	pimpl->cam->setViewDistance(20.0f);
	using osg::Vec3d;
	osg:: Quat newAttitude;
	newAttitude.makeRotate(math::D2R * 80, Vec3d(1.0, 0.0, 0.0), math::D2R * 0, Vec3d(0.0, 1.0, 0.0), math::D2R * -90, Vec3d(0.0, 0.0, 1.0));
	pimpl->cam->setAttitude(newAttitude);

	pimpl->ourCar->addChild(pimpl->cam.get());

	// We also want text on our objects
	osgText::Text * objectLabel = new osgText::Text();
	objectLabel->setText("Our Car");
	objectLabel->setCharacterSize(1.0); // thats 50cm tall letters
	objectLabel->setAxisAlignment(osgText::Text::SCREEN);  //allways readable facing the camera
	objectLabel->setFontResolution(16, 16);
	//objectLabel->setCharacterSizeMode(osgText::Text::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT);

	objectLabel->setDrawMode(osgText::Text::TEXT); // we want only text, nothing around
	objectLabel->setAlignment(osgText::Text::CENTER_TOP);
	objectLabel->setPosition(osg::Vec3(0, 0, 3.0f));   // Text is two meter over center
	objectLabel->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	pimpl->carText = osg::ref_ptr<osgText::Text>(objectLabel);
	// Text must also be kept in a Geode and added to the transform
	osg::Geode * geode = new osg::Geode();
	geode->addDrawable(objectLabel);

	pimpl->ourCar->addChild(geode);




	// the last thing we add to the scene graph is ourself to do some line and vector
	// drawing. we want to do that relative to the car, so we add ourself under the car transform
	geode = new osg::Geode();
	geode->getOrCreateStateSet()->setRenderBinDetails(120, "RenderBin");
	geode->addDrawable(this);
	pimpl->ourCar->addChild(geode);
	setUseDisplayList(false);

	sceneNode->addChild(pimpl->ourCar.get());
//	initTextOutput(sceneNode);
}

/** We draw our simple lines and stuff here, thats easier then adding each line
 * to the graph as extra object
 */
void DisplayCarData::draw3D(DrawArg arg)
{
	pimpl->mutex.lock();

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	// only if we have a texture we render the quad in front of the car
	if (pimpl->texture.valid() && mDrawDepthView.get()) {
		glColor4f(1.0f, 1.0f, 1.0f, 0.8f);
		glEnable(GL_TEXTURE_2D);
#if OSG_VERSION_MAJOR<2
		pimpl->texture->apply(arg);
#else
		pimpl->texture->apply(*arg.getState());

#endif

		Vec3 frontpos = Vec3(4, 0, 2);
		glDepthMask(GL_FALSE);

		glBegin(GL_QUADS);
		static const flt range = 1.0f;
		static const flt stepwidth = 0.05;

		for (flt i = 0.0f; i < range; i += 0.05f) {
#if defined(USE_EIGEN)
			Quaternion quat(Quaternion::AngleAxisType((i - 0.5f * range) * M_PI, Vec3::UnitZ()));
			Vec3 p1 = quat._transformVector(frontpos);
			Quaternion(Quaternion::AngleAxisType((i - 0.5f * range) * M_PI, Vec3::UnitZ()));
			quat = Quaternion(Quaternion::AngleAxisType((i + stepwidth - 0.5f * range) * M_PI * 1, Vec3::UnitZ()));
			Vec3 p2 = quat._transformVector(frontpos);
#else
			Quaternion quat(Vec3(0, 0, 1), (i - 0.5f * range) * M_PI);
			Vec3 p1 = quat.rotate(frontpos);
			quat = Quaternion(Vec3(0, 0, 1), (i + stepwidth - 0.5f * range) * M_PI * 1);
			Vec3 p2 = quat.rotate(frontpos);
#endif
			glTexCoord2f(1.0f - i, 1);
			glVertex3f(p1(0), p1(1), p1(2) + 4);
			glTexCoord2f(1.0f - i - stepwidth, 1);
			glVertex3f(p2(0), p2(1), p2(2) + 4);
			glTexCoord2f(1.0f - i - stepwidth, 0);
			glVertex3f(p2(0), p2(1), p2(2));
			glTexCoord2f(1.0f - i, 0);
			glVertex3f(p1(0), p1(1), p1(2));
		}

		glEnd();
		glDisable(GL_TEXTURE_2D);
		glDepthMask(GL_TRUE);
	}

	//Draw Aux Data
	if (mDrawAuxDevices.get() && mAuxDevices.connected()) {

		AuxDevicesData auxData;
		mAuxDevices.read(auxData);


		//draw turn signal
		if (auxData.turnsignalState == AuxDevicesData::TURNSIGNAL_OFF) {
			mTurnSignalCounter = 0;

		}
		else {
			mTurnSignalCounter = (mTurnSignalCounter + 1) % 40;

			if (mTurnSignalCounter < 25) {
				flt const frontShaftDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontShaftDistance")->value();
				flt const frontDistance = ::data::theVehicleData::instance().getPropertyType<flt>("frontDistance")->value();
				flt const medialAxisDistance = ::data::theVehicleData::instance().getPropertyType<flt>("medialAxisDistance")->value();
				flt const carWidth = ::data::theVehicleData::instance().getPropertyType<flt>("realCarWidth")->value();
				// Vec3 const carPos = pimpl->egoState.position();
				Vec3 const forwardDirection = normalized(pimpl->egoState.forwardDirection());
				Vec3 const rightDirection = Vec3(forwardDirection[1], -forwardDirection[0], forwardDirection[2]);

				//imu is at 0,0,0
				Vec3 leftTurnLightPos = (frontShaftDistance + frontDistance * 0.7f) * forwardDirection + (medialAxisDistance - carWidth * 0.3f) * rightDirection;
				Vec3 rightTurnLightPos = (frontShaftDistance + frontDistance * 0.7f) * forwardDirection + (medialAxisDistance + carWidth * 0.3f) * rightDirection;


				if (auxData.turnsignalState == AuxDevicesData::TURNSIGNAL_LEFT || auxData.turnsignalState == AuxDevicesData::TURNSIGNAL_HAZARDS) {
					glBegin(GL_TRIANGLE_FAN);
					glColor4f(1.0f, 0.9f, 0.f, 1.f);
					glVertex3v(leftTurnLightPos.data());

					glColor4f(1.0f, 0.9f, 0.0f, 0.0f);

					for (flt i = 0.05f; i < 0.45f; i += 0.05f) {
						glVertex3f(leftTurnLightPos[0] + 1.5f * cos(M_PI * i), leftTurnLightPos[1] + 1.5f * sin(M_PI * i), leftTurnLightPos[2]);
					}

					glEnd();
				}


				if (auxData.turnsignalState == AuxDevicesData::TURNSIGNAL_RIGHT || auxData.turnsignalState == AuxDevicesData::TURNSIGNAL_HAZARDS) {
					glBegin(GL_TRIANGLE_FAN);
					glColor4f(1.0f, 0.9f, 0.f, 1.f);
					glVertex3v(rightTurnLightPos.data());

					glColor4f(1.0f, 0.9f, 0.0f, 0.0f);

					for (flt i = 0.05f; i < 0.45f; i += 0.05f) {
						glVertex3f(rightTurnLightPos[0] + 1.5f * cos(M_PI * (1.5f + i)), rightTurnLightPos[1] + 1.5f * sin(M_PI * (1.5f + i)), rightTurnLightPos[2]);
					}

					glEnd();
				}
			}
		}
	}


	//draw vectors
	glColor3f(1.0f, 0.0f, 0.0f);

	glBegin(GL_LINES);

	for (uint i = 0; i < pimpl->vectors.size(); i++) {
		if (pimpl->vectors[i].first(0) > 100 || pimpl->vectors[i].first(1) > 100 || pimpl->vectors[i].first(2) > 100) {
			continue;
		}

		if (pimpl->vectors[i].first(0) < -100 || pimpl->vectors[i].first(1) < -100 || pimpl->vectors[i].first(2) < -100) {
			continue;
		}

		if (pimpl->vectors[i].second(0) > 100 || pimpl->vectors[i].second(1) > 100 || pimpl->vectors[i].second(2) > 100) {
			continue;
		}

		if (pimpl->vectors[i].second(0) < -100 || pimpl->vectors[i].second(1) < -100 || pimpl->vectors[i].second(2) < -100) {
			continue;
		}

		glVertex3f(pimpl->vectors[i].first(0) + pimpl->vectors[i].second(0),
		           pimpl->vectors[i].first(1) + pimpl->vectors[i].second(1),
		           pimpl->vectors[i].first(2));
		glVertex3f(pimpl->vectors[i].first(0), pimpl->vectors[i].first(1), pimpl->vectors[i].first(2));
		glVertex3f(pimpl->vectors[i].first(0) - pimpl->vectors[i].second(1) * 0.25f,
		           pimpl->vectors[i].first(1) + pimpl->vectors[i].second(0) * 0.25f,
		           pimpl->vectors[i].first(2));
		glVertex3f(pimpl->vectors[i].first(0) + pimpl->vectors[i].second(1) * 0.25f,
		           pimpl->vectors[i].first(1) - pimpl->vectors[i].second(0) * 0.25f,
		           pimpl->vectors[i].first(2));
	}

	glEnd();

	while (glGetError() != GL_NO_ERROR) {
		;
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glPopAttrib();

	pimpl->mutex.unlock();

}

/**
 * Sets the camera-position relative to the car.
 */
bool DisplayCarData::setCamPosition(flt x, flt y, flt z)
{
	pimpl->cam->setPosition(osg::Vec3d(x, y, z));
// 	cout << "CamPosition (" << x << ", " << y << ", " << z << ")" << endl;
	return true;
}

/**
 * Sets th camera-orientation.
 */
bool DisplayCarData::setCamAttitude(flt roll, flt pitch, flt yaw)
{
	using osg::Vec3d;
	osg::Quat newAttitude;
	newAttitude.makeRotate(math::D2R * roll, Vec3d(1.0, 0.0, 0.0), math::D2R * pitch, Vec3d(0.0, 1.0, 0.0), math::D2R * yaw, Vec3d(0.0, 0.0, 1.0));
	pimpl->cam->setAttitude(newAttitude);
// 	cout << "Roll: " << roll << ", Pitch: " << pitch << "Yaw: " << yaw << endl;
	return true;
}

/**
 * Sets the camera-distance.
 */
bool DisplayCarData::setCamViewDistance(flt dist)
{
	pimpl->cam->setViewDistance((flt)dist);
// 	cout << "ViewDistance: " << dist << endl;
	return true;
}

}
}
}
}
