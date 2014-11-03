#include "DisplaySimulatorControl.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <qevent.h>

#include <algorithm>
#include <ostream>
#include <fstream>

#include <osg/Vec3>
#include <osg/Geode>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osg/Group>
#include <osgViewer/View>
#include <boost/random.hpp>
#include <boost/noncopyable.hpp>

#include <rtt/Logger.hpp>
#include <boost/type_traits.hpp>
#include <rtt/internal/DataSourceGenerator.hpp>
#include <util/OrocosHelperFunctions.h>

#include <util/TaskContextFactory.h>
#include <math/Geodetic.h>
#include <patterns/Singleton.h>
#include <math/AutoMath.h>
#include <aa/modules/nav/simulator/SimulatorEngine.h>
#include <aa/modules/nav/simulator/Simulant.h>
#include <aa/modules/models/rndf/ZoneInfo.h>

// #define VERBOSE
#if defined(VERBOSE)
#define DLOG(X)	Logger::log() << Logger::Debug << X << Logger::endl
#else
#define DLOG(X) /**/
#endif

#if defined(USE_EIGEN)
std::ostream & operator<<(std::ostream & ostr, ::math::Quaternion const & q)
{
	ostr << q.coeffs();
	return ostr;
}
#endif


using namespace aa::modules::nav::simulator;
using namespace ::aa::modules::models::rndf;

typedef patterns::Singleton<SimulatorEngine> theSimulator;
typedef patterns::Singleton<RNDFGraph> theRNDFGraph;

REGISTERTASKCONTEXT(DisplaySimulatorCtrl);

using namespace std;
using namespace ::math;
using namespace RTT;
using namespace boost;


class DisplaySimulatorCtrl::impl
	: public osg::NodeCallback
	, boost::noncopyable
{
public:
	impl()
		: mSelectedId(0)
		, mSelectedModel(0) {

        models.push_back(make_pair(string("redcar.ac"), Vec3(1, 1, 1)));
        models.push_back(make_pair(string("box.ac"), Vec3(1, 1, 1)));
        models.push_back(make_pair(string("box"), Vec3(1, 1, 1)));
        models.push_back(make_pair(string("sphere"), Vec3(1, 1, 1)));

//        models.push_back(make_pair(string("truck3.3DS"), Vec3(1.5, 1.5, 1.5)));

//		models.push_back(make_pair(string("truck3.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("man.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("VirtualGround"), Vec3(1, 1, 1)));
//		models.push_back(make_pair(string(""), Vec3(1, 1, 1)));


//		models.push_back(make_pair(string("roadster.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("deluxvan.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("limosine.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("sporcar1.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("muellfahrzeug.3ds"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("passat.3ds"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("dodge.3ds"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("imiev.3ds"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("xeno.3ds"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("trafficLight.3ds"), Vec3(1, 1, 1)));
//		models.push_back(make_pair(string("T282C_std.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("M_TREE3.3DS"), Vec3(1.5, 1.5, 1.5)));



//		models.push_back(make_pair(string("House1.3DS"), Vec3(1, 1, 1)));
//		models.push_back(make_pair(string("House2.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("House3.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("House4.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("House5.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("House6.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("House7.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("House8.3DS"), Vec3(1.5, 1.5, 1.5)));



//		models.push_back(make_pair(string("4wheeldr.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("bus1.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("roadsign.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("mounbike.3DS"), Vec3(1.5, 1.5, 1.5)));
//		models.push_back(make_pair(string("4by4.3DS"), Vec3(1.5, 1.5, 1.5)));



//		models.push_back(make_pair(string("car.ac"), Vec3(1, 1, 1)));
//		models.push_back(make_pair(string("redcar.ac"), Vec3(1, 1, 1)));
//		models.push_back(make_pair(string("greencar.ac"), Vec3(1, 1, 1)));




//		models.push_back(make_pair(string("M_TREE4.3DS"), Vec3(0.3, 0.3, 0.3)));
//		models.push_back(make_pair(string("M_TREE5.3DS"), Vec3(0.3, 0.3, 0.3)));
//		models.push_back(make_pair(string("person.ac"), Vec3(1, 1, 1)));




	}

	~impl()
	{}

	std::vector<std::pair<std::string, Vec3> > models;
	uint mSelectedId;
	uint mSelectedModel;
};

DisplaySimulatorCtrl::DisplaySimulatorCtrl(string const & name)
	: Painter3DTask(name)
	, pimpl(new impl())
    , mEnableDisplay("                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ","EnableDisplay",true)

{
    addProperty(mEnableDisplay);
	addOperation("saveSimScene", &DisplaySimulatorCtrl::saveSimScene, this, RTT::ClientThread).doc("").arg("filename", "Name of the XML-file");
	addOperation("chooseModel", &DisplaySimulatorCtrl::chooseModel, this, RTT::ClientThread).doc("Cycle through set of obstacle models (.3ds files)");

	addOperation("previousObstacleInScenery", &DisplaySimulatorCtrl::previousObstacleInScenery, this, RTT::ClientThread).doc("Get the previous simulant object in the list.");
	addOperation("nextObstacleInScenery", &DisplaySimulatorCtrl::nextObstacleInScenery, this, RTT::ClientThread).doc("Get the next simulant object in the list.");
	addOperation("deleteSelectedObstacle", &DisplaySimulatorCtrl::deleteSelectedObstacle, this, RTT::ClientThread).doc("Remove simulant object  scene");

	addOperation("obsManipulPushForward", &DisplaySimulatorCtrl::obsManipulPushForward, this, RTT::ClientThread).doc("Move simulant forward (depending on its orientation)");
	addOperation("obsManipulPushBackwards", &DisplaySimulatorCtrl::obsManipulPushBackwards, this, RTT::ClientThread).doc("Move simulant backwards (depending on its orientation)");
	addOperation("obsManipulPushLeft", &DisplaySimulatorCtrl::obsManipulPushLeft, this, RTT::ClientThread).doc("Move simulant to the left (depending on its orientation)");
	addOperation("obsManipulPushRight", &DisplaySimulatorCtrl::obsManipulPushRight, this, RTT::ClientThread).doc("Move simulant to the right (depending on its orientation)");

	addOperation("obsManipulLift", &DisplaySimulatorCtrl::obsManipulLift, this, RTT::ClientThread).doc("lift simulant");
	addOperation("obsManipulLower", &DisplaySimulatorCtrl::obsManipulLower, this, RTT::ClientThread).doc("lower simulants position in space");
	addOperation("obsManipulRotateZAxis", &DisplaySimulatorCtrl::obsManipulRotateZAxis, this, RTT::ClientThread).doc("rotate simulant");
	addOperation("obsManipulRotateZAxisN", &DisplaySimulatorCtrl::obsManipulRotateZAxisN, this, RTT::ClientThread).doc("rotate simulants position in space");

	addOperation("obsManipulIncreaseSizeFront", &DisplaySimulatorCtrl::obsManipulIncreaseSizeFront, this, RTT::ClientThread).doc("Increase Size of simulant object");
	addOperation("obsManipulDecreaseSize", &DisplaySimulatorCtrl::obsManipulDecreaseSize, this, RTT::ClientThread).doc("Decrease Size of simulant object");
	addOperation("obsManipulIncreaseSizeSideways", &DisplaySimulatorCtrl::obsManipulIncreaseSizeSideways, this, RTT::ClientThread).doc("Increase Size of simulant object");
	addOperation("obsManipulDecreaseSizeSideways", &DisplaySimulatorCtrl::obsManipulDecreaseSizeSideways, this, RTT::ClientThread).doc("Decrease Size of simulant object");

}


// --------------------- KeyMappings.xml -------------------- //

void DisplaySimulatorCtrl::saveSimScene(std::string const & fname)
{

	std::ofstream fos(fname.c_str());
	fos << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<simulation>\n";

	for (SimulatorEngine::objects_collection_type::iterator cur = theSimulator::instance().objects.begin();
			cur != theSimulator::instance().objects.end(); ++cur) {
		Simulant * sim = *cur;

//		if (!sim->immovable) {
//			continue;
//		}

		fos << "  <simobject>" << endl;
		fos << "    <name>" << sim->name << "</name>" << endl;
		fos << "    <model>" <<  sim->model << "</model>" << endl;
		fos << "    <position>" << sim->pos()[0] << "," << sim->pos()[1] << "," << sim->pos()[2] << "</position>" << endl;
		fos << "    <size>" << sim->size[0] << " " << sim->size[1] << " " << sim->size[2] << "</size>" << endl;
		fos << "    <material>" << sim->material[0] << " " << sim->material[1] << " " << sim->material[2] << " " << sim->material[3] << "</material>" << endl;
		fos << "    <orientation>" << sim->orientation().coeffs()[3] << "," << sim->orientation().coeffs()[0] << "," << sim->orientation().coeffs()[1] << "," << sim->orientation().coeffs()[2] << "</orientation>" << endl;
		fos << "  </simobject>" << endl;

	}

	fos << "</simulation>\n";

	fos.close();
}


void DisplaySimulatorCtrl::chooseModel()
{
	Logger::In in("DisplaySimulatorCtrl");
	pimpl->mSelectedModel = (pimpl->mSelectedModel + 1) % pimpl->models.size();

	cout << "Selected obstacle model: " << pimpl->models[pimpl->mSelectedModel].first << endl;

	Logger::log() << Logger::Info << "Selected obstacle model: " << pimpl->models[pimpl->mSelectedModel].first << Logger::endl;
}

void DisplaySimulatorCtrl::previousObstacleInScenery()
{
	Logger::In in("DisplaySimulatorCtrl");

	if (pimpl->mSelectedId > 0) {
		--pimpl->mSelectedId;
		Logger::log() << Logger::Info << "current object selected: #" <<  pimpl->mSelectedId << Logger::endl;
	}

}

void DisplaySimulatorCtrl::nextObstacleInScenery()
{
	Logger::In in("DisplaySimulatorCtrl");

	if (theSimulator::instance().objects.size() - 1 > pimpl->mSelectedId) {
		++pimpl->mSelectedId;
		Logger::log() << Logger::Info << "current object selected: #" <<  pimpl->mSelectedId << Logger::endl;
	}
}

void DisplaySimulatorCtrl::deleteSelectedObstacle()
{
	Logger::In in("DisplaySimulatorCtrl");

	theSimulator::instance().destroyObject(pimpl->mSelectedId);
	Logger::log() << Logger::Info << "delete object: #" << pimpl->mSelectedId << Logger::endl;
	--pimpl->mSelectedId;
	Logger::log() << Logger::Info << "current object selected: #" <<  pimpl->mSelectedId << Logger::endl;
}

void DisplaySimulatorCtrl::obsManipulLift()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	Vec3 pos = pSim->pos();
	pSim->size[2] += 0.2f ;
	pos(2) += 0.1f;
//	pos(2) += 1.0f;
	pSim->setPosition(pos);
}


void DisplaySimulatorCtrl::obsManipulLower()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	if (pSim->size[2] - 0.2f >= 0.2f) {
		pSim->size[2] -= 0.2f ;
		Vec3 pos = pSim->pos();
		pos(2) -= 0.2f;
//		pos(2) -= 1.0f;
		pSim->setPosition(pos);
	}
}

void DisplaySimulatorCtrl::obsManipulIncreaseSizeFront()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	pSim->size[1] += pSim->size[1] * 0.05f ;
}

void DisplaySimulatorCtrl::obsManipulDecreaseSize()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	pSim->size[1] = max(flt(0.2), pSim->size[1] - pSim->size[1] * 0.05f);
}

void DisplaySimulatorCtrl::obsManipulIncreaseSizeSideways()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	pSim->size[0] += pSim->size[0] * 0.05f ;
}

void DisplaySimulatorCtrl::obsManipulDecreaseSizeSideways()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	pSim->size[0] = max(flt(0.2), pSim->size[0] - pSim->size[0] * flt(0.05));
}

void DisplaySimulatorCtrl::obsManipulPushBackwards()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	Vec3 pos = pSim->pos();
	//pos(1) -= 0.2f;
	pos(1) -= 0.8f;
	pSim->setPosition(pos);
}

void DisplaySimulatorCtrl::obsManipulPushForward()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	Vec3 pos = pSim->pos();
	pos(1) += 0.2f;
//	pos(1) += 0.8f;
	pSim->setPosition(pos);
}

void DisplaySimulatorCtrl::obsManipulPushRight()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	Vec3 pos = pSim->pos();
	pos(0) += 0.2f;
	//pos(0) += 0.8f;
	pSim->setPosition(pos);
}

void DisplaySimulatorCtrl::obsManipulPushLeft()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	Vec3 pos = pSim->pos();
	pos(0) -= 0.2f;
	//pos(0) -= 0.8f;
	pSim->setPosition(pos);
}

// positive rotation
void DisplaySimulatorCtrl::obsManipulRotateZAxis()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	Quaternion ori(pSim->orientation());
#if defined(USE_EIGEN)
	pSim->setOrientation(ori * Quaternion(Quaternion::AngleAxisType(0.05, Vec3::UnitZ())));
#else
	pSim->setOrientation(ori * Quaternion(Vec3(0, 0, 1), 0.05));
#endif
}

// negative rotation
void DisplaySimulatorCtrl::obsManipulRotateZAxisN()
{
	Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!pSim) {
		return;
	}

	Quaternion ori(pSim->orientation());
#if defined(USE_EIGEN)
	pSim->setOrientation(ori * Quaternion(Quaternion::AngleAxisType(-0.05, Vec3::UnitZ())));
#else
	pSim->setOrientation(ori * Quaternion(Vec3(0, 0, 1), -0.05));
#endif


}



// --------------------- END KeyMappings.xml ---------------- //

DisplaySimulatorCtrl::~DisplaySimulatorCtrl()
{
	stop();
}

bool DisplaySimulatorCtrl::startHook()
{
	return true;
}

void DisplaySimulatorCtrl::updateHook()
{
}

void DisplaySimulatorCtrl::stopHook()
{
}

bool DisplaySimulatorCtrl::handle(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object * o, osg::NodeVisitor * nv)
{
	switch (ea.getEventType()) {
	case osgGA::GUIEventAdapter::KEYDOWN:
//		return keyPressEvent(ea);
	case osgGA::GUIEventAdapter::KEYUP:
//		return keyReleaseEvent(ea);
	case osgGA::GUIEventAdapter::PUSH:
		return mousePressEvent(ea, aa, o, nv);
	case osgGA::GUIEventAdapter::RELEASE:
		return mouseReleaseEvent(ea, aa, o, nv);
	case osgGA::GUIEventAdapter::MOVE:
	case osgGA::GUIEventAdapter::DRAG:
		return mouseMoveEvent(ea, aa, o, nv);
	default:
		return false;
	}
}

template<typename ostream>
ostream & operator<<(ostream & os, osg::Vec3d const & v)
{
	os << v[0];
	os << ", ";
	os << v[1];
	os << ", ";
	os << v[2];
}

bool DisplaySimulatorCtrl::pick(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, Vec2 & point)
{
	osgViewer::View * view = dynamic_cast<osgViewer::View *>(&aa);

	if (!view) {
		return false;
	}

	osgUtil::LineSegmentIntersector::Intersections intersections;

	flt x = ea.getX();
	flt y = ea.getY();

	if (view->computeIntersections(x, y, intersections)) {
		for (osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
				hitr != intersections.end();
				++hitr) {
			point(0) = hitr->getWorldIntersectPoint()[0];
			point(1) = hitr->getWorldIntersectPoint()[1];
			return true;
			// gdlist += os.str();
		}
	}

	return false;
}


bool DisplaySimulatorCtrl::mouseMoveEvent(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object *, osg::NodeVisitor *)
{
	// move the selected object by pressing the CTRL
	//   and left mouse key while moving the mouse toward the goal position
	if (0 != (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON & ea.getButtonMask())
			&& 0 != (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {
		Vec2 pos;

		if (!pick(ea, aa, pos)) {
			mMousePos = boost::optional<Vec2>();
			return false;
		}

		mMousePos = boost::optional<Vec2>(pos);
#if 0
		ZoneInfo::zone_ptr zone;

		if (zoneInfo::instance().findZone(mMousePos.get(), zone)) {
			std::cout << "mouse in zone #"  <<  zone->mId << std::endl;
		}

#endif
		Simulant * pSim = theSimulator::instance().objects[pimpl->mSelectedId];

		if (pSim) {
			theSimulator::instance().setPosition(pimpl->mSelectedId, ::math::zeroExtend(pos));
		}

		return true;
	}

	return false;
}

bool DisplaySimulatorCtrl::mousePressEvent(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object *, osg::NodeVisitor *)
{
	RTT::Logger::In in("DisplaySimulatorCtrl");

	Vec2 pos;

	if (!pick(ea, aa, pos)) {
		mMousePos = boost::optional<Vec2>();
		//  std::cout << "pick() was not successful" << std::endl;
		return false;
	}

	mMousePos = boost::optional<Vec2>(pos);

	// adding an obstacle by pressing SHIFT & LEFT mouse button
	if (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON == ea.getButton()
			&& 0 != (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)) {
		boost::optional<ZoneInfo::Zone> zone = theRNDFGraph::instance().zoneInfo().findZone(pos);

		if (zone) {
			Logger::log() << Logger::Debug << "mouse in zone #"  <<  zone.get().getZoneID() << Logger::endl;
		}

		uint newId = theSimulator::instance().registerObject(true, ::math::zeroExtend(pos), pimpl->models[pimpl->mSelectedModel].second,
#if defined(USE_EIGEN)
					 Quaternion(0, 0, 0, 1),
#else
					 Quaternion(0, 0, 1, 0),
#endif
					 pimpl->models[pimpl->mSelectedModel].first);
		pimpl->mSelectedId = newId;


		Logger::log() << Logger::Debug << "adding obstacle" << Logger::endl;
		return true;
	}

	return false;
}

/** Here we handle selection events.
 *
 */
bool DisplaySimulatorCtrl::mouseReleaseEvent(osgGA::GUIEventAdapter const & ea, osgGA::GUIActionAdapter & aa, osg::Object *, osg::NodeVisitor *)
{
	Vec2 pos;

	if (!pick(ea, aa, pos)) {
		return false;
	}

	mMousePos = boost::optional<Vec2>(pos);

	return true;
}


void DisplaySimulatorCtrl::init3D(SceneNodePtr scene)
{
	Painter3DTask::init3D(scene);
	scene->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
	setSupportsDisplayList(false);
}


void DisplaySimulatorCtrl::draw3D(DrawArg)
{
	Simulant * simulant = theSimulator::instance().objects[pimpl->mSelectedId];

	if (!simulant) {
		return;
	}

	if (pimpl->mSelectedId == 0) {
		return;
	}

    if(mEnableDisplay.get()){
        glPushAttrib(GL_ALL_ATTRIB_BITS);
		glDisable(GL_LIGHTING);
		glDisable(GL_BLEND);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		static flt const scale = 0.5f;

		glColor3f(1, 0, 0);
		glBegin(GL_QUADS);
		glVertex3f(simulant->pos()(0) + simulant->size(0) * 0.5 + scale, simulant->pos()(1) + simulant->size(1) * 0.5 + scale, 0);
		glVertex3f(simulant->pos()(0) + simulant->size(0) * 0.5 + scale, simulant->pos()(1) - simulant->size(1) * 0.5 - scale, 0);
		glVertex3f(simulant->pos()(0) - simulant->size(0) * 0.5 - scale, simulant->pos()(1) - simulant->size(1) * 0.5 - scale, 0);
		glVertex3f(simulant->pos()(0) - simulant->size(0) * 0.5 - scale, simulant->pos()(1) + simulant->size(1) * 0.5 + scale, 0);
		glEnd();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

        glPopAttrib();
    }
}
