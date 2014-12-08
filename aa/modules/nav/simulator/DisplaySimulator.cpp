#include "DisplaySimulator.h"
#include <util/TaskContextFactory.h>
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
#include <osg/Billboard>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgText/Text>
#include <osgUtil/SceneView>
#include <osgDB/ReadFile>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#include <math/AutoMath.h>
#include <gui/GlDispatch.h>
#include "SimulatorEngine.h"
#include "AutoSimulant.h"
#include "ScannerCamera.h"

using namespace aa::modules::nav::simulator;

#define VERBOSE
#include <util/DLog.h>

REGISTERTASKCONTEXT(DisplaySimulator);

using namespace std;
using RTT::Logger;
using namespace math;

typedef patterns::Singleton<SimulatorEngine> theSimulator;

Mat4x4 genDrawData(std::string const & model, Vec3 const & position, Quaternion const & q, Vec3 const & scale);

class ReferencedId
	: public osg::Referenced
{
public:
	int id;
	ReferencedId(int i) {
		id = i;
	}
};

class DisplaySimulator::impl
	: public osg::NodeCallback
{
private:
	impl(impl const &);
	impl & operator=(impl const &);

public:
	impl()
		: globalNodes(NULL)
		, scanner()
	{}

	~impl()
	{}

	/** we use this method to update our main node for this painter
		*  We check if there are new objects that we need to put into the graph
		*  Then we update the position and text of all objects
		*/
	virtual void operator()(osg::Node * node, osg::NodeVisitor * nv) {
		osg::LightSource * light = (osg::LightSource *)node;

		if (m_drawWorld == false) {
			if (light->containsNode(globalNodes.get())) {
				light->removeChild(globalNodes.get());
			}
		}
		else if (!light->containsNode(globalNodes.get())) {
			light->addChild(globalNodes.get());
		}

		//First we update our car
		if (globalNodes->getNumChildren() != theSimulator::instance().objects.size()) {
			std::stringstream sstr;
// 			std::cout << "regenerating nodes, got " << globalNodes->getNumChildren()  << " but need " << globals.size()<< std::endl;
			globalNodes->removeChildren(0, globalNodes->getNumChildren());
			scanner.clear();

// 			cout << "generating all Nodes..." << std::endl;
			BOOST_FOREACH(Simulant * sim, theSimulator::instance().objects) {
				// We need a transform node, it holds position and rotation of the object
				osg::Transform * transform = new osg::MatrixTransform();
				transform->setUserData(new ReferencedId(sim->id));

				// we don't draw autonomous cars
				if (sim->typ == 0) {
					// The geode is the actual geometrynode, it gets a drawable of a type
					if (meshes.find(sim->model) == meshes.end()) {
						osg::Geode * geode = new osg::Geode();
						osg::ShapeDrawable * shape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 1.0f));
						//					shape->setColor(osg::Vec4(col(0),col(1),col(2),0.5f) );
						geode->addDrawable(shape);

						transform->addChild(geode);
					}
					else {
						transform->addChild(meshes[sim->model]);
					}

					osg::Billboard * bb = new osg::Billboard();
					bb->setMode(osg::Billboard::POINT_ROT_EYE);
					osgText::Text * text = new osgText::Text;
					text->setCharacterSize(1.0f);
					text->setPosition(osg::Vec3(0.0, 0.0, 1.5));
					text->setAxisAlignment(osgText::Text::SCREEN);
					text->setCharacterSizeMode(osgText::Text::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT);
					sstr.str(" ");
					sstr << sim->id;
					text->setText(sstr.str());
					text->setColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
					text->setBackdropColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
					text->setBackdropType(osgText::Text::OUTLINE);
					text->setFont("/usr/share/fonts/truetype/msttcorefonts/Verdana.ttf");
					bb->addDrawable(text);
					transform->addChild(bb);

					Vec4 col = sim->material;
					osg::StateSet * stateset = transform->getOrCreateStateSet();
					osg::Material * material = dynamic_cast<osg::Material *>(stateset->getAttribute(osg::StateAttribute::MATERIAL));

					if (material == NULL) {
						material = new osg::Material();
					}

					material->setShininess(osg::Material::FRONT_AND_BACK, col(3));

					if (col(3) == 0.0f) {
						material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(col(0), col(1), col(2), 1.0f));
//						material->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(0,0,0,1.0f));
//						material->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(0,0,0,1.0f));
						material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(col(0), col(1), col(2), 1.0f));
						material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(col(0), col(1), col(2), 1.0f));
					}
					else {
						material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(col(0) * 0.1, col(1) * 0.1, col(2) * 0.1, 1.0f));
						material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(col(0), col(1), col(2), 1.0f));
						material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(col(0), col(1), col(2), 1.0f));
					}

					stateset->setAttribute(material, osg::StateAttribute::ON);
				}
				else {
					// for autonoumes cars, we need to supply cameras for depth rendering
					osg::MatrixTransform * camTrans = new osg::MatrixTransform();
					camTrans->addChild(globalNodes.get());
					ScannerCamera * scannerCam = new ScannerCamera(camTrans, 360.0f, 26.0f, 128, 64, 10.0f);
					((osg::LightSource *)node)->addChild(scannerCam);
					scannerCam->setUserData(new ReferencedId(sim->id));
					scanner.push_back(scannerCam);
				}

				globalNodes->addChild(transform);
			}

// 			cout << "generated all Nodes" << std::endl;
		}


		// then we update all globals
		for (uint i = 0; i < globalNodes->getNumChildren(); i++) {
			int id = ((ReferencedId *)globalNodes->getChild(i)->getUserData())->id;

			if (id >= theSimulator::instance().objects.size()) {
				globalNodes->removeChildren(0, globalNodes->getNumChildren());
				break;
			}

            Simulant * sim = theSimulator::instance().objects.at(id);
// 			Mat4x4 mat = genDrawData("", sim->pos, sim->orientation, sim->size);
            Mat4x4 mat = genDrawData(sim->model, sim->pos(), sim->orientation(), sim->modelscale);
			osg::Matrixf matrix((flt *)&(mat));
			globalNodes->getChild(i)->asTransform()->asMatrixTransform()->setMatrix(matrix);
		}

		traverse(node, nv);
	}


	std::vector<osg::ref_ptr<ScannerCamera> > scanner;
	QMutex m_mutex;
	tr1::unordered_map<std::string, osg::Node *> meshes;
	bool m_drawWorld;
	osg::ref_ptr<osg::Group> globalNodes;
	SceneNodePtr sceneNode;
};


DisplaySimulator::DisplaySimulator(string const & name)
	: Painter3DTask(name)
	// Members
	, pimpl(new DisplaySimulator::impl())
	, mDrawPhysics("DrawPhysics", "DrawPhysics", true)
	, mDrawWorld("DrawWorld", "DrawWorld", true)
	, mDrawScanner("DrawScanner", "DrawScanner", false)
{
	addProperty(mDrawPhysics);
	addProperty(mDrawWorld);
	addProperty(mDrawScanner);
}

DisplaySimulator::~DisplaySimulator()
{
	stop();
}


bool DisplaySimulator::startHook()
{
	return true;
}

void DisplaySimulator::updateHook()
{
	pimpl->m_drawWorld = mDrawWorld.get();
}

void DisplaySimulator::stopHook()
{
	tr1::unordered_map<std::string, osg::Node *>::iterator it = pimpl->meshes.begin();

	while (it != pimpl->meshes.end()) {
		it->second->unref();
		it++;
	}
}

void DisplaySimulator::init3D(SceneNodePtr sceneNode)
{
	pimpl->sceneNode = sceneNode;
	sceneNode->addDescription("DisplaySimulator");

	osg::NotifySeverity oldNotifyLevel = osg::getNotifyLevel();
	osg::setNotifyLevel(osg::ALWAYS);

	// init 3d models
    pimpl->meshes["dodge.3ds"]    = osgDB::readNodeFile("3dmodels/dodge.3ds");
    pimpl->meshes["passat.3ds"]    = osgDB::readNodeFile("3dmodels/passat.3ds");
    pimpl->meshes["imiev.3ds"]    = osgDB::readNodeFile("3dmodels/imiev.3ds");
    pimpl->meshes["car.ac"]       = osgDB::readNodeFile("3dmodels/car.ac");
	pimpl->meshes["redcar.ac"]    = osgDB::readNodeFile("3dmodels/redcar.ac");
	pimpl->meshes["greencar.ac"]  = osgDB::readNodeFile("3dmodels/greencar.ac");
	pimpl->meshes["yellowcar.ac"] = osgDB::readNodeFile("3dmodels/yellowcar.ac");
// 	pimpl->meshes["kia.obj"]      = osgDB::readNodeFile("3dmodels/kia_rio.obj");
	pimpl->meshes["box.ac"]       = osgDB::readNodeFile("3dmodels/box.ac");
	pimpl->meshes["person.ac"]    = osgDB::readNodeFile("3dmodels/person.ac");
	pimpl->meshes["wheel.ac"]     = osgDB::readNodeFile("3dmodels/wheel.ac");
	pimpl->meshes["M_TREE3.3DS"]  = osgDB::readNodeFile("3dmodels/M_TREE3.3DS");
	pimpl->meshes["M_TREE4.3DS"]  = osgDB::readNodeFile("3dmodels/M_TREE4.3DS");
	pimpl->meshes["M_TREE5.3DS"]  = osgDB::readNodeFile("3dmodels/M_TREE5.3DS");

	osg::setNotifyLevel(oldNotifyLevel);

	tr1::unordered_map<std::string, osg::Node *>::iterator it = pimpl->meshes.begin();

	// ref() on all model meshes
	while (it != pimpl->meshes.end()) {
		it->second->ref();
		it++;
	}

	// light management
	osg::LightSource * ls = new osg::LightSource();
	ls->setName("Light");
	osg::Light * light = new osg::Light();
	light->setDirection(osg::Vec3(0.1, -0.8, 0.3));
	light->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	ls->setLight(light);
	ls->setReferenceFrame(osg::LightSource::ABSOLUTE_RF);
	pimpl->globalNodes = osg::ref_ptr<osg::Group>(new osg::Group());
	ls->addChild(pimpl->globalNodes.get());

	// We add our pimpl as callback for updates to the globalNodes node
	ls->setUpdateCallback(pimpl.get());
	sceneNode->addChild(ls);

	//sceneNode->addChild(pimpl->autonoumesCars.get());
	Painter3DTask::init3D(sceneNode);
}

/** We draw our simple lines and stuff here, thats easier then adding each line
 * to the graph as extra object
 */
void DisplaySimulator::draw3D(DrawArg arg)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	if (mDrawScanner.get()) {
		for (int s = 0; s < pimpl->scanner.size(); s++) {
			const std::vector<Vec3>& scanData = pimpl->scanner[s]->getScanData();
			int id = ((ReferencedId *)pimpl->scanner[s]->getUserData())->id;
// 			cout << "DISPLAYSIMULATOR - draw3D - id: " << id  << "   "  << scanData.size() << endl;

			if (id < theSimulator::instance().objects.size()) {
				AutoSimulant * sim = (AutoSimulant *) theSimulator::instance().objects.at(id);
				Quaternion const & q = sim->orientation();
				glTranslatef(sim->pos()(0), sim->pos()(1), 1.0f);
				flt const angle = 2.0 * std::acos(q.w()) * math::R2D;
				Vec3 const axis = q.vec() / q.vec().norm();
				glRotatef(angle, axis(0), axis(1), axis(2));
				glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);

// 				glMultTransposeMatrixf((flt*)pimpl->scanner->child->getInverseMatrix().ptr());

				glColor3f(1.0f, 0.3f, 0.3f);
				glPointSize(1.0f);
				glBegin(GL_POINTS);

				for (int v = 0; v < scanData.size(); v++) {
					glVertex3v((flt *)&(scanData[v]));
					//glVertex3f(0.0f,0.0f,0.0f);
				}

				glEnd();
			}
		}

// 		while(glGetError() != GL_NO_ERROR);
	}

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glPopAttrib();

	if (mDrawPhysics.get()) {
		theSimulator::instance().draw();
	}
}

