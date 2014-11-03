#include "OsgCamera.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <osg/PositionAttitudeTransform>
#include <osg/ref_ptr>
#include <osg/io_utils>
#include <QMutexLocker>
#include <gui/LookAtCameraView.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace osgcamera
{

REGISTERTASKCONTEXT(OsgCamera);

OsgCamera::OsgCamera(std::string const & name)
	: Painter3DTask(name)
	, mCameraParameterIn("CameraParameterIn")
	, mImageIn("ImageIn", RTT::ConnPolicy::buffer(16))
	, mStereoImageIn("StereoImageIn", RTT::ConnPolicy::buffer(16))
{
	addPort(mCameraParameterIn);
	addPort(mImageIn);
	addPort(mStereoImageIn);
}


OsgCamera::~OsgCamera()
{

}


bool OsgCamera::startHook()
{
	return Painter3DTask::startHook();
}


void OsgCamera::updateHook()
{
	QMutexLocker lock(&mMutex);


	::data::config::CameraParameter::ListPtr ptr;
	mCameraParameterIn.read(ptr);
	::data::TimedImagePtr imagePtr;

	while (RTT::NewData == mImageIn.read(imagePtr)) {
		if (!imagePtr->cameraParameterPtr && ptr) {
			imagePtr->cameraParameterPtr = ::data::config::CameraParameter::getParameter(ptr, imagePtr->guid, imagePtr->unit);
		}

		updateCamera(imagePtr);
	}

	data::TimedStereoImagePtr stereoImagePtr;

	while (RTT::NewData == mStereoImageIn.read(stereoImagePtr)) {
		if (stereoImagePtr.left && !stereoImagePtr.left->cameraParameterPtr && ptr) {
			stereoImagePtr.left->cameraParameterPtr = ::data::config::CameraParameter::getParameter(ptr, stereoImagePtr.left->guid, stereoImagePtr.left->unit);
		}

		if (stereoImagePtr.middle && !stereoImagePtr.middle->cameraParameterPtr && ptr) {
			stereoImagePtr.middle->cameraParameterPtr = ::data::config::CameraParameter::getParameter(ptr, stereoImagePtr.middle->guid, stereoImagePtr.middle->unit);
		}

		if (stereoImagePtr.right && !stereoImagePtr.right->cameraParameterPtr && ptr) {
			stereoImagePtr.right->cameraParameterPtr = ::data::config::CameraParameter::getParameter(ptr, stereoImagePtr.right->guid, stereoImagePtr.right->unit);
		}

		updateCamera(stereoImagePtr.left);
		updateCamera(stereoImagePtr.middle);
		updateCamera(stereoImagePtr.right);
	}
}

void OsgCamera::updateCamera(::data::TimedImagePtr const & imagePtr)
{
	if (!mSceneNodePtr || !imagePtr || !imagePtr->cameraParameterPtr) {
		return;
	}
	bool found = false;
	bool changed = false;
	int index = 0;
	for (int i=0; i<mCurrentCameraParams.size(); ++i) {
		if (mCurrentCameraParams[i]->guid() == imagePtr->cameraParameterPtr->guid() && mCurrentCameraParams[i]->unit() == imagePtr->cameraParameterPtr->unit()) {
			found = true;
			index = i;
			if (!(*mCurrentCameraParams[i] == *imagePtr->cameraParameterPtr)) { // carToLocal transform has changed
				changed = true;
				mCurrentCameraParams[i] = imagePtr->cameraParameterPtr;
			}
		}
	}
	if (found && !changed) {
		return;
	}
	if (!found) {
		mCurrentCameraParams.push_back(imagePtr->cameraParameterPtr);
		index = mCurrentCameraParams.size()-1;
		mCameraTransforms.push_back(osg::ref_ptr<osg::PositionAttitudeTransform>(new osg::PositionAttitudeTransform));
	}

	::data::config::CameraParameter & paras = *mCurrentCameraParams[index];

	osg::Vec3d pos(paras.position()[0], paras.position()[1], paras.position()[2]);
	osg::Matrixd rot(paras.localToCar().matrix().data());
	rot(3, 0) = 0;
	rot(3, 1) = 0;
	rot(3, 2) = 0;
	osg::Quat attitude;
	osg::Matrixd baseSwitch(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);
	attitude.set(baseSwitch*rot);
	mCameraTransforms[index]->setPosition(pos);
	mCameraTransforms[index]->setAttitude(attitude);
	mCameraTransforms[index]->setScale(osg::Vec3d(1,1,1));

	osg::ref_ptr<gui::LookAtCameraView> cam = osg::ref_ptr<gui::LookAtCameraView>(new gui::LookAtCameraView()) ;
	std::stringstream s;
	s << "Camera " << index;
	cam->setName(s.str());
	cam->setFieldOfViewMode(osg::CameraView::HORIZONTAL);
	cam->setFieldOfView(58.0);
	cam->setViewDistance(0.0f);
	mCameraTransforms[index]->addChild(cam);

	if (!found) {
		mSceneNodePtr->addChild(mCameraTransforms[index].get());
		toggleView(-1);
	}
}


void OsgCamera::init3D(SceneNodePtr ptr)
{
	mSceneNodePtr = ptr;
	osg::StateSet * stateSet = mSceneNodePtr->getOrCreateStateSet();
	stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
	stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

}
}
}
}
