#pragma once
/**
 * \file OsgCamera.h
 * \brief this task adds a camera view to the scene graph for each real camera
 * \author Tobias Langner
 */

#include <data/Image.h>
#include <data/StereoImage.h>
#include <gui/Painter3DTask.h>
#include <data/config/CameraParameter.h>
#include <QMutex>

namespace aa
{
namespace modules
{
namespace display
{
namespace osgcamera
{

class OsgCamera
	: public gui::Painter3DTask
{
public:
	explicit OsgCamera(std::string const & name);
	~OsgCamera();

	bool startHook();
	void updateHook();
	virtual void init3D(SceneNodePtr);

protected:

	RTT::InputPort< ::data::config::CameraParameter::ListPtr> mCameraParameterIn;
	RTT::InputPort< ::data::TimedImagePtr> mImageIn;
	RTT::InputPort<data::TimedStereoImagePtr> mStereoImageIn;

private:

	void updateCamera(::data::TimedImagePtr const & imagePtr);
	osg::Geode * createPyramid(math::Affine3 const & cameraToWorld, cv::Mat const & invertedIntrinsics, cv::Size const & size, double distance, osg::Vec4 const & color, bool depthIsZ = true);
	QMutex mMutex;
	SceneNodePtr mSceneNodePtr;
	std::vector< ::data::config::CameraParameter::Ptr> mCurrentCameraParams;
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > mCameraTransforms;
};

}
}
}
}
