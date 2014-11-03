//
// C++ Interface: ScannerCamera
//
// Description:
//
//
// Author:  <lars@larswolter.de>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#pragma once

#include <math/AutoMath.h>

#include <osg/ShapeDrawable>
#include <osg/CameraNode>
#include <osg/Texture2D>

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class ScannerCamera
	: public osg::Transform
{
	static osg::Geode * groundNode;
public:
	typedef ::math::flt flt;
	typedef ::math::Vec3 Vec3;
	ScannerCamera(osg::MatrixTransform * toRender, flt viewYaw, flt viewPitch, int xres, int yres, flt frustumDegree = 90.0f);
	const std::vector<Vec3>& getScanData();
	void updateViewMatrix(osg::Matrixd mat);

public:
	std::vector<osg::ref_ptr<osg::CameraNode> > cameras;
	std::vector<osg::Image *> images;
	osg::ref_ptr<osg::Texture2D> texture;
	std::vector<Vec3> scanPoints;
	int width, height;
	flt yawF, pitchF;
	osg::MatrixTransform * child;
};

}
}
}
}
