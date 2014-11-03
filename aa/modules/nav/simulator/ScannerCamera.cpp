//
// C++ Implementation: ScannerCamera
//
// Description:
//
//
// Author:  <lars@larswolter.de>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "ScannerCamera.h"

#include <rtt/Logger.hpp>

#include <set>
#include <algorithm>
#include <tr1/unordered_map>
#include <osg/Vec3>
#include <osg/LightSource>
#include <osg/Light>
#include <osg/Geode>
#include <osg/Material>
#include <osg/CameraView>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/GLExtensions>
#include <osg/GL2Extensions>
#include <osgUtil/SceneView>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <boost/filesystem/convenience.hpp>
#include <math/AutoMath.h>

namespace fs = boost::filesystem;


using namespace aa::modules::nav::simulator;

using namespace std;
using namespace math;
using RTT::Logger;

osg::Geode * ScannerCamera::groundNode = NULL;

ScannerCamera::ScannerCamera(osg::MatrixTransform * toRender, flt viewYaw, flt viewPitch, int xres, int yres, flt frustumDegree)
	: Transform()
	, yawF(viewYaw)
	, pitchF(viewPitch)
	, child(toRender)
{
	Logger::In in("ScannerCamera");

	//setUserData(new osg::ref_ptr<ScannerCamera>(this));
	width = xres;
	height = yres;
	//osg::State& state = *info.getState();

	//load shaders for depth view
	string fragmentShaderPath = osgDB::findDataFile("shader/depth.frag");
	string vertexShaderPath = osgDB::findDataFile("shader/depth.vert");

	if (!fs::exists(fragmentShaderPath) || !fs::exists(vertexShaderPath)) {
		if (!fs::exists(fragmentShaderPath)) {
			Logger::log(Logger::Warning) << "Could not load Fragment GLSL-Shader: " << fragmentShaderPath << " - Depth View disabled" << Logger::endl;
		}

		if (!fs::exists(vertexShaderPath)) {
			Logger::log(Logger::Warning) << "Could not load Vertex GLSL-Shader: " << vertexShaderPath << " - Depth View disabled" << Logger::endl;
		}
	}
	else {
// 		osg::NotifySeverity oldNotifyLevel = osg::getNotifyLevel();
// 		osg::setNotifyLevel( osg::INFO );

		osg::Camera::RenderTargetImplementation renderTargetImplementation;
		/*		if(osg::isGLExtensionSupported(state.getContextID(), "WGL_ARB_render_texture"))
					renderTargetImplementation = osg::CameraNode::FRAME_BUFFER_OBJECT;
				else*/
		renderTargetImplementation = osg::CameraNode::FRAME_BUFFER;

		// load programs
		osg::Program * program = new osg::Program();
		program->addShader(osg::Shader::readShaderFile(osg::Shader::FRAGMENT, fragmentShaderPath));
		program->addShader(osg::Shader::readShaderFile(osg::Shader::VERTEX, vertexShaderPath));


		//image->allocateImage(xres, yres, 1, GL_RGBA, GL_UNSIGNED_BYTE);
		/*
		texture = osg::ref_ptr<osg::Texture2D>(new osg::Texture2D());
		texture->setTextureSize (xres, yres);
		texture->setInternalFormat(GL_RGBA);
		//texture->setSourceFormat(GL_RGBA);
		//texture->setSourceType(GL_FLOAT);
		texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::NEAREST);
		texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::NEAREST);*/
		int numCams = ceil(viewYaw / frustumDegree);
		flt yawPerCam = viewYaw / numCams;

		for (int c = 0; c < numCams; c++) {
			osg::Image * image = new osg::Image();
			image->allocateImage(xres / numCams, yres, 1, GL_RGBA, GL_FLOAT);

			osg::ref_ptr<osg::CameraNode> camera(new osg::CameraNode());
			camera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);
			camera->setViewport(0, 0, xres / numCams, yres);
			camera->setClearColor(osg::Vec4(1000.0f, 1000.0f, 1000.0f, 1000.0f));
			camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
//			camera->setProjectionMatrixAsPerspective(yawPerCam, yawPerCam / viewPitch, 0.1, 200.0f);
			camera->setProjectionMatrixAsFrustum(-tan(yawPerCam * math::D2R * 0.5), tan(yawPerCam * math::D2R * 0.5), -tan(viewPitch * math::D2R * 0.5), tan(viewPitch * math::D2R * 0.5), 1.0f, 200.0f);
			camera->setViewMatrix(osg::Matrix::lookAt(osg::Vec3d(0, 0, 1.0f), osg::Vec3d(-10.0f, 0.0f, 0), osg::Vec3d(0, 0, 1)));
			camera->setRenderOrder(osg::CameraNode::PRE_RENDER);
			camera->setRenderTargetImplementation(renderTargetImplementation);
			camera->attach(osg::CameraNode::COLOR_BUFFER, image);
			//camera->attach( osg::CameraNode::COLOR_BUFFER, texture.get() );

			camera->getOrCreateStateSet()->setAttribute(program, osg::StateAttribute::ON);
			cameras.push_back(camera);
			images.push_back(image);
			this->addChild(camera.get());
			camera->addChild(toRender);

			// add artificial ground
			if (groundNode == NULL) {
				groundNode = new osg::Geode();
				osg::ShapeDrawable * shape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0), 10000.0f, 10000.0f, 0.01f));
				groundNode->addDrawable(shape);
			}

			camera->addChild(groundNode);
		}

		//texture->setImage(0,image);
		Logger::log(Logger::Info) << "Setup Laserscanner Camera with " << numCams << " beams" << Logger::endl;

// 		osg::setNotifyLevel( oldNotifyLevel );
	}
}

void ScannerCamera::updateViewMatrix(osg::Matrixd mat)
{
	flt yawPerCam = yawF / cameras.size();
	flt angle = -yawF * 0.5f + yawPerCam * 0.5f;

	for (size_t i = 0; i < cameras.size(); i++) {
		osg::Matrixd matrix = osg::Matrixd::rotate(angle * math::D2R, osg::Vec3f(0, 1, 0));
		matrix.preMult(mat);
		cameras[i]->setViewMatrix(matrix);
		angle += yawPerCam;
	}
}

const std::vector<Vec3>& ScannerCamera::getScanData()
{
	if (images.size() == 0) {
		return scanPoints;
	}

	scanPoints.clear();
	flt yawPerCam = yawF / images.size();
	flt angle = -yawF * 0.5f + yawPerCam * 0.5f;
	int subWidth = width / images.size();

	for (size_t i = 0; i < images.size(); i++) {
		osg::Vec4f * distances = (osg::Vec4f *)(images[i]->data());
		osg::Matrixd matrix = osg::Matrixd::rotate(angle * math::D2R, osg::Vec3f(0, 0, 1));

		for (int h = 0; h < height; h++) {
			for (int w = subWidth * i; w < subWidth*(i + 1); w++) {
				int w2 = (w - subWidth * i);
				osg::Vec4f cur = distances[subWidth*h+w2];
				osg::Vec3f res;

				if (cur[1] < 1.0f) {
					res[0] = tan((w2 / (flt)subWidth - 0.5f) * yawPerCam * math::D2R) * (cur[1] * 50.0f);
					res[2] = tan((h / (flt)height - 0.5f) * pitchF * math::D2R) * (cur[1] * 50.0f);
					res[1] = (cur[1] * 50.0f);
					res = matrix.postMult(res);
					scanPoints.push_back(Vec3(res[0], res[1], res[2]));
				}
			}
		}

		angle += yawPerCam;
	}

	return scanPoints;
}
