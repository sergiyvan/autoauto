/*!
 * \file "DisplayVirtualRangeScan.h"
 * \brief Display virtual range scan
 * \author Michael Schnürmacher
 */

#include "DisplayObjects.h"

#include <util/TaskContextFactory.h>
#include <gui/GlTools.h>
#include <math/IbeoMath.h>
#include <aa/modules/display/utilities/Util.h>
#include <Eigen/Dense>
#include <modules/models/egostate/EgoState.h>
#include <modules/models/egostate/InterpolationModel.h>


using namespace aa::modules::display::obstacles;
using namespace aa::modules::display::utilities;
using namespace modules::models::egostate;
using namespace aa::data::obstacle;
using namespace aa::data::obstacle::util;
using namespace gui;
using namespace RTT;
using namespace math;

REGISTERTASKCONTEXT(DisplayObjects);


DisplayObjects::DisplayObjects(std::string const & name)
	: Painter3DTask(name)
	// Inports
	, mBaseObjectsIn("BaseObjectsIn")
	// Outports
	// Properties
	, mDisplayBoundingBoxes(false)
	, mDisplayPositionCovariances(true)
	, mDisplayVelocityCovariances(true)
	, mDisplayVelocityVectors(true)
	, mPosColorR(0.0)
	, mPosColorG(0.0)
	, mPosColorB(0.0)
	, mVelColorR(0.0)
	, mVelColorG(0.0)
	, mVelColorB(0.0)
	, mBaseObjectsInCounter(0)
	// Members

{
	// Inports
	addEventPort(mBaseObjectsIn);
	// Outports
	// Properties
	addProperty("DisplayBoundingBoxes", mDisplayBoundingBoxes).doc("Display Bounding Boxes?");
	addProperty("DisplayPositionCovariances", mDisplayPositionCovariances).doc("Display Position Covariances?");
	addProperty("DisplayVelocityCovariances", mDisplayVelocityCovariances).doc("Display Velocity Covariances?");
	addProperty("DisplayVelocityVectors", mDisplayVelocityVectors).doc("Display Velocity Vectors?");
	addProperty("PosColorR", mPosColorR).doc("Red");
	addProperty("PosColorG", mPosColorG).doc("Green");
	addProperty("PosColorB", mPosColorB).doc("Blue");
	addProperty("VelColorR", mVelColorR).doc("Red");
	addProperty("VelColorG", mVelColorG).doc("Green");
	addProperty("VelColorB", mVelColorB).doc("Blue");
	addProperty("BaseObjectsInCounter", mBaseObjectsInCounter).doc("Counts the number of new read data packages from the corresponding port.");
}


bool DisplayObjects::startHook()
{
	return true;
}

void DisplayObjects::updateHook()
{
	mMutex.lock();

	FlowStatus fs = mBaseObjectsIn.read(mBaseObjectsPtr);

	if (fs == NewData && mBaseObjectsPtr) {
		mBaseObjectsInCounter++;
	}

	if (mDisplayPositionCovariances) {
//		mPosCovVecs1.resize(mBaseObjectsPtr->size());
//		mPosCovVecs2.resize(mBaseObjectsPtr->size());
		mPosCovEllipse.resize(mBaseObjectsPtr->size());

		for (uint i = 0; i < mBaseObjectsPtr->size(); ++i) {
			Mat2x2 S = mBaseObjectsPtr->object(i).positionCovarianceMatrix().topLeftCorner(2, 2);
			Eigen::SelfAdjointEigenSolver<Mat2x2> eigensolver(S);
			flt a = sqrt(eigensolver.eigenvalues()(0, 0));
			flt b = sqrt(eigensolver.eigenvalues()(1, 0));
//			mPosCovVecs1[i] = a * eigensolver.eigenvectors().col(0);
//			mPosCovVecs2[i] = b * eigensolver.eigenvectors().col(1);

			flt phi = atan2(eigensolver.eigenvectors().col(0)[1], eigensolver.eigenvectors().col(0)[0]);
			gui::computeEllipseContourPoints(mPosCovEllipse[i], head(mBaseObjectsPtr->object(i).position()), a, b, phi, 18.0 * math::D2R);
		}
	}

	if (mDisplayVelocityCovariances) {
//		mVelCovVecs1.resize(mBaseObjectsPtr->size());
//		mVelCovVecs2.resize(mBaseObjectsPtr->size());
		mVelCovEllipse.resize(mBaseObjectsPtr->size());

		for (uint i = 0; i < mBaseObjectsPtr->size(); ++i) {
			Mat2x2 S = mBaseObjectsPtr->object(i).velocityCovarianceMatrix().topLeftCorner(2, 2);
			Eigen::SelfAdjointEigenSolver<Mat2x2> eigensolver(S);
			flt a = sqrt(eigensolver.eigenvalues()(0, 0));
			flt b = sqrt(eigensolver.eigenvalues()(1, 0));
//			mVelCovVecs1[i] = a * eigensolver.eigenvectors().col(0);
//			mVelCovVecs2[i] = b * eigensolver.eigenvectors().col(1);

			flt phi = atan2(eigensolver.eigenvectors().col(0)[1], eigensolver.eigenvectors().col(0)[0]);
			gui::computeEllipseContourPoints(mVelCovEllipse[i], head(mBaseObjectsPtr->object(i).position()), a, b, phi, 18.0 * math::D2R);
		}



	}

	mMutex.unlock();
}

void DisplayObjects::stopHook()
{

}

void DisplayObjects::draw3D(Painter3D::DrawArg)
{
	RTT::Logger::In in("DisplayObstacleInformation");

	mMutex.lock();

	if (!mBaseObjectsPtr) {
		mMutex.unlock();
		return;
	}

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPushMatrix();
	glEnable(GL_BLEND);
	glDisable(GL_LIGHTING);

	if (mDisplayBoundingBoxes) {
		for (uint i = 0; i < mBaseObjectsPtr->size(); ++i) {
			BaseObject const & object = mBaseObjectsPtr->object(i);

			displayBoundingBoxGrid(object.boundingBox(), Vec3(mPosColorR, mPosColorG, mPosColorB));
		}
	}

	if (mDisplayPositionCovariances && mPosCovVecs1.size() == mBaseObjectsPtr->size()) {
		glColor3d(mPosColorR, mPosColorG, mPosColorB);

		for (uint i = 0; i < mBaseObjectsPtr->size(); ++i) {
			BaseObject const & object = mBaseObjectsPtr->object(i);
			Vec3 const & pos = object.position();

			glBegin(GL_LINE_STRIP);
			glVertex3f(pos[0], pos[1], 0.0);
			glVertex3f(pos[0] + mPosCovVecs1[i][0], pos[1] + mPosCovVecs1[i][1], 0.0);
			glEnd();

			glBegin(GL_LINE_STRIP);
			glVertex3f(pos[0], pos[1], 0.0);
			glVertex3f(pos[0] + mPosCovVecs2[i][0], pos[1] + mPosCovVecs2[i][1], 0.0);
			glEnd();
		}

		for (uint i = 0; i < mPosCovEllipse.size(); ++i) {
			std::vector<Vec2> const & ellipse = mPosCovEllipse[i];

			glBegin(GL_LINE_STRIP);

			for (uint j = 0; j < ellipse.size(); ++j) {
				glVertex3f(ellipse[j][0], ellipse[j][1], 0.0);
			}

			glEnd();
		}
	}

	if (mDisplayVelocityCovariances && mVelCovVecs1.size() == mBaseObjectsPtr->size()) {
		glColor3d(mVelColorR, mVelColorG, mVelColorB);

		flt const Z = 0.1;

		for (uint i = 0; i < mBaseObjectsPtr->size(); ++i) {
			BaseObject const & object = mBaseObjectsPtr->object(i);
			Vec3 const & pos = object.position();

			if (mVelCovVecs1[i].squaredNorm() < 1.0) {
				glBegin(GL_LINE_STRIP);
				glVertex3f(pos[0], pos[1], Z);
				glVertex3f(pos[0] + mVelCovVecs1[i][0], pos[1] + mVelCovVecs1[i][1], Z);
				glEnd();
			}

			if (mVelCovVecs2[i].squaredNorm() < 1.0) {
				glBegin(GL_LINE_STRIP);
				glVertex3f(pos[0], pos[1], Z);
				glVertex3f(pos[0] + mVelCovVecs2[i][0], pos[1] + mVelCovVecs2[i][1], Z);
				glEnd();
			}
		}

		for (uint i = 0; i < mVelCovEllipse.size(); ++i) {
			std::vector<Vec2> const & ellipse = mVelCovEllipse[i];

			glBegin(GL_LINE_STRIP);

			for (uint j = 0; j < ellipse.size(); ++j) {
				glVertex3f(ellipse[j][0], ellipse[j][1], 0.0);
			}

			glEnd();
		}
	}


	if (mDisplayVelocityVectors) {
		glColor3d(mVelColorR, mVelColorG, mVelColorB);

		flt const Z = 0.1;

		for (uint i = 0; i < mBaseObjectsPtr->size(); ++i) {
			BaseObject const & object = mBaseObjectsPtr->object(i);
			Vec3 const & pos = object.position();
			Vec3 const & vel = object.velocity();

			glBegin(GL_LINE_STRIP);
			glVertex3f(pos[0], pos[1], Z);
			glVertex3f(pos[0] + vel[0], pos[1] + vel[1], Z);
			glEnd();
		}
	}

	glPopMatrix();
	glPopAttrib();

	mMutex.unlock();
}



DisplayObjects::~DisplayObjects()
{

}

