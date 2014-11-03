#include "ScannerSystem.h"

#include "RegionOfInterest.h"


#include <math/AutoMath.h>
#include <math/IbeoMath.h>



using namespace math;
using namespace aa::data::obstacle::util;

ScannerSystem::ScannerSystem()
	: mLocalToGlobalMatrix(Affine3::Identity())
	, mType(SENSOR_TYPE_UNKNOWN)
	, mScanFrequency(0.0)
	, mGlobalPosition(0.0, 0.0, 0.0)
	, mGlobalOrientation(1.0, 0.0, 0.0)
	, mMinDistOfLeftGroundPoints(1000.0)
	, mMinDistOfRightGroundPoints(1000.0)
	, mGroundPointsAngle(0.0)
	, mRoi()
{
//	mCarContour[0] = Vec3(-1.0, -1.0, 0.0);
//	mCarContour[1] = Vec3(-1.0, 1.0, 0.0);
//	mCarContour[2] = Vec3(1.0, 1.0, 0.0);
//	mCarContour[3] = Vec3(1.0, -1.0, 0.0);
}

ScannerSystem::ScannerSystem(
	Affine3 const & localToGlobal,
	SensorType type,
	flt scanFrequency,
	SharedRoi roi)
	: mLocalToGlobalMatrix(localToGlobal)
	, mGlobalPosition(localToGlobal.translation())
	, mType(type)
	, mScanFrequency(scanFrequency)
	, mMinDistOfLeftGroundPoints(1000.0)
	, mMinDistOfRightGroundPoints(1000.0)
	, mGroundPointsAngle(0.0)
	, mRoi(roi)
{
	for (uint i = 0; i < 3; ++i) {
		mGlobalOrientation(i) = mLocalToGlobalMatrix.matrix()(i, 0);
	}

//	computeCarContour();
}


bool ScannerSystem::isInsideOfSightField2D(Vec2 const & point) const
{
	bool inside = false;

	if (mRoi != NULL) {
		inside = mRoi->hasPoint(zeroExtend(point));

		if (!inside) {
			return false;
		}
	}

	for (unsigned int i = 0; i < this->size(); i++) {
		inside = this->at(i).isInsideOfSightField2D(point);

		if (inside) {
			return true;
		}
	}

	return false;
}


//void ScannerSystem::computeBlindRegion()
//{
//	if (this->empty()) {
//		return;
//	}
//	else if (this->size() == 1) {
//		flt const sqrt2 = sqrt(2.0);

//		mBlindRegion.clear();

//		Vec3 center = this->front().globalPosition();
//		Vec3 p1 =
//			ibeomath::localToGlobal(
//				Vec3(cos(this->front().endAngle()) * 100.0 / sqrt2,
//					 sin(this->front().endAngle()) * 100.0 / sqrt2,
//					 0.0),
//				this->front().localToGlobalMatrix());
//		Vec3 p2 =
//			ibeomath::localToGlobal(
//				Vec3(cos(this->front().startAngle() + 180.0 * math::D2R) * 100.0 / sqrt2,
//					 sin(this->front().startAngle() + 180.0 * math::D2R) * 100.0 / sqrt2,
//					 0.0),
//				this->front().localToGlobalMatrix());
//		Vec3 p3 =
//			ibeomath::localToGlobal(
//				Vec3(cos(this->front().endAngle() + 180.0 * math::D2R) * 100.0 / sqrt2,
//					 sin(this->front().endAngle() + 180.0 * math::D2R) * 100.0 / sqrt2,
//					 0.0),
//				this->front().localToGlobalMatrix());
//		Vec3 p4 =
//			ibeomath::localToGlobal(
//				Vec3(cos(this->front().startAngle()) * 100.0 / sqrt2,
//					 sin(this->front().startAngle()) * 100.0 / sqrt2,
//					 0.0),
//				this->front().localToGlobalMatrix());

//		// Trinagle 1
//		mBlindRegion.push_back(center);
//		mBlindRegion.push_back(p1);
//		mBlindRegion.push_back(p2);
//		// Trinagle 2
//		mBlindRegion.push_back(center);
//		mBlindRegion.push_back(p2);
//		mBlindRegion.push_back(p3);
//		// Trinagle 3
//		mBlindRegion.push_back(center);
//		mBlindRegion.push_back(p3);
//		mBlindRegion.push_back(p4);
//	}
//	else {
//		flt const sqrt2 = sqrt(2.0);

//		mBlindRegion.clear();

//		for (unsigned int i1 = 0; i1 < this->size(); i1++) {
//			Vec2 s1_p1 = math::head(this->at(i1).globalPosition());
//			Vec2 s1_p2 = math::head(
//							 ibeomath::localToGlobal(
//								 Vec3(cos(this->at(i1).endAngle()) * 100.0 / sqrt2, sin(this->at(i1).endAngle()) * 100.0 / sqrt2, 0.0),
//								 this->at(i1).localToGlobalMatrix()));

//			unsigned int i2 = (i1 + 1) % this->size();

//			Vec2 s2_p1 = math::head(this->at(i2).globalPosition());
//			Vec2 s2_p2 = math::head(
//							 ibeomath::localToGlobal(
//								 Vec3(cos(this->at(i2).startAngle()) * 100.0 / sqrt2, sin(this->at(i2).startAngle()) * 100.0 / sqrt2, 0.0),
//								 this->at(i2).localToGlobalMatrix()));

//			Vec2 cross;
//			bool ret = math::geradeStreckenSchnitt(s1_p1, s1_p2, s2_p1, s2_p2, cross);



//			if (ret) {
//				mBlindRegion.push_back(this->at(i1).globalPosition());
//				mBlindRegion.push_back(
//					Vec3(cross[0], cross[1], (this->at(i1).globalPosition()[2] + this->at(i2).globalPosition()[2]) / 2.0));
//				mBlindRegion.push_back(this->at(i2).globalPosition());
//			}
//			else {
//				if (!ibeomath::isLeftTo(s2_p2 - s2_p1, s1_p2 - s2_p1)) {
//					mBlindRegion.push_back(this->at(i1).globalPosition());
//					mBlindRegion.push_back(Vec3(s1_p2[0], s1_p2[1], this->at(i1).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s2_p2[0], s2_p2[1], this->at(i2).globalPosition()[2]));
//					mBlindRegion.push_back(this->at(i1).globalPosition());
//					mBlindRegion.push_back(Vec3(s2_p2[0], s2_p2[1], this->at(i2).globalPosition()[2]));
//					mBlindRegion.push_back(this->at(i2).globalPosition());
//				}
//				else {
//					Vec2 s1_p3 = math::head(
//									 ibeomath::localToGlobal(
//										 -Vec3(cos(this->at(i1).endAngle()) * 100.0 / sqrt2, sin(this->at(i1).endAngle()) * 100.0 / sqrt2, 0.0),
//										 this->at(i1).localToGlobalMatrix()));
//					Vec2 s2_p3 = math::head(
//									 ibeomath::localToGlobal(
//										 -Vec3(cos(this->at(i2).startAngle()) * 100.0 / sqrt2, sin(this->at(i2).startAngle()) * 100.0 / sqrt2, 0.0),
//										 this->at(i2).localToGlobalMatrix()));

//					mBlindRegion.push_back(Vec3(s1_p1[0], s1_p1[1], this->at(i1).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s1_p2[0], s1_p2[1], this->at(i1).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s2_p3[0], s2_p3[1], this->at(i2).globalPosition()[2]));

//					mBlindRegion.push_back(Vec3(s1_p1[0], s1_p1[1], this->at(i1).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s2_p3[0], s2_p3[1], this->at(i2).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s1_p3[0], s1_p3[1], this->at(i1).globalPosition()[2]));

//					mBlindRegion.push_back(Vec3(s1_p1[0], s1_p1[1], this->at(i1).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s1_p3[0], s1_p3[1], this->at(i1).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s2_p1[0], s2_p1[1], this->at(i2).globalPosition()[2]));

//					mBlindRegion.push_back(Vec3(s2_p1[0], s2_p1[1], this->at(i2).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s1_p3[0], s1_p3[1], this->at(i1).globalPosition()[2]));
//					mBlindRegion.push_back(Vec3(s2_p2[0], s2_p2[1], this->at(i2).globalPosition()[2]));
//				}
//			}
//		}
//	}
//}


//void ScannerSystem::computeFieldOfView(unsigned int n, vgl_clip_type clipType)
//{
//	if (this->empty()) {
//		return;
//	}

//	int num = std::min((unsigned int)(this->size()), n);

//	if (num == 1) {
//		mFieldOfView = this->front().fieldOfView();
//		return;
//	}

//	mFieldOfView = this->front().fieldOfView();

//	for (int i = 1; i < num; i++) {
//		if (clipType == vgl_clip_type_union) {
//			for (unsigned int s = 0; s < this->at(i).fieldOfView().num_sheets(); ++s) {
//				mFieldOfView.push_back(this->at(i).fieldOfView()[s]);
//			}
//		}
//		else {
//			mFieldOfView = vgl_polygon<flt>(vgl_clip(mFieldOfView, this->at(i).fieldOfView(), clipType));
//		}
//	}
//}

\
flt ScannerSystem::occludedProbability(Vec3 const & p) const
{
	flt prob = 1.0;

	for (uint i = 0; i < this->size(); ++i) {
		prob *= this->at(i).occlusionMap()->occludedProbability(p);
		// 		prob *= this->at(i).occlusionMap()->occludedProbability(zeroExtend(p));

	}

	return prob;
}


//void ScannerSystem::computeCarContour()
//{
//	const flt lPositionToLeft   = 1.0;
//	const flt lPositionToRight  = 1.0;
//	const flt lPositionToFront = 1.2;
//	const flt lPositionToRear  = 4.0;

//	Vec3 dirForward 	= mGlobalOrientation;
//	Vec3 dirTangential 	= Vec3(dirForward[1], -dirForward[0], 0.0);
//	Vec3 const & position 		= mGlobalPosition;
//	dirForward[2] = 0.0;

//	mCarContour[0] = position - lPositionToRear * dirForward  - lPositionToLeft * dirTangential;
//	mCarContour[1] = position - lPositionToRear * dirForward  + lPositionToRight * dirTangential;
//	mCarContour[2] = position + lPositionToFront * dirForward + lPositionToRight * dirTangential;
//	mCarContour[3] = position + lPositionToFront * dirForward - lPositionToLeft * dirTangential;
//}

//void ScannerSystem::computeLocalToGlobalMatrix(TimedEgoState const & egostate)
//{
//	mLocalToGlobalMatrix = egostate.localToGlobal() * getScannerMatrix();
//}
//
//void ScannerSystem::computeGlobalPosition()
//{
//}
//
//void ScannerSystem::computeGlobalOrientation(Vec3 const & direction)
//{
//	Vec2 systemOrientation = math::head(direction);
//	flt yaw = mRotationImuOffset[2];
//	mGlobalOrientation2D = normalized(Vec2(systemOrientation[0] * cos(yaw) - systemOrientation[1] * sin(yaw),
//										   systemOrientation[0] * sin(yaw) + systemOrientation[1] * cos(yaw)));
//
//}
//
//Affine3 ScannerSystem::getScannerMatrix() const
//{
//#if defined(USE_EIGEN)
//	return ::math::fromPositionRpy(mPositionImuOffset, mRotationImuOffset);
//#else
//	Affine3 matScanner;
//	matScanner.setIdentity();
//	matScanner.set_rotation_roll_pitch_yaw(mRotationImuOffset[0], mRotationImuOffset[1], mRotationImuOffset[2]);
//	matScanner.set_translation(mPositionImuOffset[0], mPositionImuOffset[1], mPositionImuOffset[2]);
//	return matScanner;
//#endif
//}
