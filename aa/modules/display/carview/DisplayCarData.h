#pragma once
#include <gui/Painter3DTask.h>

#include <core/TimedData.h>
#include <qmutex.h>

#include <modules/models/egostate/EgoState.h>
#include <modules/models/carstate/CarState.h>
#include <modules/models/carstate/AuxDevicesData.h>

namespace aa
{
namespace modules
{
namespace display
{
namespace carview
{

class DisplayCarData
	: public gui::Painter3DTask
{
	class impl;
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Vec4 Vec4;
	typedef ::math::Mat4x4 Mat4x4;
	explicit DisplayCarData(std::string const & name);
	virtual ~DisplayCarData();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	virtual void init3D(SceneNodePtr);
	virtual void draw3D(DrawArg);

	void draw3D2();


	bool setCamPosition(flt x, flt y, flt z);
	bool setCamAttitude(flt roll, flt pitch, flt yaw);
	bool setCamViewDistance(flt dist);

protected:
	bool inited;

	/// Properties
	RTT::Property<int> mDrawOverlays;
	RTT::Property<bool> mDrawDepthView;
	RTT::Property<bool> mDrawGpsErrorHalo;
	RTT::Property<bool> mDrawAuxDevices;
	RTT::Property<std::string> mCarModelFilename;



	/// Ports
	RTT::InputPort< int > mCarGear;
	RTT::InputPort< std::vector<std::pair<Vec3, Vec3> > > mVectors;
	RTT::InputPort< std::vector<boost::tuple<Mat4x4, std::string, Vec4> > > mGlobals;
	RTT::InputPort< std::string > mInfos;
	RTT::InputPort< AuxDevicesData > mAuxDevices;

	RTT::InputPort< TimedEgoState > mEgoState;
	RTT::InputPort< ::modules::models::carstate::TimedCarState> mCarState;


	Vec3 mLastCarPosition;
	Vec3 mLastDirection;
	int mTurnSignalCounter;

private:
	boost::shared_ptr<impl> pimpl;
};

}
}
}
}
