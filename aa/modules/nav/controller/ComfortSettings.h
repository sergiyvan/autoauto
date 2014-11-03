#pragma once
#include <rtt/PropertyBag.hpp>
#include <rtt/Property.hpp>
#include <patterns/Singleton.h>

#include <math/AutoMath.h>

namespace aa
{
namespace modules
{
namespace nav
{

namespace controller
{

class ComfortSettings
	: public RTT::PropertyBag
{
public:
	typedef ::math::flt flt;

	ComfortSettings();

	/** @name Properties: general properties */
	RTT::Property<flt> mCentrifugalAccelerationComfort;
	RTT::Property<flt> mThrottleAccelerationComfort;
	RTT::Property<flt> mBrakingAccelerationComfort;
	RTT::Property<flt> mBrakingAccelerationForObstacles;
	RTT::Property<flt> mBrakingAccelerationForCurveEntries;
	RTT::Property<flt> mSampleCurvatureDistanceComfort;

	RTT::Property<flt> mCentrifugalAccelerationSporty;
	RTT::Property<flt> mThrottleAccelerationSporty;
	RTT::Property<flt> mBrakingAccelerationSporty;
	RTT::Property<flt> mBrakingAccelerationForObstaclesSporty;
	RTT::Property<flt> mBrakingAccelerationForCurveEntriesSporty;
	RTT::Property<flt> mSampleCurvatureDistanceSporty;

	RTT::Property<flt> mCentrifugalAccelerationUltimate;
	RTT::Property<flt> mThrottleAccelerationUltimate;
	RTT::Property<flt> mBrakingAccelerationUltimate;
	RTT::Property<flt> mBrakingAccelerationForObstaclesUltimate;
	RTT::Property<flt> mBrakingAccelerationForCurveEntriesUltimate;
	RTT::Property<flt> mSampleCurvatureDistanceUltimate;

};

typedef patterns::Singleton<ComfortSettings> theComfortSettings;

}

}

}

}

