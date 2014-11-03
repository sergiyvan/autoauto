#include "ComfortSettings.h"

using namespace RTT;
using namespace math;

namespace aa
{
namespace modules
{
namespace nav
{

namespace controller
{


ComfortSettings::ComfortSettings()
	: RTT::PropertyBag()

	, mCentrifugalAccelerationComfort("CentrifugalAccelerationComfort", "comfort centrifugal acceleration value in m/s^2", 2.3) //2.3 //tested 1.5 // RACE 6.5, use throttle 0.8
	, mThrottleAccelerationComfort("ThrottleAccelerationComfort", "comfort throttle acceleration out of curves and stop signs in m/s^2", 2) //tested 1.0
	, mBrakingAccelerationComfort("BrakingAccelerationComfort", "comfort braking acceleration value for stop signs in m/s^2", -1.5) //-1.5) // //tested -1.0 // race -3.6
	, mBrakingAccelerationForObstacles("BrakingAccelerationForObstacles", "braking acceleration for sudden obstacles value in m/s^2", -4.0) // //tested -4.0 ?
	, mBrakingAccelerationForCurveEntries("BrakingAccelerationForCurveEntries", "braking acceleration for curve entries", -1.5)//-1.5) // race -3,6 //tested -1.0, !!!must be smaller than mBrakingAccelerationComfort!!!
	, mSampleCurvatureDistanceComfort("SampleCurvatureDistanceComfort", "sampling distance for curvatures comfort", 2.0)

//tire preserving
	, mCentrifugalAccelerationSporty("CentrifugalAccelerationSporty", "Sporty centrifugal acceleration value in m/s^2", 5.6) //2.3 //tested 1.5 // RACE 6.5, use throttle 0.8
	, mThrottleAccelerationSporty("ThrottleAccelerationSporty", "Sporty throttle acceleration out of curves and stop signs in m/s^2", 2) //tested 1.0
	, mBrakingAccelerationSporty("BrakingAccelerationSporty", "Sporty braking acceleration value for stop signs in m/s^2", -2.8) //-1.5) // //tested -1.0 // race -3.6
	, mBrakingAccelerationForObstaclesSporty("BrakingAccelerationForObstaclesSporty", "braking acceleration for sudden obstacles value in m/s^2", -4.0) // //tested -4.0 ?
	, mBrakingAccelerationForCurveEntriesSporty("BrakingAccelerationForCurveEntriesSporty", "braking acceleration for curve entries", -2.8)//-1.5) // race -3,6
	, mSampleCurvatureDistanceSporty("SampleCurvatureDistanceSporty", "sampling distance for curvatures sporty", 8.0)


	, mCentrifugalAccelerationUltimate("CentrifugalAccelerationUltimate", "Sporty centrifugal acceleration value in m/s^2", 8.0) //2.3 //tested 1.5 // RACE 6.5, use throttle 0.8
	, mThrottleAccelerationUltimate("ThrottleAccelerationUltimate", "Sporty throttle acceleration out of curves and stop signs in m/s^2", 2) //tested 1.0
	, mBrakingAccelerationUltimate("BrakingAccelerationUltimate", "Sporty braking acceleration value for stop signs in m/s^2", -3.6) //-1.5) // //tested -1.0 // race -3.6
	, mBrakingAccelerationForObstaclesUltimate("BrakingAccelerationForObstaclesUltimate", "braking acceleration for sudden obstacles value in m/s^2", -4.0) // //tested -4.0 ?
	, mBrakingAccelerationForCurveEntriesUltimate("BrakingAccelerationForCurveEntriesUltimate", "braking acceleration for curve entries", -3.6)//-1.5) // race -3,6 //tested -1.0, !!!must be smaller than mBrakingAccelerationComfort!!!
	, mSampleCurvatureDistanceUltimate("SampleCurvatureDistanceUltimate", "sampling distance for curvatures sporty", 8.0)

{
	addProperty(mCentrifugalAccelerationComfort);
	addProperty(mThrottleAccelerationComfort);
	addProperty(mBrakingAccelerationComfort);
	addProperty(mBrakingAccelerationForObstacles);
	addProperty(mBrakingAccelerationForCurveEntries);
	addProperty(mSampleCurvatureDistanceComfort);

	addProperty(mCentrifugalAccelerationSporty);
	addProperty(mThrottleAccelerationSporty);
	addProperty(mBrakingAccelerationSporty);
	addProperty(mBrakingAccelerationForObstaclesSporty);
	addProperty(mBrakingAccelerationForCurveEntriesSporty);
	addProperty(mSampleCurvatureDistanceSporty);

	addProperty(mCentrifugalAccelerationUltimate);
	addProperty(mThrottleAccelerationUltimate);
	addProperty(mBrakingAccelerationUltimate);
	addProperty(mBrakingAccelerationForObstaclesUltimate);
	addProperty(mBrakingAccelerationForCurveEntriesUltimate);
	addProperty(mSampleCurvatureDistanceUltimate);
}
}

}

}

}

