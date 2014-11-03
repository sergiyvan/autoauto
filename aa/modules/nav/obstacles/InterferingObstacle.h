#pragma once

#include <aa/data/obstacle/BaseObstacle.h>
#include <aa/data/obstacle/util/Contour.h>

#include <aa/modules/nav/controller/Plan.h>

namespace aa
{
namespace modules
{
namespace nav
{

namespace obstacles
{

template<class _Spline, class _ObstacleBundle>
class InterferingObstacle_
{
public:
	typedef _Spline SplineType;

	typedef typename SplineType::Img Image;
	typedef typename SplineType::Dom Domain;
	typedef typename _ObstacleBundle::value_type BaseObstacle;


	InterferingObstacle_(BaseObstacle * o, Image contourPoint, Domain param = -1, Domain leftDistance = 0, Domain rightDistance = 0)
		: baseObstacle(o)
		, interferingContourPoint(contourPoint)
		, obstParam(param)
		, leftDistance(leftDistance)
		, rightDistance(rightDistance) {
	}

	~InterferingObstacle_() {
	}

	bool operator==(InterferingObstacle_<_Spline, _ObstacleBundle> const & o) const {
		return baseObstacle == o.baseObstacle
			   && interferingContourPoint == o.interferingContourPoint
			   && obstParam == o.obstParam;
	}

	BaseObstacle * baseObstacle;

	Image interferingContourPoint;
	Domain obstParam;

	/**
	 * the distance of the rightmost point from the obstacle to the spline
	 * <0 left of spline, >0 is right on SplineType
	 *
	 * You can check if the obstacle is on the spline with:

		if (rightDistance < 0) { //on the left
			...
		}
		else if (leftDistance > 0) { //on the right
			...
		}
		else { //on both sides of the spline
			...
		}
	 */
	Domain rightDistance;
	/// analogous to rightDistance
	Domain leftDistance;
};

}


}

 // modules
}

 // nav
}

 // obstacles
