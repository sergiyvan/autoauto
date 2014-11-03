#pragma once
#include <vector>
#include <boost/tuple/tuple.hpp>
#include <math/LaneSpline.h>

/// \return tuple of the squared distance and parameter at the closest position
boost::tuple< ::math::flt, ::math::flt> findClosestPoint(math::LaneSpline const & spline, ::math::flt from, ::math::flt to, ::math::Vec2 const & point);

/**
  * \param markings Stores the points of the lane-markings on the line given by pos and orthornormal
  *				may be empty, when no lane-markings are found within a certain distance
  * \param pos the position of the observer
  * \param on the othononomal of the line , or the direction of line of sight of the observer
  */
void laneMarkings(std::vector< ::math::Vec2> & markings,  ::math::Vec2 const & pos, ::math::Vec2 const & on);

::math::flt maxCurvature(math::LaneSpline const & spline, ::math::flt from, ::math::flt to);
::math::flt minRadius(math::LaneSpline const & spline, ::math::flt from, ::math::flt to);
