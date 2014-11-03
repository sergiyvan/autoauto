#pragma once
/*!
 * \file Util.h
 * \brief Display utilities.
 * \author Michael Schnürmacher
 */

#include <data/geometry/BoundingBox3D.h>
#include <data/geometry/BoundingBox2D.h>


namespace aa
{
namespace modules
{
namespace display
{
namespace utilities
{

class Color
{
public:
	static const math::Vec3 RED;
	static const math::Vec3 GREEN;
	static const math::Vec3 BLUE;
	static const math::Vec3 PINK;
	static const math::Vec3 WHITE;
	static const math::Vec3 YELLOW;
	static const math::Vec3 GREY;
	static const math::Vec3 LIGHT_GREY;
	static const math::Vec3 ORANGE;
	static const math::Vec3 LIGHT_BLUE;
	static const math::Vec3 MAGENTA;
	static const math::Vec3 BLACK;
};

void displayBoundingBox2D(::data::geometry::BoundingBox2D const & box, math::Vec3 const & color, math::flt z = 0.0);
void displayBoundingBox2DSolid(::data::geometry::BoundingBox2D const & box, math::Vec3 const & color, math::flt t, math::flt z = 0.0);
void displayBoundingBoxGrid(::data::geometry::BoundingBox3D const & box, math::Vec3 const & color);
void displayBoundingBoxSolid(::data::geometry::BoundingBox3D const & box, math::Vec3 const & color, math::flt transparency);
void displaySolidCircle(math::Vec3 const & center, math::flt radius, math::Vec3 const & color, math::flt transparency, math::flt maxPointDistance = 0.01);
void displayId(const math::Vec3 & color, math::Vec3 const & position, uint id);
void displayFootboard(::data::geometry::BoundingBox2D const & box, math::Vec3 const & color, uint steps, math::flt z = 0.0);
math::Vec3 makeColor(uint id);
uint hash(uint key);
void displayUnfilledRectangle(math::Vec3 const & color,	math::flt alpha, math::Vec3 const & p1, math::Vec3 const & p2, math::Vec3 const & p3, math::Vec3 const & p4);
void displayFilledRectangle(math::Vec3 const & color,	math::flt alpha, math::Vec3 const & p1, math::Vec3 const & p2, math::Vec3 const & p3, math::Vec3 const & p4);
/* Now in PointsSplitter
void displayBox(modules::vision::utils::PointsSplitter::Part3D const & part3D, math::Vec3 const & color, math::flt const t = 1, const bool solid = false);
void displaySplitterPartSolid(modules::vision::utils::PointsSplitter::Part3D const & part3D, math::Vec3 const & color, math::flt t, math::flt z); */

}
}
}
}