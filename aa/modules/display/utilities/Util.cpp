/*!
 * \file "Util.h"
 * \brief Display utilities.
 * \author Michael Schnürmacher
 */

#include "Util.h"

#include <GL/gl.h>
#include <gui/GlTools.h>
#include <math/IbeoMath.h>

using namespace math;
using namespace gui;

namespace aa
{
namespace modules
{
namespace display
{
namespace utilities
{

const math::Vec3 Color::RED(1.0, 0.0, 0.0);
const math::Vec3 Color::GREEN(0.0, 1.0, 0.0);
const math::Vec3 Color::BLUE(0.0, 0.0, 1.0);
const math::Vec3 Color::PINK(1.0, 0.0, 1.0);
const math::Vec3 Color::WHITE(1.0, 1.0, 1.0);
const math::Vec3 Color::YELLOW(1.0, 1.0, 0.0);
const math::Vec3 Color::GREY(0.5, 0.5, 0.5);
const math::Vec3 Color::LIGHT_GREY(0.75, 0.75, 0.75);
const math::Vec3 Color::ORANGE(1.0, 0.5, 0.0);
const math::Vec3 Color::LIGHT_BLUE(0.0, 1.0, 1.0);
const math::Vec3 Color::MAGENTA(1.0, 0.0, 1.0);
const math::Vec3 Color::BLACK(0.0, 0.0, 0.0);


void displayFootboard(::data::geometry::BoundingBox2D const & box, math::Vec3 const & color, uint steps, math::flt z)
{
	glColor3f(color[0], color[1], color[2]);

	glBegin(GL_LINE_STRIP);

	for (uint i = 0; i < 5; i++) {
		glVertex3f(box.vertex(i % 4)[0], box.vertex(i % 4)[1], z);
	}

	glEnd();

	flt step = sqrt(ssd(box.vertex(0), box.vertex(1))) / flt(steps);
	Vec2 dir = (box.vertex(1) - box.vertex(0)).normalized();

	for (uint i = 0; i < steps; ++i) {
		Vec2 l = box.vertex(0) + i * step * dir;
		Vec2 r = box.vertex(3) + i * step * dir;
		glBegin(GL_LINE_STRIP);
		glVertex3f(l[0], l[1], z);
		glVertex3f(r[0], r[1], z);
		glEnd();
	}

	step = sqrt(ssd(box.vertex(1), box.vertex(2))) / flt(steps);
	dir = (box.vertex(2) - box.vertex(1)).normalized();

	for (uint i = 0; i < steps; ++i) {
		Vec2 l = box.vertex(1) + i * step * dir;
		Vec2 r = box.vertex(0) + i * step * dir;
		glBegin(GL_LINE_STRIP);
		glVertex3f(l[0], l[1], z);
		glVertex3f(r[0], r[1], z);
		glEnd();
	}

}

void displayBoundingBox2D(::data::geometry::BoundingBox2D const & box, math::Vec3 const & color, flt z)
{
	glColor3f(color[0], color[1], color[2]);

	glBegin(GL_LINE_STRIP);

	for (uint i = 0; i < 5; i++) {
		glVertex3f(box.vertex(i % 4)[0], box.vertex(i % 4)[1], z);
	}

	glEnd();
}


void displayBoundingBox2DSolid(::data::geometry::BoundingBox2D const & box, math::Vec3 const & color, flt t, flt z)
{
	glColor4f(color[0], color[1], color[2], t);

	glBegin(GL_QUADS);

	for (uint i = 0; i < 4; i++) {
		glVertex3f(box.vertex(i)[0], box.vertex(i)[1], z);
	}

	glEnd();
}


void displayBoundingBoxGrid(::data::geometry::BoundingBox3D const & box, Vec3 const & color)
{
	glColor3f(color[0], color[1], color[2]);

	//top and bottom
	for (int j = 0; j < 2; j++) {
		glBegin(GL_LINE_STRIP);

		for (int i = 0; i < 5; i++) {
			int ix = i % 4;
			glVertex3f(box.vertex(ix, j)[0], box.vertex(ix, j)[1], box.vertex(ix, j)[2]);
		}

		glEnd();
	}

	//vertical edges
	glBegin(GL_LINES);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 2; j++) {
			glVertex3f(box.vertex(i, j)[0], box.vertex(i, j)[1], box.vertex(i, j)[2]);
		}
	}

	glEnd();
}


void displayBoundingBoxSolid(::data::geometry::BoundingBox3D const & box, Vec3 const & color, flt transparency)
{
	int sides = 4;

	glBegin(GL_QUADS);
	{
		//sides of the obstacle
		for (unsigned int side = 0; side < sides; ++side) {
			unsigned int i = side,
			             j = (side + 1) % sides;

			glColor4f(color[0], color[1], color[2], transparency);
			renderVnl(box.vertex(i, 0));
			renderVnl(box.vertex(j, 0));

			glColor4f(color[0], color[1], color[2], transparency);
			renderVnl(box.vertex(j, 1));
			renderVnl(box.vertex(i, 1));
		}

		//bottom
		glColor4f(color[0], color[1], color[2], transparency);

		for (unsigned int side = 0; side < sides; ++side) {
			renderVnl((box.vertex(side, 0)));
		}

		//top
		glColor4f(color[0], color[1], color[2], transparency);

		for (unsigned int side = 0; side < sides; ++side) {
			renderVnl((box.vertex(side, 0)));
		}
	}
	glEnd();
}


void displaySolidCircle(Vec3 const & center, flt radius, Vec3 const & color, flt transparency, flt maxPointDistance)
{
	//render a triangle at least
	const int mMinPoints = 3;

	const ::math::flt u = 2.0f * M_PI * radius;
	const unsigned int numPoints = std::max(mMinPoints,
	                                        ((int)(u / maxPointDistance)) + 1);

	glBegin(GL_QUAD_STRIP);
	{
		glColor4f(color[0], color[1], color[2], transparency);

		for (unsigned int i = 0; i <= numPoints; ++i) {
			flt alpha = 2.0f * M_PI * i / numPoints;
			flt cosA  = cos(alpha);
			flt sinA  = sin(alpha);

			glVertex3f(center[0] + 0.01 * cosA,
			           center[1] + 0.01 * sinA,
			           center[2]);

			glVertex3f(center[0] + radius * cosA,
			           center[1] + radius * sinA,
			           center[2]);
		}
	}
	glEnd();
}


void displayId(const Vec3 & color, Vec3 const & position, uint id)
{
	std::string idStr = "";
	math::ibeomath::append(idStr, id);
	gui::renderString(&idStr[0], position, color[0], color[1], color[2]);
}


math::Vec3 makeColor(uint id)
{
	flt fMax = 255.0f;
	uint hr = hash(id);
	uint hg = hash(hr);
	uint hb = hash(hg);
	uint iMax = fMax + 1;

	return Vec3((hr % iMax) / fMax, (hg % iMax) / fMax, (hb % iMax) / fMax);
}


uint hash(uint key)
{
	key = (key + 0x7ed55d16) + (key << 12);
	key = (key ^ 0xc761c23c) ^ (key >> 19);
	key = (key + 0x165667b1) + (key << 5);
	key = (key + 0xd3a2646c) ^ (key << 9);
	key = (key + 0xfd7046c5) + (key << 3);
	key = (key ^ 0xb55a4f09) ^ (key >> 16);

	return key;
}

void displayUnfilledRectangle(math::Vec3 const & color, math::flt alpha, math::Vec3 const & p1, math::Vec3 const & p2, math::Vec3 const & p3, math::Vec3 const & p4)
{
	glColor4f(color[0], color[1], color[2], alpha);
	glBegin(GL_LINE_STRIP);
	renderVnl(p1);
	renderVnl(p2);
	renderVnl(p4);
	renderVnl(p3);
	renderVnl(p1);
	glEnd();
}


void displayFilledRectangle(math::Vec3 const & color, math::flt alpha, math::Vec3 const & p1, math::Vec3 const & p2, math::Vec3 const & p3, math::Vec3 const & p4)
{
	glBegin(GL_TRIANGLES);
	{
		glColor4f(color[0], color[1], color[2], alpha);
		renderVnl(p1);
		renderVnl(p2);
		renderVnl(p3);
	}
	glEnd();

	glBegin(GL_TRIANGLES);
	{
		glColor4f(color[0], color[1], color[2], alpha);
		renderVnl(p2);
		renderVnl(p3);
		renderVnl(p4);
	}
	glEnd();
}

}
}
}
}