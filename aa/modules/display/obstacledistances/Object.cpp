/**
 *  \brief This file contains a class for obstacle representations.
 *  \author Sebastian Hempel
 */

#include <vector>
#include <GL/gl.h>

#include "Object.h"

using namespace aa::modules::display::obstacledistances;


ObstacleObject::ObstacleObject()
{

}


ObstacleObject::ObstacleObject(
	const Obstacle & obstacle,
	const math::flt safetyDistance)
	: mContour(obstacle.contour())
{
	simpleSafetyArea(safetyDistance);
}


void ObstacleObject::render(
	const math::flt z,
	const bool renderObstacle_,
	const Color & obstacleColor,
	const bool renderSafetyArea_,
	const Color & boundaryColor,
	const Color & borderColor,
	const bool renderBackFace)
{
	if (renderObstacle_) {
		renderObstacle(z, obstacleColor);
	}

	if (renderSafetyArea_) {
		renderSafetyArea(z, boundaryColor, borderColor, renderBackFace);
	}
}


void ObstacleObject::renderObstacle(
	const math::flt z,
	const Color & color)
{
	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_POLYGON);
	{
		for (Contour::const_iterator it = mContour.begin();
				it != mContour.end();
				++it) {
			const math::Vec2 vertex = *it;
			glVertex3f(vertex[0], vertex[1], z);
		}
	}
	glEnd();
}


void ObstacleObject::renderSafetyArea(
	const math::flt z,
	const Color & boundaryColor,
	const Color & borderColor,
	const bool renderBackFace)
{
	glPushAttrib(GL_LIGHTING);
	glDisable(GL_LIGHTING);

	//TODO: (?) colors by triangle
	glColor3f(boundaryColor[0], boundaryColor[1], boundaryColor[2]);

	glBegin(GL_TRIANGLES);
	{
		for (Area::iterator it = mArea.begin(); it != mArea.end(); ++it) {
			Triangle & triangle = *it;

			triangle.render(z, renderBackFace);
		}
	}
	glEnd();

	glPopAttrib();
}


const math::Vec3 twoToThree(const math::Vec2 & v)
{
	return math::Vec3(v[0], v[1], 0.0);
}


void ObstacleObject::simpleSafetyArea(const math::flt safetyDistance)
{
	const size_t size = mContour.size();

	if (size == 1) {
		circle(twoToThree(mContour[0]),
			   safetyDistance);
	}
	else {
		for (size_t i = 0; i < size - 1; ++i) {
			const math::Vec3 & a = twoToThree(mContour[i]);
            const math::Vec3 & b = twoToThree(mContour[i + 1]);
            if ((b - a) == math::Vec3(0,0,0)) continue;
            const math::Vec3 dir = (b - a).normalized();
            const math::Vec3 ortho = math::Vec3(-dir[1], dir[0], 0);
			const math::Vec3 side = ortho * safetyDistance;

			//create box on the left side
			const math::Vec3 boxLeft[] = {a, a + side, b + side, b};
			pushBox(boxLeft);

			//create box on the right side
			const math::Vec3 boxRight[] = {a, a - side, b - side, b};
			pushBox(boxRight);

			if (i < size - 2) {
				const math::Vec3 & c = twoToThree(mContour[i + 2]);
                if ((c - b) == math::Vec3(0,0,0)) continue;
                const math::Vec3 dir2 = (c - b).normalized();

                const math::flt cosBeta = ortho[0] * dir2[0] + ortho[1] * dir2[1];
				const math::flt cosAlpha = dir[0] * dir2[0] + dir[1] * dir2[1];

				if (cosBeta < 0) {
					//left open
					math::flt angle = (cosAlpha > 0)
									  ? acos(cosAlpha)
									  : 2 * M_PI - acos(cosBeta) - M_PI / 2;

                    segment(b,
							dir, ortho,
							angle, safetyDistance, Triangle::SEGMENT);
				}
				else {
					//right open
					math::flt angle = (cosAlpha > 0)
									  ? acos(cosAlpha)
									  : 2 * M_PI - acos(-cosBeta) - M_PI / 2;

                    segment(b,
							dir, -ortho,
							angle, safetyDistance, Triangle::SEGMENT);
				}
			}
		}

		pushEnd(twoToThree(mContour[0]),
				twoToThree(mContour[1]),
				safetyDistance);

		pushEnd(twoToThree(mContour[size - 1]),
				twoToThree(mContour[size - 2]),
				safetyDistance);
	}
}


void ObstacleObject::segment(
	const math::Vec3 & center,
	const math::Vec3 & dir,
	const math::Vec3 & ortho,
	const math::flt angle,
	const math::flt safetyDistance,
	const Triangle::Type t,
	const bool forceFront)
{
	const math::flt length = 2 * angle * safetyDistance;
	const math::flt stepSize = 0.3;
	const size_t numSteps = ceil(length / stepSize);

	for (size_t i = 0; i < numSteps; ++i) {
		const math::flt an = i * angle / numSteps;
		const math::flt an2 = (i + 1) * angle / numSteps;

		const math::Vec3 x =
			(ortho * cos(an) + dir * sin(an)) * safetyDistance;
		const math::Vec3 y =
			(ortho * cos(an2) + dir * sin(an2)) * safetyDistance;

		//the step is small
		const math::Vec3 n = x + y;

		push_triangle(center, x + center, y + center, t, n, forceFront);
	}
}


void ObstacleObject::circle(
	const math::Vec3 & center,
	const math::flt safetyDistance)
{
	segment(center,
			math::Vec3(1, 0, 0),
			math::Vec3(0, 1, 0),
			2 * M_PI,
			safetyDistance,
			Triangle::CIRCLE);
}


void ObstacleObject::pushEnd(
	const math::Vec3 & a,
	const math::Vec3 & b,
	const math::flt safetyDistance)
{
	const math::Vec3 & dir = (b - a).normalized();
	const math::Vec3 & ortho = math::Vec3(-dir[1], dir[0], 0);

	segment(a, -dir, ortho, M_PI, safetyDistance, Triangle::END);
}


void ObstacleObject::pushBox(math::Vec3 const * const box)
{
	const math::Vec3 n = box[1] - box[0];

	push_triangle(box[0], box[1], box[2], Triangle::BOX, n);
	push_triangle(box[0], box[2], box[3], Triangle::BOX, n);
}


ObstacleObject::Triangle::Triangle()
{

}


ObstacleObject::Triangle::Triangle(
	const Vertex & a,
	const Vertex & b,
	const Vertex & c,
	const Type t,
	const Face f)
	: mType(t)
	, mFace(f)
{
	mVertices[0] = a;
	mVertices[1] = b;
	mVertices[2] = c;
}


const ObstacleObject::Triangle::Vertex & ObstacleObject::Triangle::vertex(
	const size_t index) const
{
	return mVertices[index];
}


void ObstacleObject::Triangle::render()
{
	for (size_t i = 0; i < SIZE; ++i) {
		const Vertex & v = mVertices[i];

		glVertex3d(v[0], v[1], v[2]);
	}
}


void ObstacleObject::Triangle::render(
	const math::flt z,
	const bool renderBackFace)
{
	if (isFront() || renderBackFace) {
		for (size_t i = 0; i < SIZE; ++i) {
			const Vertex & v = mVertices[i];

			glVertex3d(v[0], v[1], z);
		}
	}
}


void ObstacleObject::push_triangle(
	const math::Vec3 & first,
	const math::Vec3 & second,
	const math::Vec3 & third,
	const Triangle::Type t,
	const math::Vec3 & normal,
	const bool forceFront)
{
	const math::Vec2 & r = mContour.referencePoint();

	const bool test =
		forceFront ||
		testIsFrontFace(second, normal, r) ||
		testIsFrontFace(third, normal, r) ||
		testIsFrontFace(first, normal, r);

	const Triangle::Face face = test ? Triangle::FRONT : Triangle::BACK;

	mArea.push_back(Triangle(first, second, third, t, face));
}


bool ObstacleObject::testIsFrontFace(
	const math::Vec3 & point,
	const math::Vec3 & normal,
	const math::Vec2 & ref) const
{
	const math::Vec2 d(ref[0] - point[0], ref[1] - point[1]);
	const math::flt dp = d[0] * normal[0] + d[1] * normal[1];

	return dp > 0.0;
}
