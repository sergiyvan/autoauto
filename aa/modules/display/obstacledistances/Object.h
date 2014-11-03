/**
 *  \brief This file contains a class for obstacle representations.
 *  \author Sebastian Hempel
 */

#pragma once

#include <aa/data/obstacle/BaseObstacleBundle.h>



namespace aa
{
namespace modules
{
namespace display
{
namespace obstacledistances
{

class ObstacleObject
{
public:
	/**
	 * shortcut to obstacles
	 */
	typedef aa::data::obstacle::BaseObstacle Obstacle;


	/**
	 *  shortcut to contours
	 */
	typedef Obstacle::Contour Contour;


	/**
	 *  shortcut for colors
	 */
	typedef math::Vec3f Color;


	ObstacleObject();


	/**
	 *  \brief Creates object from contour.
	 *  \param[in] Obstacle obstacle
	 *  \param[in] safetyDistance safety distance
	 */
	ObstacleObject(const Obstacle & obstacle, const math::flt safetyDistance);


	/**
	 *  \brief Renders the object.
	 *  \param[in] z vertical position of the obstacles
	 *  \param[in] renderObstacle render obstacle?
	 *  \param[in] obstacleColor color of the obstacle
	 *  \param[in] renderSafetyArea render safety area?
	 *  \param[in] boundaryColor color of the boundary
	 *  \param[in] borderColor color of the boundary
	 *  \param[in] renderBackFace render back face of obstacles?
	 */
	void render(const math::flt z,
				const bool renderObstacle,
				const Color & obstacleColor,
				const bool renderSafetyArea,
				const Color & boundaryColor,
				const Color & borderColor,
				const bool renderBackFace);


protected:
	/**
	 *  \brief Triangle for rendering.
	 */
	class Triangle
	{
	public:
		enum {
			/**
			 *  number of vertices
			 */
			SIZE = 3
		};


		enum Type {
			BOX,
			END,
			CIRCLE,
			SEGMENT
		};


		enum Face {
			FRONT,
			BACK
		};


		/**
		 *  shortcut for vertices
		 */
		typedef math::Vec3 Vertex;


		Triangle();


		/**
		 *  \param[in] a first coordinate
		 *  \param[in] b second coordinate
		 *  \param[in] c third coordinate
		 *  \param[in] t type of the triangle
		 */
		Triangle(const Vertex & a,
				 const Vertex & b,
				 const Vertex & c,
				 const Type t,
				 const Face f);


		/**
		 *  \brief Acces to triangle vertices.
		 *  \param[in] index index of a vertex
		 */
		const Vertex & vertex(const size_t index) const;


		/**
		 *  \brief Renders the triangle.
		 */
		void render();


		/**
		 *  \brief Render triangle and overrides z-component.
		 *  \param[in] z z value
		 *  \param[in] renderBackFace render back face?
		 */
		void render(const math::flt z,
					const bool renderBackFace);


		/**
		 *  \brief Is front face?
		 */
		inline bool isFront() const {
			return mFace == FRONT;
		}


		/**
		 *  \brief Is back face?
		 */
		inline bool isBack() const {
			return mFace == BACK;
		}


	protected:
		/**
		 *  coordinates
		 */
		Vertex mVertices[SIZE];


		/**
		 *  type of the triangle
		 */
		Type mType;


		/**
		 *  face
		 */
		Face mFace;
	};


	/**
	 *  type of safety area
	 */
	typedef std::vector<Triangle> Area;


	/**
	 *  \brief Renders the obstacle.
	 *  \param[in] z vertical position of the obstacles
	 *  \param[in] color obstacle color
	 */
	void renderObstacle(const math::flt z,
						const Color & color);


	/**
	 *  \brief Renders the safety area.
	 *  \param[in] z vertical position of the obstacles
	 *  \param[in] boundaryColor color of the boundary
	 *  \param[in] borderColor color of the border
	 *  \param[in] renderBackFace render back face of the obstacle?
	 */
	void renderSafetyArea(const math::flt z,
						  const Color & boundaryColor,
						  const Color & borderColor,
						  const bool renderBackFace);


	/**
	 *  contour of the obstacle
	 */
	Contour mContour;


	/**
	 *  safety area
	 */
	Area mArea;


private:
	/**
	 *  \brief Calculates simple safety area.
	 *  \param[in] safetyDistance safety distance
	 */
	void simpleSafetyArea(const math::flt safetyDistance);


	/**
	 *  \brief Pushes a box into the area.
	 *  \param[in] box pointer to box
	 */
	void pushBox(math::Vec3 const * const box);


	/**
	 *  \brief Creates end piece.
	 *  \param[in] a begin
	 *  \param[in] b end
	 *  \param[in] safetyDistance safety distance
	 */
	void pushEnd(const math::Vec3 & a,
				 const math::Vec3 & b,
				 const math::flt safetyDistance);


	/**
	 *  \brief Creates a circle.
	 *  \param[in] center center of the circle
	 *  \param[in] safetyDistance safety distance
	 */
	void circle(const math::Vec3 & center,
				const math::flt safetyDistance);


	/**
	 *  \brief Creates a segment.
	 *  \param[in] center center
	 *  \param[in] dir direction
	 *  \param[in] ortho otrthogonal
	 *  \param[in] angle angle
	 *  \param[in] safetyDistance safetyDistance
	 *  \param[in] t type of the segment
	 *  \param[in] forceFront forces face type to front
	 */
	void segment(const math::Vec3 & center,
				 const math::Vec3 & dir,
				 const math::Vec3 & ortho,
				 const math::flt angle,
				 const math::flt safetyDistance,
				 const Triangle::Type t,
				 const bool forceFront = false);


	/**
	 *  \brief Pushes a triangle.
	 *  \param[in] first first vertex
	 *  \param[in] second second vertex
	 *  \param[in] third third vertex
	 *  \param[in] t type of the triangle
	 *  \param[in] normal (normal of the contour at this point)
	 *  \param[in] forceFront forces face type to front
	 */
	void push_triangle(const math::Vec3 & first,
					   const math::Vec3 & second,
					   const math::Vec3 & third,
					   const Triangle::Type t,
					   const math::Vec3 & normal,
					   const bool forceFront = false);


	/**
	 *  \brief Tests a point agains the reference point.
	 *  \param[in] point point to test
	 *  \param[in] normal normal of the point
	 *  \param[in] ref reference point
	 */
	bool testIsFrontFace(const math::Vec3 & point,
						 const math::Vec3 & normal,
						 const math::Vec2 & ref) const;
};

}
}
}
}
