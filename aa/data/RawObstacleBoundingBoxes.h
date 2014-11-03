#pragma once

#include <math/Types.h>
#include <boost/array.hpp>
#include <vector>
#include <util/aligned_allocator.h>
#include <util/PooledObjectTemplate.h>
#include <rtt/Port.hpp>

namespace aa
{

namespace data
{

class BoundingBox
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;

	BoundingBox() {}

	BoundingBox(Vec3 const & v0,
				Vec3 const & v1,
				Vec3 const & v2,
				Vec3 const & v3,
				Vec3 const & v4,
				Vec3 const & v5,
				Vec3 const & v6,
				Vec3 const & v7) {
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		vertices[3] = v3;
		vertices[4] = v4;
		vertices[5] = v5;
		vertices[6] = v6;
		vertices[7] = v7;
	}

	explicit BoundingBox(Vec3 box[2][4]) {
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 4; j++) {
				vertices[4*i + j] = box[i][j];
			}
	}

	//number of vertices for this box
	static const int NUM_VERTICES = 8;

	//vertices
	boost::array<Vec3, NUM_VERTICES> vertices;


	//vertex index description:
	/*
	    5---------------------6
	   /                     /|
	  / |       top         / |
	 /  |                  /  |
	4---------------------7   |
	|   |                 |   |
	|   |                 |   |
	|   |                 |   |
	|   |                 |   |
	|   1-----------------|---2
	|  /                  |  /
	| /       bottom      | /
	|/                    |/
	0---------------------3

	*/

	int bottomIndex() const {
		return 0;
	}

	int topIndex() const {
		return 4;
	}

	int width() const {
		return vertices[0][0] - vertices[3][0] + 1;
	}

	int depth() const {
		return vertices[1][1] - vertices[0][1] + 1;
	}

	int height() const {
		return vertices[4][2] - vertices[0][2] + 1;
	}
};


class RawObstacleBoundingBoxes
	: public std::vector<data::BoundingBox, util::aligned_allocator<data::BoundingBox, 16u> >
{
public:
	typedef std::vector<data::BoundingBox, util::aligned_allocator<data::BoundingBox, 16u> > parent_type;

	RawObstacleBoundingBoxes() { }
	RawObstacleBoundingBoxes(size_t size) : parent_type(size) { }
	~RawObstacleBoundingBoxes() { }

};
TIMEDPOOLEDOBJECT(RawObstacleBoundingBoxes, 8);
}
}


namespace RTT
{
extern template class InputPort<aa::data::TimedRawObstacleBoundingBoxes_ptr>;
extern template class OutputPort<aa::data::TimedRawObstacleBoundingBoxes_ptr>;
}
