#pragma once
#include "BaseObstacle.h"
#include <math/AutoMath.h>
#include <util/SkdTree.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/minmax_element.hpp>

namespace aa
{
namespace data
{
namespace obstacle
{

template < typename ObstacleType = aa::data::obstacle::BaseObstacle, unsigned int Dim = 2 >
struct ObstaclePtr {
	typedef ObstacleType obstacle_type;
	typedef typename obstacle_type::flt flt;
	typedef MATH_TYPES_VEC(flt, Dim) point_type;
	enum {
		dim = Dim
	};

	ObstaclePtr()
		: lowerBound(std::numeric_limits<flt>::max(), std::numeric_limits<flt>::max())
		, upperBound(-std::numeric_limits<flt>::max(), -std::numeric_limits<flt>::max())
		, ptr(0)
		, nr(std::numeric_limits< unsigned int>::max())
	{}

	ObstaclePtr(obstacle_type const * p, unsigned int _nr)
		: lowerBound(p->boundingCircleCentre() - point_type(p->boundingCircleRadius(), p->boundingCircleRadius()))
		, upperBound(p->boundingCircleCentre() + point_type(p->boundingCircleRadius(), p->boundingCircleRadius()))
		, ptr(p)
		, nr(_nr)
	{}

	flt distance(point_type const & p) const {
		return std::max(flt(0), std::sqrt(::math::ssd(ptr->boundingCircleCentre(), p)) - ptr->boundingCircleRadius());
	}
	::math::Vec2 lowerBound, upperBound;
	obstacle_type const * ptr;
	unsigned int nr;
};

template < typename ObstacleType = aa::data::obstacle::BaseObstacle, unsigned int Dim = 2 >
struct ObstacleContourPtr {
	typedef ObstacleType obstacle_type;
	typedef typename obstacle_type::flt flt;
	typedef ::aa::data::obstacle::util::Contour contour_type;
	typedef contour_type::value_type contour_point_type;
	typedef MATH_TYPES_VEC(flt, Dim) point_type;

	enum {
		dim = point_type::SizeAtCompileTime
	};

	struct comparator {
		comparator(unsigned int dim)
			: dimension(dim)
		{}
		bool operator()(contour_point_type const & a, contour_point_type const & b) const {
			return a[dimension] < b[dimension];
		}
		unsigned int const dimension;
	};

	ObstacleContourPtr()
		: lowerBound(std::numeric_limits<flt>::max(), std::numeric_limits<flt>::max())
		, upperBound(-std::numeric_limits<flt>::max(), -std::numeric_limits<flt>::max())
		, ptr(0)
		, nr(std::numeric_limits< unsigned int>::max())
	{}

	ObstacleContourPtr(obstacle_type const * p, unsigned int _nr)
		: ptr(p)
		, nr(_nr) {
		for (int i = 0; i < dim; ++i) {
			std::pair<contour_type::const_iterator const, contour_type::const_iterator const> minMax = boost::minmax_element(p->contour().begin(), p->contour().end(), comparator(i));
			assert(p->contour().end() != minMax.first);
			assert(minMax.first->size() > i);
			//std::cout << (*minMax.first)[i];
			lowerBound[i] = (*minMax.first)[i];
			upperBound[i] = (*minMax.second)[i];
		}
	}

	flt distance(point_type const & p) const {
		flt dist = 0;
		point_type diff0 = lowerBound - p;
		point_type diff1 = p - upperBound;

		for (int i = 0; i < dim; ++i) {
			dist += math::sqr(std::max(std::max(flt(0), diff0[i]), diff1[i]));
		}

		return sqrt(dist);
	}

	point_type lowerBound, upperBound;
	obstacle_type const * ptr;
	unsigned int nr;
};


template < typename ObstacleType = BaseObstacle, unsigned int Dim = 2, typename ObstaclePtrType = ObstaclePtr<ObstacleType, Dim> >
class ObstacleSkdTree
	: public ::util::SKDTree<ObstaclePtrType>
{
public:
	typedef ObstaclePtrType obstacle_ptr_type;
	typedef ::util::SKDTree<obstacle_ptr_type> parent_type;
	typedef typename ObstacleType::flt flt;
	ObstacleSkdTree()
		: parent_type()
	{}

	template<typename ObstacleVector>
	void rebuildTree(ObstacleVector const & obstacles) {
		typedef typename ObstacleVector::value_type value_type;
		mObstaclePtrs.clear();
		mObstaclePtrs.reserve(obstacles.size());

		for (unsigned int i = 0; i < obstacles.size(); ++i) {
			mObstaclePtrs.push_back(obstacle_ptr_type(&obstacles.at(i), i));
		}

		parent_type::rebuildTree(&mObstaclePtrs);
	}

protected:
	typename parent_type::box_vector mObstaclePtrs;
};

}
}
}
