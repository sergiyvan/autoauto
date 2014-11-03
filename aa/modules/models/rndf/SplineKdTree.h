#pragma once
#include <util/SkdTree.h>
#include <math/LaneSpline.h>
#include <math/PathSpline.h>
#include <boost/noncopyable.hpp>

class SplineKdTreeAccess;

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

struct BoundingBox {
	typedef ::math::flt flt;
	typedef ::math::Vec2 point_type;
	enum {
		dim = 2
	};

	point_type lowerBound, upperBound;
	math::LaneSpline const * spline;
	flt from;
	flt to;
};

class SplineKdTree
    : public util::SKDTree<BoundingBox>
{
	friend class SplineKdTreeAccess;
public:
    typedef BoundingBox::flt flt;
    typedef BoundingBox::point_type point_type;
	SplineKdTree()
	{}

	~SplineKdTree()
	{}

	typedef std::vector<std::pair<math::LaneSpline const *, flt> > spline_hit_vector;

	class SplineVisitor
		: boost::noncopyable	// Only checks the findClosest() logic, technically we could copy it
	{
	public:
		SplineVisitor(spline_hit_vector & _hits, point_type const & _p, flt _maxDistance, flt _accuracy)
			: hits(_hits)
			, p(_p)
			, maxDistance(_maxDistance)
			, maxDistance2(_maxDistance * _maxDistance)
			, accuracy(_accuracy)
		{}

		void operator()(const_box_iter leaf) {
			math::LaneSpline const * spline = leaf->spline;
			flt dist2, param;
			boost::tie(dist2, param) = math::findClosestPoint(*spline, leaf->from, leaf->to, 0.5f * (leaf->from + leaf->to), p, accuracy);

			if (dist2 == maxDistance2) {
				hits.push_back(std::make_pair(spline, param));
			}
			else if (dist2 < maxDistance2) {
				maxDistance2 = dist2;
				maxDistance = sqrt(dist2);
				hits.clear();
				hits.push_back(std::make_pair(spline, param));
			}
		}

		spline_hit_vector & hits;
		point_type p;
		flt maxDistance;
		flt maxDistance2;
		flt accuracy;
	};


	template<typename Qualifier>
	class QualifiedSplineVisitor
		: public SplineVisitor
	{
	public:
		QualifiedSplineVisitor(SplineKdTree::spline_hit_vector & _hits, point_type const & _p, flt _maxDistance, flt _accuracy, Qualifier const & _q)
			: SplineVisitor(_hits, _p, _maxDistance, _accuracy)
			, q(_q)
		{}

		void operator()(const_box_iter leaf) {
			math::LaneSpline const * spline = leaf->spline;
			flt dist, param;
			boost::tie(dist, param) = math::findClosestPoint(*spline, leaf->from, leaf->to, 0.5f * (leaf->from + leaf->to), p, accuracy);

			if (dist > maxDistance2) {
				return;
			}

			if (!q(spline, param, dist)) {
				return;
			}

			if (dist == maxDistance2) {
				hits.push_back(std::make_pair(spline, param));
			}
			else if (dist < maxDistance2) {
				maxDistance2 = dist;
				maxDistance = sqrt(dist);
				hits.clear();
				hits.push_back(std::make_pair(spline, param));
			}
		}

		Qualifier q;
	};

	flt findClosest(spline_hit_vector & hits, point_type const & p,
					flt maxDistance = std::numeric_limits<flt>::max(),
					flt accuracy = 0.01f
				   ) const {
		if (mNodes.empty()) {
			return std::numeric_limits<flt>::max();
		}

		SplineVisitor visitor(hits, p, maxDistance, accuracy);
        return util::SKDTree<BoundingBox>::findClosest<0, SplineVisitor>(0, visitor);
	}

	template<typename Qualifier>
	flt findClosestQualified(spline_hit_vector & hits, point_type const & p, Qualifier const & q,
							 flt maxDistance = std::numeric_limits<flt>::max(),
							 flt accuracy = 0.01f
							) const {
		typedef QualifiedSplineVisitor<Qualifier> Visitor;

		if (mNodes.empty()) {
			return std::numeric_limits<flt>::max();
		}

		Visitor visitor(hits, p, maxDistance, accuracy, q);
        return util::SKDTree<BoundingBox>::findClosest<0, Visitor>(0, visitor);
	}

	box_vector const & leaves() const {
        return util::SKDTree<BoundingBox>::leaves();
	}
};

}


}


}


}


