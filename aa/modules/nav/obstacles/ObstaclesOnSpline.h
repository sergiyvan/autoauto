#pragma once
/**
 * \file ObstaclesOnSpline.h
 * \author Georg Bremer, Miao Wang
 */

#include "InterferingObstacle.h"

#include <math/Quad2d.h>
#include <math/AABB.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <aa/data/obstacle/ObstacleSkdTree.h>
#include <aa/data/obstacle/BaseObstacleBundle.h>
#include <cmath>

#ifdef DISPLAY_OBSTACLE_MATH
#include "SampleRects.h"
#endif

namespace aa
{
namespace modules
{
namespace nav
{

namespace obstacles
{

/**
 * \class SkdTreeWrapper<class ObstacleBundle>
 * \brief This class is used so we can use findObstaclesOnSpline with every container of obstacles, wether it has a builtin ObstacleSkdTree or not
 */
template<class Obstacles>
struct SkdTreeWrapper {
	typedef aa::data::obstacle::ObstacleSkdTree<typename Obstacles::value_type, 2, aa::data::obstacle::ObstacleContourPtr<typename Obstacles::value_type, 2> > SkdTree;
	SkdTree tree;

	explicit SkdTreeWrapper(Obstacles const & b) {
		tree.rebuildTree(b);
	}

#ifdef DISPLAY_OBSTACLE_MATH
	static void display(SkdTree const &) {
	}
#endif
};

/**
 * \class SkdTreeWrapper<>
 * \brief Specialization so the builtin ObstacleSkdTree is used for BaseObstacleBundle
 */
template<>
struct SkdTreeWrapper<BaseObstacleBundle> {
	typedef BaseObstacleBundle::SkdTree SkdTree;
	SkdTree const & tree;

	explicit SkdTreeWrapper(BaseObstacleBundle const & b)
		: tree(b.skdTree()) {
		if (tree.empty() && !b.empty()) {
			const_cast<BaseObstacleBundle &>(b).buildTree();
		}
	}

#ifdef DISPLAY_OBSTACLE_MATH
	static void display(SkdTree const & t) {
		theSampleRects::instance().setTree(t);
	}
#endif
};


template<class _Spline, class _ObstacleBundle>
class ObstaclesOnSpline
{
public:
	typedef _Spline Spline;
	typedef _ObstacleBundle ObstacleBundle;
	typedef typename Spline::Img Image;
	typedef typename Spline::Dom Domain;
	typedef typename ObstacleBundle::value_type Obstacle;
	typedef typename Obstacle::Contour Contour;

	typedef InterferingObstacle_<Spline, ObstacleBundle> InterferingObstacle;
	typedef std::list<InterferingObstacle> InterferingObstacleList;

	typedef math::AABB<Domain, 2> aabb;//axis aligned bounding box


	/**
	 * \brief computes first intersecting obstacles in the region around a spline
	 * \param spline				given spline
	 * \param width					The width around the spline which forms the region of interest
	 * \param startParam			The parameter of the spline where the region starts
	 * \param endParam				The parameter of the spline where the region ends, can be smaller then startParam
	 * \param sampleStep			The spline is sampled in this stepping with overlapping. Smaller steps reduce errors at the expense of higher runtimes, must be positive
	 * \param startStopLookahead	The length of the region before and after startParam and endParam, default = sampleStep
	 * \param maxLateralError		If the lateral error between two spline samples is smaller than this, these will be joined to one
	 * \return InterferingObstacleList with InterferingObstacle = (nearest contourpoint, corresponding parameter of the spline, * to the intersecting Obstacle)
	 */
	static InterferingObstacleList findFirstObstacleOnSpline(Spline const & spline, ObstacleBundle const & obstacles, Domain left, Domain right, Domain startParam, Domain endParam, Domain sampleStep = 0.5, Domain startStopLookahead = 0, Domain const maxLateralError = 0.01) {
		return findObstaclesOnSpline(spline, obstacles, left, right, startParam, endParam, sampleStep, startStopLookahead, maxLateralError, true);
	}

	/**
	 * \brief computes all intersecting obstacles in the region around a spline
	 * \param spline				given spline
	 * \param width					The width around the spline which forms the region of interest
	 * \param startParam			The parameter of the spline where the region starts
	 * \param endParam				The parameter of the spline where the region ends, can be smaller then startParam
	 * \param sampleStep			The spline is sampled in this stepping with overlapping. Smaller steps reduce errors at the expense of higher runtimes, must be positive
	 * \param startStopLookahead	The length of the region before and after startParam and endParam, default = sampleStep
	 * \param maxLateralError		If the lateral error between two spline samples is smaller than this, these will be joined to one
	 * \return InterferingObstacleList with InterferingObstacle = (nearest contourpoint, corresponding parameter of the spline, * to the intersecting Obstacle)
	 */
	static InterferingObstacleList findAllObstaclesOnSpline(Spline const & spline, ObstacleBundle const & obstacles, Domain left, Domain right, Domain startParam, Domain endParam, Domain sampleStep = 0.5, Domain startStopLookahead = 0, Domain const maxLateralError = 0.01) {
		return findObstaclesOnSpline(spline, obstacles, left, right, startParam, endParam, sampleStep, startStopLookahead, maxLateralError, false);
	}

	static bool isAnyObstacleInBox(math::Quad2d const & box, ObstacleBundle const & obstacleBundle) {
		typedef SkdTreeWrapper<ObstacleBundle> wrap;
		wrap w(obstacleBundle);

		typedef typename wrap::SkdTree::hit_vector hit_vector;
		hit_vector hits;

		aabb q = axisAlignedBoundingBox(box);
		w.tree.findRegion(hits, q);
		BOOST_FOREACH(typename hit_vector::value_type & hit, hits) {
			if (isObstacleInBox(*hit->ptr, box)) {
				return true;
			}
		}
		return false;
	}

	//18.9 2013, added by shuiying. for static obstacle map rendering in stateLatticePlanner .
	static int isAnyObstacleInBox(math::Quad2d const & box, SkdTreeWrapper<ObstacleBundle> w) {

		typedef typename SkdTreeWrapper<ObstacleBundle>::SkdTree::hit_vector hit_vector;
		hit_vector hits;

		aabb q = axisAlignedBoundingBox(box);
		w.tree.findRegion(hits, q);
		BOOST_FOREACH(typename hit_vector::value_type & hit, hits) {
			if (isObstacleInBox(*hit->ptr, box)) {
				return (*hit->ptr).id();
			}
		}
		return -1;
	}

	static InterferingObstacleList findAllObstaclesInBox(math::Quad2d const & box, ObstacleBundle const & obstacleBundle) {
		typedef SkdTreeWrapper<ObstacleBundle> wrap;
		wrap w(obstacleBundle);

		InterferingObstacleList result;

		typedef typename wrap::SkdTree::hit_vector hit_vector;
		hit_vector hits;

		aabb q = axisAlignedBoundingBox(box);
		w.tree.findRegion(hits, q);
		BOOST_FOREACH(typename hit_vector::value_type & hit, hits) {
			if (isObstacleInBox(*hit->ptr, box)) {
//				result.push_back(hit);


				result.push_back(InterferingObstacle(const_cast< aa::data::obstacle::BaseObstacle *>(hit->ptr), Image(0, 0)));
			}
		}
		return result;
	}

private:
	///TODO use boost::minmax and move to math or similar
	static aabb axisAlignedBoundingBox(math::Quad2d const & box) {
		aabb q;
		q.lowerBound(0) = std::min(std::min(std::min(box.corner(0)(0), box.corner(1)(0)), box.corner(2)(0)), box.corner(3)(0));
		q.lowerBound(1) = std::min(std::min(std::min(box.corner(0)(1), box.corner(1)(1)), box.corner(2)(1)), box.corner(3)(1));
		q.upperBound(0) = std::max(std::max(std::max(box.corner(0)(0), box.corner(1)(0)), box.corner(2)(0)), box.corner(3)(0));
		q.upperBound(1) = std::max(std::max(std::max(box.corner(0)(1), box.corner(1)(1)), box.corner(2)(1)), box.corner(3)(1));
		return q;
	}

	static void moveAabb(aabb & q, Image const & t) {
		q.lowerBound += t;
		q.upperBound += t;
	}

	struct SplineSample {
		Image position;
		Image direction;
		Domain parameter;
		Domain lookahead;

		SplineSample(Image const & pos, Image const & dir, Domain param, Domain look)
			: position(pos)
			, direction(dir)
			, parameter(param)
			, lookahead(look) {
		}

	};

	struct NearerObstacle {

		int reverse;

		bool operator()(InterferingObstacle const & first, InterferingObstacle const & second) {
			return reverse * first.obstParam < reverse * second.obstParam;
		}

	};

	static InterferingObstacleList findObstaclesOnSpline(Spline const & spline, ObstacleBundle const & obstacleBundle, Domain const left, Domain const right, Domain startParam, Domain endParam, Domain sampleStep = 0.5, Domain startStopLookahead = 0, Domain const maxLateralError = 0.01, bool onlyFirst = false) {
		RTT::Logger::In in("ObstaclesOnSpline");


		std::set<uint> marked;
		InterferingObstacleList result;
		InterferingObstacleList resultChunk;
#ifdef DISPLAY_OBSTACLE_MATH
		SampleRects::sample_type displaySamples;
#endif

		std::pair<Domain, Domain> dom = spline.domain();

		if (!(dom.first <= startParam && startParam <= dom.second)) {
			std::cout << "first: " << dom.first << " start: " << startParam << " second: " << dom.second << std::endl;
		}

		assert(dom.first <= startParam && startParam <= dom.second);

		if (!(dom.first <= endParam && endParam <= dom.second)) {
			std::cout << "first: " << dom.first << " end: " << endParam << " second: " << dom.second << std::endl;
		}

		assert(dom.first <= endParam && endParam <= dom.second);


		//if sampling spline in reverse order, reverse will be -1, otherwise 1
		int reverse = copysign(1, endParam - startParam);
		sampleStep *= reverse;

		NearerObstacle comparator;
		comparator.reverse = reverse;

		//Sample the spline

		std::list<SplineSample> splineSamples;
		std::pair<Image, Image> posDir;
		spline.valueAndFirstDerivative(posDir, startParam);
		posDir.second.normalize();
		splineSamples.push_back(SplineSample(posDir.first, posDir.second, startParam, sampleStep));

		for (Domain xx = startParam + sampleStep; xx < endParam; xx += sampleStep) {
			spline.valueAndFirstDerivative(posDir, xx);
			SplineSample & oldPosDir = splineSamples.back();

			if ((posDir.second - oldPosDir.direction).norm() < maxLateralError) {//join both samples
				oldPosDir.lookahead += sampleStep;
			}
			else {
				splineSamples.push_back(SplineSample(posDir.first, posDir.second.normalized(), xx, sampleStep));
			}
		}

		//to compute the distance to the spline, we compute the distance to a point far, far away
		//this distance ensures that the error is always smaller than maxLateralError
		Domain const farDistance = (::math::sqr(startParam - endParam) - ::math::sqr(maxLateralError)) / maxLateralError;

		spline.valueAndFirstDerivative(posDir, endParam);
		posDir.second.normalize();
		splineSamples.push_back(SplineSample(posDir.first, posDir.second, endParam, startStopLookahead));


//      Image startPoint(splineSamples.front().position[0], splineSamples.front().position[1]);
//		Image backward(splineSamples.front().direction[0], splineSamples.front().direction[1]);
//		Image rear(backward[1], -backward[0]);
        Image startPoint( splineSamples.front().position );
        Image backward( splineSamples.front().direction );
        Image rear = ::math::smartOrthogonalVector(backward);
        backward *= -1 * startStopLookahead;
		backward += startPoint;

		typedef SkdTreeWrapper<ObstacleBundle> wrap;
		wrap w(obstacleBundle);

#ifdef DISPLAY_OBSTACLE_MATH
		wrap::display(w.tree);
#endif

		typedef typename wrap::SkdTree::hit_vector hit_vector;
		hit_vector hits;
		math::Quad2d box;

		for (typename std::list<SplineSample>::const_iterator posDir = splineSamples.begin();
				posDir != splineSamples.end();
				++posDir) {

            Image const position = posDir->position;
            Image const normDir(posDir->direction);

			Image const forward = position + normDir * (posDir->lookahead);
            Image const front = ::math::smartOrthogonalVector(normDir);

            Image const p1 = forward - front * left;
            Image const p2 = backward - rear * left;
            Image const p3 = backward + rear * right;
            Image const p4 = forward + front * right;

			//don't change the order of these corners!
            box.set( p1, p2, p3, p4);
//            box.set(forward - front * left,
//                    backward - rear * left,
//                    backward + rear * right,
//                    forward + front * right);

			hits.clear();
			aabb q = axisAlignedBoundingBox(box);

			w.tree.findRegion(hits, q);
#ifdef DISPLAY_OBSTACLE_MATH
			displaySamples.push_back(SampleRects::sample(box, posDir->position, hits.size(), q));
#endif

			Image const farLeft = forward - front * farDistance;
			Image const farRear = position - normDir * farDistance;

			BOOST_FOREACH(typename hit_vector::value_type & hit, hits) {
				Obstacle * it = const_cast<Obstacle *>(hit->ptr);

				if (marked.count(it->id())) {
					continue;
				}

				//TODO check bounding box first

				Domain minDistance = std::numeric_limits<Domain>::infinity();
				Domain minLeftDistance = std::numeric_limits<Domain>::infinity();
				Domain maxLeftDistance = -std::numeric_limits<Domain>::infinity();
				Image minPoint;

				Contour const & contour = (it)->contour();

				BOOST_FOREACH(typename Contour::value_type const pt, contour) {

                    if (box.contains( ::math::head(pt) )) {
                        Domain const dist = ::math::smartSquaredNorm(pt, farRear);

						if (dist < minDistance)	{
                            minPoint = math::zeroExtend(pt);
							minDistance = dist;
						}

					}
				}

				if (minDistance < std::numeric_limits<Domain>::infinity()) {
					marked.insert(it->id());

					BOOST_FOREACH(typename Contour::value_type const pt, contour) {

                        Domain const dist = ::math::smartSquaredNorm(pt, farLeft);

						if (dist < minLeftDistance) {
							minLeftDistance = dist;
						}

						if (dist > maxLeftDistance) {
							maxLeftDistance = dist;
						}
					}

					minLeftDistance = sqrt(minLeftDistance) - farDistance;
					maxLeftDistance = sqrt(maxLeftDistance) - farDistance;

					Domain const distanceFromBox = sqrt(minDistance) - farDistance;

					if (!(distanceFromBox > -2 * maxLateralError && distanceFromBox < posDir->lookahead + 2 * maxLateralError)) {
						RTT::Logger::In("ObstaclesOnSpline");
						RTT::log(RTT::Error) <<  "imprecise distance: " << distanceFromBox << " should be greater 0 and less than " << posDir->lookahead << RTT::endlog();
					}

					Domain const parameter = posDir->parameter + reverse * distanceFromBox;
					resultChunk.push_back(InterferingObstacle(&*it, minPoint, parameter, minLeftDistance, maxLeftDistance));
				}
			}

			resultChunk.sort(comparator);

			if (onlyFirst) {
#ifdef DISPLAY_OBSTACLE_MATH
				theSampleRects::instance().addSamples(displaySamples);
#endif
				return resultChunk;
			}

			rear = front;
			backward = forward;

			result.splice(result.end(), resultChunk);
		}

#ifdef DISPLAY_OBSTACLE_MATH
		theSampleRects::instance().addSamples(displaySamples);
#endif

		std::cout << std::flush;
		return result;
	}

	static bool isObstacleInBox(Obstacle const & obstacle, math::Quad2d const & box) {
		Contour const & contour = obstacle.contour();
		return box.containsAny(&contour.front(), contour.size());
	}

};

}

}
	//namespace obstacles
}
	//namespace nav
}
	//namespace modules
