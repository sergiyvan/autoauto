#pragma once
#include <vector>
#include "RndfGraph.h"

struct XingWP {
	typedef ::math::Vec2 Vec2;
	::aa::modules::models::rndf::vertex_descr wp;
	Vec2 pos, dir;
	bool isStop, isExit, isEntry, isBoth;
};

class Xing
{
public:
	typedef ::math::flt flt;
	typedef XingWP::Vec2 Vec2;
	typedef	std::vector<XingWP> Lane;
	typedef std::vector<Lane> lane_vector_type;

	Xing()
	{}

	Xing(Xing const & other)
		: xingID(other.xingID)
		, xingLanes(other.xingLanes)
		, xingCenter(other.xingCenter)
		, mCrossingEdges(other.mCrossingEdges)
		, mIntersectingEdges(other.mIntersectingEdges)
	{}

	Xing & operator=(Xing const & other) {
		if (this == &other) {
			return *this;
		}

		xingID = other.xingID;
		xingLanes = other.xingLanes;
		xingCenter = other.xingCenter;
		mCrossingEdges = other.mCrossingEdges;
		mIntersectingEdges = other.mIntersectingEdges;

		return *this;
	}

	uint xingID;
	lane_vector_type xingLanes;
	Vec2 xingCenter;

	std::vector< ::aa::modules::models::rndf::edge_descr> mCrossingEdges;
	struct EdgeIntersection {
		EdgeIntersection(::aa::modules::models::rndf::edge_descr const & _intersectingEdge, flt _intersectingParam, flt _otherParam)
			: intersectingEdge(_intersectingEdge)
			, intersectingParam(_intersectingParam)
			, otherParam(_otherParam)
		{}

		::aa::modules::models::rndf::edge_descr intersectingEdge;
		flt intersectingParam;
		flt otherParam;
	};

	std::map< ::aa::modules::models::rndf::edge_descr, std::vector<EdgeIntersection>, ::aa::modules::models::rndf::EdgeOrder> mIntersectingEdges;
};
