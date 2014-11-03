#pragma once

#include <set>
#include <iterator>
#include <aa/modules/models/rndf/RndfGraph.h>

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

/**
	Iterates through all lanes of a segment.

	Usage:

		LaneIterator::LaneIteratorPair p = LaneIterator::init(&mGraph,
															  &mEdgeDataMap,
															  segment);

		for(LaneIterator it = p.first; it != p.second; ++it) {
			const EdgeData &edgeData = mEdgeDataMap[*it];

		}
**/



class LaneIterator
	: public std::iterator<std::forward_iterator_tag, edge_descr>
{
public:
	LaneIterator & operator=(const LaneIterator & other) {
		mSegment = other.mSegment;
		mGraph = other.mGraph;
		mIterator = other.mIterator;
		mEnd = other.mEnd;
		mLaneIDs = other.mLaneIDs;
		mEdgeDataMap = other.mEdgeDataMap;
		mEndIterator = other.mEndIterator;

		return(*this);
	}


	bool operator==(const LaneIterator & other) {
		return mIterator == other.mIterator;
	}


	bool operator!=(const LaneIterator & other) {
		return mIterator != other.mIterator;
	}


	LaneIterator & operator++() {
		return next();
	}


	LaneIterator & operator++(int) {
		return next();
	}


	edge_descr operator*() {
		return *mIterator;
	}


	/**
		Parameters:
			graph		-	graph to analyse
			info		-	edge map
			segment		-	segment id
	**/
	typedef std::pair<LaneIterator, LaneIterator> LaneIteratorPair;
	static LaneIteratorPair init(
		aGraph const * graph,
		boost::property_map<aGraph, edge_data_t>::const_type * info,
		unsigned int segment) {
		LaneIterator begin(graph, info, segment),
					 end(begin.mEnd);

		begin.setEnd(&end);

		return LaneIteratorPair(begin, end);
	}

protected:
	/**
		Creates the end item.

		Parameters:
			end		-	internal iterator end
	**/
	LaneIterator(edge_iter & end)
		: mIterator(end) {
	}


	/**
		Parameters:
			graph		-	graph to analyse
			info		-	edge map
			segment		-	segment id
	**/
	LaneIterator(aGraph const * graph,
				 boost::property_map<aGraph, edge_data_t>::const_type * info,
				 unsigned int segment)
		: mGraph(graph)
		, mEdgeDataMap(info)
		, mSegment(segment) {
		tie(mIterator, mEnd) = edges(*graph);

		next();
	}


	void setEnd(LaneIterator * end) {
		mEndIterator = end;
	}

	LaneIterator & next() {
		for (; mIterator != mEnd; ++mIterator) {
			const EdgeData & edgeData = (*mEdgeDataMap)[*mIterator];

			if (edgeData.segment == mSegment) {
				//found one!
				std::set<unsigned int>::iterator it =
					mLaneIDs.find(edgeData.lane);

				if (it == mLaneIDs.end()) {
					//found new element!
					mLaneIDs.insert(edgeData.lane);
					return (*this);
				}
			}
		}

		return *mEndIterator;
	}

	//segment we look for
	unsigned int mSegment;

	//graph we work with
	aGraph const * mGraph;
	boost::property_map<aGraph, edge_data_t>::const_type * mEdgeDataMap;

	//status
	edge_iter mIterator,
			  mEnd;
	LaneIterator * mEndIterator;

	std::set<unsigned int> mLaneIDs;
};

}


}


}


}



