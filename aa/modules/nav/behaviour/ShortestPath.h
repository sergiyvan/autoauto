#pragma once

#include <aa/modules/models/rndf/RndfGraph.h>

#include <boost/noncopyable.hpp>
#include <core/TimeStamp.h>
namespace aa
{
namespace modules
{
namespace nav
{
namespace behaviour
{
class ShortestPath
	: boost::noncopyable
{
public:
	typedef ::math::flt flt;
	typedef std::vector< std::vector<flt> > WeightMatrix;

	ShortestPath();
	~ShortestPath();

	/**
	* @return   calcAPSP returns false if there is a negative weight cycle in the graph and true otherwise
	* @param DistM  The DistanceMatrix stores the length of the shortest path between each pair of vertices u,v in DistM[u][v]
	*/
	bool calcAPSP();

	void printMatrix();

	WeightMatrix const & getWeightMatrix() const
	{
		return mDistMatrix;
	}

    flt getLengthOfPath(::aa::modules::models::rndf::vertex_descr const & from, ::aa::modules::models::rndf::vertex_descr const & to);

    void blockEdge(::aa::modules::models::rndf::edge_descr const & edge);
    void unblockEdge(::aa::modules::models::rndf::edge_descr const & edge);
	void decayBlockedEdges();

private:
    typedef std::set< ::aa::modules::models::rndf::edge_descr, ::aa::modules::models::rndf::EdgeOrder> edge_set_type;

    ::aa::modules::models::rndf::RNDFGraph & mARNDGraph;
	WeightMatrix mDistMatrix;
	TimeStamp mLastBlockedEdgesDecay;
	edge_set_type mBlockedEdges;
};
}
}
}
}
