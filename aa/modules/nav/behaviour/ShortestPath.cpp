#include "ShortestPath.h"

#include <util/Templates.h>
#include <patterns/Singleton.h>
#include <rtt/Logger.hpp>
// #include <boost/graph/astar_search.hpp>
// #include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <iostream>
#include <cmath>

#if defined(VERBOSE)
//#define DLOG(X)	Logger::log() << Logger::Debug << X << Logger::endl
#define DLOG(X)	std::cout << X << std::endl
#else
#define DLOG(X) /**/
#endif
#include <iomanip>

namespace aa
{
namespace modules
{
namespace nav
{
namespace behaviour
{

using namespace boost;
using namespace std;
using namespace RTT;
using RTT::Logger;
using namespace ::math;
using namespace ::aa::modules::models::rndf;


ShortestPath::ShortestPath()
    : mARNDGraph(patterns::Singleton<RNDFGraph>::instance())
{}

ShortestPath::~ShortestPath()
{}

bool ShortestPath::calcAPSP()
{
	Logger::In in("ShortestPath");

	Logger::log(Logger::Debug) << "Starting johnsons algorithm for all pairs shortest path " << Logger::endl;
	aGraph const & rGraph = mARNDGraph.getBoostGraph();

	// count all vertices
	unsigned long V = num_vertices(rGraph);

	//init weight Matrix with infinity
	mDistMatrix.resize(V);

	for (unsigned long i = 0; i < V; ++i) {
		mDistMatrix[i].resize(V, std::numeric_limits<flt>::infinity());
	}

	// Johnsons apsp algo runs in O(VE logV) time
	bool ret = johnson_all_pairs_shortest_paths(
	               const_cast<aGraph &>(rGraph),
	               mDistMatrix
	           );

	if (ret) {
		Logger::log(Logger::Debug) << "Johnson Shortest Paths - finished" << Logger::endl;
	}
	else {
		Logger::log(Logger::Error) << "Johnson Shortest Paths - failed" << Logger::endl;
	}

	return ret;
}

flt ShortestPath::getLengthOfPath(vertex_descr const & from, vertex_descr const & to)
{
	Logger::In in("ShortestPath");

	if (from == boost::graph_traits<aGraph>::null_vertex() || to == boost::graph_traits<aGraph>::null_vertex()) {
		Logger::log(Logger::Error) << "no waypoint selected - no route to host" << Logger::endl;
		return numeric_limits<flt>::infinity();
	}

	property_map<aGraph, boost::vertex_index_t>::const_type indexMap = get(boost::vertex_index_t(), const_cast<aGraph const &>(mARNDGraph.getBoostGraph()));
	unsigned long const fromIndex	= indexMap[from];
	unsigned long const toIndex		= indexMap[to];

	flt const ret = mDistMatrix[fromIndex][toIndex];
// 	Logger::log(Logger::Info) << "Start:" << startIdx << " -> End:" << endIdx << " = " << mDistMatrix[startIdx][endIdx] << Logger::endl;

	return ret;
}


void ShortestPath::printMatrix()
{

	/*	for (std::vector< std::vector<flt> >::iterator oiter=mDistMatrix.begin(); oiter != mDistMatrix.end(); oiter++)
			for (std::vector<flt>::iterator iiter=mDistMatrix.begin(); iiter != mDistMatrix.end(); iiter++)
				log() << Logger::Info << "" << Logger::endl;*/

	for (size_t i = 0; i < mDistMatrix.size(); ++i) {
		log() << Logger::Info << i << " -> ";

		for (size_t j = 0; j < mDistMatrix.size(); ++j) {
			log() << Logger::Info << j << ":" << mDistMatrix[i][j] << ",";
		}

		log() << Logger::Info << Logger::endl;
	}
}


void ShortestPath::blockEdge(edge_descr const & edge)
{
	if (mLastBlockedEdgesDecay == TimeStamp()) {
		mLastBlockedEdgesDecay.stamp();
	}

	aGraph const & graph = mARNDGraph.getBoostGraph();

	property_map<aGraph, edge_data_t>::const_type
	edgeDataMap = get(edge_data_t(), graph);
	property_map<aGraph, edge_weight_t>::type
	edgeWeightMap = get(edge_weight_t(), const_cast<aGraph &>(graph));

    flt originalLength = std::abs(edgeDataMap[edge].targetParam - edgeDataMap[edge].sourceParam);
	edgeWeightMap[edge] = 10.0f * originalLength + 10000.0f;
	mBlockedEdges.insert(edge);
}

void ShortestPath::unblockEdge(edge_descr const & edge)
{
	aGraph const & graph = mARNDGraph.getBoostGraph();

	property_map<aGraph, edge_data_t>::const_type
	edgeDataMap = get(edge_data_t(), graph);
	property_map<aGraph, edge_weight_t>::type
	edgeWeightMap = get(edge_weight_t(), const_cast<aGraph &>(graph));

    flt originalLength = std::abs(edgeDataMap[edge].targetParam - edgeDataMap[edge].sourceParam);
	edgeWeightMap[edge] = originalLength;
	mBlockedEdges.erase(edge);
}

void ShortestPath::decayBlockedEdges()
{
	TimeStamp now;
	now.stamp();
	flt dt = 1e-9f * RTT::os::TimeService::ticks2nsecs(mLastBlockedEdgesDecay - now);

	aGraph const & graph = mARNDGraph.getBoostGraph();
	property_map<aGraph, edge_data_t>::const_type
	edgeDataMap = get(edge_data_t(), graph);
	property_map<aGraph, edge_weight_t>::type
	edgeWeightMap = get(edge_weight_t(), const_cast<aGraph &>(graph));

	for (edge_set_type::iterator it = mBlockedEdges.begin(); it != mBlockedEdges.end(); ++it) {
		flt & edgeWeight = edgeWeightMap[*it];
		EdgeData const & edgeData = edgeDataMap[*it];
        flt originalLength = std::abs(edgeData.targetParam - edgeData.sourceParam);
		flt decay = dt / (60.0f * 30.0f) * ((10.0f - 1.0f) * originalLength + 10000.0f)  ;

		edgeWeight -= decay;

		if (edgeWeight <= originalLength) {
			edgeWeight = originalLength;
			mBlockedEdges.erase(*it);
		}
	}

	mLastBlockedEdgesDecay = now;
}
}
}
}
}
