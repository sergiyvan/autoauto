#pragma once

#include "VertexData.h"
#include "EdgeData.h"
#include <math/Geodetic.h>

#include <boost/noncopyable.hpp>
#include <vector>

#define BOOST_NO_HASH	// Disables use of deprecated header
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <boost/any.hpp>
#include <rtt/PropertyBag.hpp>

#include <math/AkimaSpline.h>
#include <math/LaneSpline.h>
#include <util/BitMask.h>

#include "io/BoundaryType.h"

// serialize
#include <boost/serialization/shared_ptr.hpp>

// typedef ::math::AkimaSpline< ::math::Vec2, ::math::flt> LaneSpline;

class ZoneInfo;

namespace aa
{
namespace modules
{
namespace models
{
namespace rndf
{
class SplineKdTree;
namespace io {
class RNDFData;
class Waypoint;
class Lane;
}


class GraphData
{
public:
	GraphData()
	{}

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
    }
};

// Boost Property Map - Magic

struct graph_data_t {
	typedef boost::graph_property_tag kind;
};

struct vertex_data_t {
	typedef boost::vertex_property_tag kind;
};

struct edge_data_t {
	typedef boost::edge_property_tag kind;
};

struct EmptyNullNil {
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
	}
};

//////////////////////////////////////// typedefs ////////////////////////////////

typedef boost::vecS out_edge_list_type;
typedef boost::listS vertex_list_type;
// typedef boost::vecS vertex_list_type;

typedef boost::adjacency_list_traits < out_edge_list_type,
		vertex_list_type,
		boost::bidirectionalS > aGraphTraits;

typedef boost::property < boost::vertex_index_t, uint,
		boost::property < boost::vertex_predecessor_t, aGraphTraits::vertex_descriptor,
		boost::property<vertex_data_t, VertexData, EmptyNullNil>
		>
		> vertex_properties_type;

typedef /* boost::property< boost::edge_index_t, uint, */
boost::property < boost::edge_weight_t, ::math::flt,
	  boost::property<edge_data_t, EdgeData, EmptyNullNil>
	  // >
	  > edge_properties_type;

typedef boost::adjacency_list < out_edge_list_type,
		vertex_list_type,
		boost::bidirectionalS,
		vertex_properties_type,
		edge_properties_type,
		GraphData
		> aGraph;

typedef boost::graph_traits<aGraph> GraphTraits;

typedef GraphTraits::vertex_descriptor vertex_descr;
typedef GraphTraits::vertex_iterator vertex_iter;
typedef GraphTraits::adjacency_iterator adjacency_iter;
typedef GraphTraits::edge_descriptor edge_descr;
typedef GraphTraits::edge_iterator edge_iter;
typedef GraphTraits::in_edge_iterator in_edge_iter;
typedef GraphTraits::out_edge_iterator out_edge_iter;

/**
* A Vector of edges as representation for a lane
*/
typedef std::vector<edge_descr> lane_vec_type;

/**
* A Vector of lanes as representation for a segment
*/
typedef std::vector<lane_vec_type> segment_vec_type;

/**
* A Vector of perimeter points and a vector of spot lanes as representation for a zone
*/
typedef std::pair<std::vector<vertex_descr>, std::vector<edge_descr> > zone_vec_type;

/**
* A Vector of segments and a vector of zones as representation for the complete graph
*/
typedef std::pair<std::vector<segment_vec_type>, std::vector<zone_vec_type> > graph_vec_type;

//////////////////////////////////////// struct EdgeOrder ////////////////////////////////

struct EdgeOrder {
	bool operator()(edge_descr const & a, edge_descr const & b) const {
		return a.get_property() < b.get_property();
	}
};

}
}
}
}

//////////////////////////////////////// template classes ////////////////////////////////
//@{
extern template class boost::adjacency_list < aa::modules::models::rndf::out_edge_list_type,
	   aa::modules::models::rndf::vertex_list_type,
	   boost::bidirectionalS,
	   aa::modules::models::rndf::vertex_properties_type,
	   aa::modules::models::rndf::edge_properties_type,
	   aa::modules::models::rndf::GraphData
	   >;

extern template class boost::adjacency_list < boost::vecS,
	   boost::vecS,
	   boost::bidirectionalS,
	   boost::property < boost::vertex_predecessor_t, aa::modules::models::rndf::GraphTraits::vertex_descriptor,
	   boost::property<aa::modules::models::rndf::vertex_data_t, aa::modules::models::rndf::VertexData>
	   > ,
	   boost::property < boost::edge_index_t, uint,
	   boost::property < boost::edge_weight_t, ::math::flt,
	   boost::property<aa::modules::models::rndf::edge_data_t, aa::modules::models::rndf::EdgeData>
	   >
	   > ,
	   aa::modules::models::rndf::GraphData
	   >;
//@}

namespace aa
{
namespace modules
{
namespace models
{
namespace rndf
{
//////////////////////////////////////// Graph ////////////////////////////////

class RNDFGraph
	: boost::noncopyable
{
public:
	/**
	* Constructor. An empty graph will be created.
	* For loading data, buildGraph(boost::shared_ptr<RNDFData> const & pRNDFData) should be called.
	*/
	RNDFGraph();

	/**
	* Destructor
	*/
	~RNDFGraph();

	/**
	* Initial build of the graph. Does not optimize
	*/
	void buildGraph(boost::shared_ptr<aa::modules::models::rndf::io::RNDFData> const & pRNDFData, bool optimised = false);

	/**
	* Removes all vertices and edges
	*/
	void resetGraph();

	/**
	* Optimizes the graph, which means:
	* - prune vertices
	* - build mirror graph
	* - add lane changing edges
	*/
	void optimiseGraph();

	//////////////////////////////////////// editing ////////////////////////////////

	/**
	* Adds a new vertex to the graph.
	* The vertex will be registered in the vertex index map, and the vertex data map.
	* Please note, that the graph will be inconsistent after this operation.
	* The position will be set. Other values (eg. segment ID) will not be set.
	* These values must be set by the caller.
	* \param pos a ::math::Vec2 representing the position of the point
	* \return the new created vertex
	*/
	vertex_descr addVertex(::math::Vec2 const & pos);

	vertex_descr addWaypoint(uint l0, uint l1, uint id, ::math::Vec2 const & pos);


	/**
	* Add a new lane edge to the graph, which leads from source v0 to target v1
	* Param lane is parsed for the following data:
	* - the index of the segment: must not be 0. A new segment is created, if necessary.
	* - the index of the lane: must not be 0. A new lane is created, if necessary.
	* - the lane width (in feet)
	* - whether the edge is a zone edge (TODO: this may be a precondition as well)
	*
	* Preconditions:
	* - source != target
	* \return the newly created edge, or an existing one from v0 to v1
	*/
	edge_descr addLaneEdge(vertex_descr v0, vertex_descr v1, aa::modules::models::rndf::io::Lane const & lane);

	/**
	* Add a new connecting edge to the graph, which leads from source v0 to target v1.
	* If the edge already exists, it will be returned to the caller and no new edge will be created.
	*
	* Preconditions:
	* - source != target
	* \return the newly created edge, or an existing one from source to target
	*/
	edge_descr addConnectingEdge(vertex_descr source, vertex_descr target);

	/**
	* Creates a new segment.
	* Please note, that the graph will be inconsistent after this call.
	* There will be an empty segment!
	* \return the ID of the new segment
	*/
	uint createNewSegment();

	/**
	* Creates a new lane in the given segment.
	* Please note, that the graph will be inconstistent after this call.
	* There will be an empty lane!
	* \param segmentID the ID of the segment, in which we want to create a new lane
	* \return the ID of the new lane
	*/
	uint createNewLane(uint segmentID);

	/**
	* removes a connection (either zone or lane connection) from the graph
	*/
	void removeConnectingEdge(edge_descr const & edge);

	/**
	* removes an edge from the graph.
	* WARNING! The graph will be inconsistent after this call. Caller needs to clean up!
	*/
	void removeEdge(edge_descr const & edge);

	/**
	* removes a lane vertex from the graph.
	* If the vertex is the starting (end) point of a lane, then the first (last) edge of the graph will be removed as well
	* Otherwise, if the vertex is inside a lane, then the incoming and outgoing lane edges will be removed and a new lane edge
	* from the predecessor to the sucessor will be created.
	* All connecting edges from and to this vertex will be removed.
	*
	* Preconditions to be checked:
	* 1. this vertex is a lane vertex (not a perimeter point or a spot point)
	* 2. this vertex is not a checkpoint
	* 3. the lane contains at least 2 edges
	*/
	bool removeLaneVertex(vertex_descr v);

	/**
	* Should be called, if the graph was changed. Will rebuild splines.
	*/
	void recalculateData();

	/**
	* Updates the name of the vertex
	* This method should be called, after changing any related field in the vertexData
	*/
	void updateVertexName(vertex_descr v);

	/**
	* Inserts a point in the middle of this (splinified) edge.
	*/
	vertex_descr insertPoint(edge_descr const & edge, VertexData::vertex_type vertexType);

	/**
	* Sets the checkpoint flag and id for the given vertex and inserts it into the checkpoints vector
	*/
	void addCheckpoint(vertex_descr v);

	/**
	* Unsets the checkpoint flag and id for the given vertex, removes it from the checkpoints vector
	* and updates the ids of the other checkpoints if necessary (to keep them consecutively numbered)
	*/
	void removeCheckpoint(vertex_descr v);

	//////////////////////////////////////// observing methods, getters (const) ////////////////////////////////

	aGraph const & getBoostGraph() const;
	ZoneInfo const & zoneInfo() const;

	/**
	* Counts the vertices in the lane
	*/
	/// computes a heap of edges that is nearest to the given point, the second value is the param, returns squared distance
	::math::flt getNearestEdges(std::vector<std::pair<edge_descr, ::math::flt> > & , ::math::Vec2 const & punkt, ::math::flt maxDist, ::math::flt accuracy = 0.01f) const;

	std::pair< ::math::flt, ::math::flt> getDirectNeighbouredEdge(::math::Vec2 const & pos, edge_descr const & edge, edge_descr & left, edge_descr & right, ::math::Vec2 & avoidPosLeft, ::math::Vec2 & avoidPosRight) const;

	bool getDirectNeighbouredEdge(::math::flt param, edge_descr const & edge, bool left,
								  edge_descr & neighbourEdge, ::math::Vec2 & neighbourPos,
								  ::math::flt & closestParm, ::math::flt & closestSquaredDist) const;

	/// pairs< *,* > stands for the left- and righthand direct neighbour edges, respectivly
	std::pair< ::math::flt, ::math::flt> getDirectNeighbouredEdge(::math::flt param, edge_descr const & edge, edge_descr & left,
			edge_descr & right, ::math::Vec2 & avoidPosLeft, ::math::Vec2 & avoidPosRight) const;

	/// \return the vertex having the name \arg name, or boost::graph_traits<G>::null_vertex(), if there is no such vertex
	vertex_descr getVertex(std::string const & name) const;
	/// \return the vertex with the id l0.l1.l2, or boost::graph_traits<G>::null_vertex(), if there is no such vertex
	vertex_descr getVertex(uint l0, uint l1, uint l2) const;
	/// saves the RNDFGraph to a file fname in RNDF or ARND
	void save(std::string const & fname);
	/// loads the RNDFGraph from a file fname in ARND Format
	void load(std::string const & fname);
	/// \return the mirrored vertex to the given one or or boost::graph_traits<G>::null_vertex(), if there is no such vertex
	vertex_descr getMirroredVertex(vertex_descr v) const;
	edge_descr getMirroredEdge(edge_descr const & ed) const;
	/// \return the vertexData associated with the given vertex_descr

	/**
	 * \return the width of the lane given the edge ed at the parameter param
	 */
	::math::flt laneWidth(edge_descr const & ed, ::math::flt param) const;
	unsigned int numSegments() const;

	/**
	 * Returns the closest edge to pos with the same direction as orientation, including mirrored ones
	 * \return tuple of - edge_descr, which correspondends to the closest edge, if there is no edge: edge_descr()
	 *                  - parameter, which called on the lane-spline yields the closest point
	 *                  - distance of the closest point
	 */
	boost::tuple<edge_descr, ::math::flt, ::math::flt> getClosestSameDirectedEdge(::math::Vec2 const & pos, ::math::Vec2 const & orientation) const;

	/**
	 * Returns the closest non-mirrored edge to pos with the same direction as orientation
	 * \return tuple of - edge_descr, which correspondends to the closest edge, if there is no edge: edge_descr()
	 *                  - parameter, which called on the lane-spline yields the closest point
	 *                  - distance of the closest point
	 */
	boost::tuple<edge_descr, ::math::flt, ::math::flt> closestEdgeSameDirNonMirrored(::math::Vec2 const & pos, ::math::Vec2 const & orientation) const;

	/**
	* Collects all vertices from start of this lane until the target vertex of param edge in reverse order
	*/
	void getLane(std::vector<vertex_descr> & verticesResultVec, std::vector<edge_descr> & edgesResultVec, edge_descr edge) const;

	/**
	* Collects all vertices of this lane in forward order
	*/
	void getWholeLane(std::vector<vertex_descr> & verticesResultVec, std::vector<edge_descr> & edgesResultVec, edge_descr edge) const;

	bool trackDir(::math::Vec2 & dir, vertex_descr v) const;
	bool inDir(::math::Vec2 & dir, vertex_descr v) const;
	bool outDir(::math::Vec2 & dir, vertex_descr v) const;

	graph_vec_type const & getGraphVector() const;

	/**
	* \param segmentID The segment ID of the lane to be moved
	* \param laneID The lane ID of the lane to be moved
	* \return an uint representing the new segment ID of the lane (lane ID will be 1)
	*/
	uint moveLaneToNewSegment(uint segmentID, uint laneID);

	/**
	* Merges 2 segments into one segment.
	* Precondition: segmentID1 < segmentID2
	*/
	void mergeSegments(uint segmentID1, uint segmentID2);

	/**
	* recalculates the indices and names of all edges and vertices in the given lane
	* vertices with ID==0 will be taken as inserted points. Their ID will remain unchanged.
	*/
	void updateLaneFromGraph(edge_descr const & edge);

	/**
	* indicates, whether this graph was optimised.
	*/
	bool isOptimised;

	/**
	* flag indicating, whether the enhanced RNDF file format (with traffic lights etc.) shall be used for export
	*/
	static const bool ENHANCED_RNDF = true;

	void resplinify();
	RTT::PropertyBag const & getProperties() const;

	SplineKdTree const & splineKDTree() const;

	/**
	* returns a vector of vertex_descr containing all checkpoints of this rndf graph
	*/
	std::vector<vertex_descr> const & checkpoints() const;

	void variousGraphFixes();

	//////////////////////////////////////// notification and locking ////////////////////////////////

	boost::signals2::signal<void (edge_descr const &)> signalEdgeChanged;
	boost::signals2::signal<void (RNDFGraph const &)> signalGraphChanged;

	void lock() const;
	void unlock() const;

	/**
	 * access to some functions of private member p (added by shuiying)
	 */
	std::pair<edge_descr, bool> findNonMirroredEdge(::math::flt param, boost::shared_ptr< ::math::LaneSpline const> laneSpline) const;
	std::pair<edge_descr, bool> findMirroredEdge(::math::flt param, boost::shared_ptr< ::math::LaneSpline const> laneSpline) const ;

private:
	struct Pimpl;
	std::auto_ptr<Pimpl> p;

	// serialization
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

}
}
}
}
