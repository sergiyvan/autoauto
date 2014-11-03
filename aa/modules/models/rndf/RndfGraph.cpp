#include "RndfGraph.h"
#include "GraphAlgs.h"
#include "SplineKdTree.h"
#include "ZoneInfo.h"
#include "XingInfo.h"

#include <math/AutoMath.h>
#include <math/DiffGeom.h>
#include <math/PathSpline.h>
#include <patterns/Singleton.h>
#include <util/Templates.h>
#include <util/OrocosHelperFunctions.h>
#include <core/TimeStamp.h>
#include <util/serialization/Mat.h>
#include <util/serialization/Tuple.h>
#include <util/Ports.h>


#include <deque>
#include <tr1/unordered_map>
#include <boost/functional/hash.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include <rtt/Logger.hpp>

#include <qmutex.h>
#include <fstream>
#include <ctime>

#include <aa/modules/models/rndf/io/RndfReader.h>
#include <aa/modules/models/rndf/io/RndfWriter.h>

// serialization
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/graph/adj_list_serialize.hpp>


#ifdef __GXX_EXPERIMENTAL_CXX0X__
# include <unordered_map>
namespace tr1 = std;
#else
namespace tr1 = std::tr1;
#endif

//#define STATISTICS
#define SPLINES_FOR_UTURNS
#define CIRCULAR_CURVES
// #define CIRCULAR_CONNECTION_HEURISTIC
#define VERBOSE

#if defined(VERBOSE)
#define DLOG(X)	RTT::log(RTT::Debug) << X << RTT::endlog()
#else
#define DLOG(X) /**/
#endif

template class boost::adjacency_list<aa::modules::models::rndf::out_edge_list_type, aa::modules::models::rndf::vertex_list_type, boost::bidirectionalS, aa::modules::models::rndf::vertex_properties_type, aa::modules::models::rndf::edge_properties_type, aa::modules::models::rndf::GraphData>;

namespace aa
{
namespace modules
{
namespace models
{
namespace rndf
{

using namespace aa::modules::models::rndf::io;

typedef boost::shared_ptr<math::LaneSpline const> LaneSplinePtr;
typedef patterns::Singleton<XingInfo> xingInfo;



/////////////////////////////////////////////////////////////////////
// define the vector and map types specific to store our graph data
// Note: if you do operations on the graph, these data structure must be maintained

typedef std::map<math::LaneSpline const *, std::pair<std::vector<edge_descr>, std::vector<edge_descr> > > reverse_map_type;
typedef tr1::unordered_map<vertex_descr, vertex_descr> vertex_mirror_map_type;


template<typename _Tp>
_Tp & atr(std::vector<_Tp> & v, size_t el)
{
	if (el >= v.size()) {
		v.resize(el + 1);
	}

	return v.at(el);
}

/////////////////////////////////////////////////////////////////////
// define structs

/**
* This struct allows to access the outgoing edges for a certain vertex
*/
struct BySourceParam {
	BySourceParam(boost::property_map<aGraph, edge_data_t>::type const & _edgeDataMap)
		: edgeDataMap(_edgeDataMap)
	{}

	bool operator()(edge_descr const & a, edge_descr const & b) const {
		return edgeDataMap[a].sourceParam < edgeDataMap[b].sourceParam;
	}

	boost::property_map<aGraph, edge_data_t>::type	edgeDataMap;
};

/**
* This struct allows to access the incoming edges for a certain vertex
*/
struct ByTargetParam {
	ByTargetParam(boost::property_map<aGraph, edge_data_t>::const_type const & _edgeDataMap)
		: edgeDataMap(_edgeDataMap)
	{}

	bool operator()(edge_descr const & a, edge_descr const & b) const {
		return edgeDataMap[a].targetParam < edgeDataMap[b].targetParam;
	}

	boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap;
};

/**
* This struct allows to access the outgoing edges for a certain vertex
*/
struct CompareToSource {
	CompareToSource(boost::property_map<aGraph, edge_data_t>::type const & _edgeDataMap)
		: edgeDataMap(_edgeDataMap)
	{}

	bool operator()(edge_descr const & e, math::flt param) const {
		return edgeDataMap[e].sourceParam < param;
	}

	boost::property_map<aGraph, edge_data_t>::type edgeDataMap;
};

struct CompareToTarget {
	CompareToTarget(boost::property_map<aGraph, edge_data_t>::type const & _edgeDataMap)
		: edgeDataMap(_edgeDataMap)
	{}

	bool operator()(edge_descr const & e, math::flt param) const {
		return edgeDataMap[e].targetParam < param;
	}

	boost::property_map<aGraph, edge_data_t>::type edgeDataMap;
};

template<typename Comparator>
std::pair<edge_descr, bool> findEdge(std::vector<edge_descr> const & references, math::flt closestParm, boost::property_map<aGraph, edge_data_t>::type const & edgeDataMap)
{
	std::vector<edge_descr>::const_iterator lit = std::lower_bound(references.begin() + 1, references.end(), closestParm, Comparator(edgeDataMap)) - 1;

	for (; lit != references.end(); ++lit) {
		math::flt const param = edgeDataMap[*lit].paramDir() * closestParm;
		math::flt	const mlow = edgeDataMap[*lit].paramDir() * edgeDataMap[*lit].sourceParam;
		math::flt	const mupp = edgeDataMap[*lit].paramDir() * edgeDataMap[*lit].targetParam;

		if (mlow <= param && param <= mupp) {
			return std::make_pair(*lit, true);
		}
		else if (mlow > param) {
			break;
		}
	}

	return std::make_pair(edge_descr(), false);
}


/* concat two integers into a string seperated by a dot */
std::string l1string(uint l0, uint l1)
{
	std::stringstream strstr;
	strstr << l0 << '.' << l1;
	return strstr.str();
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Pimpl ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct RNDFGraph::Pimpl {

	typedef std::vector<math::flt> NeighbourStatistics;
	typedef std::vector<NeighbourStatistics> SegmentStatistics;
	typedef std::vector<SegmentStatistics> GlobalNeighbourStatistics;
	typedef std::pair<uint, uint> NeighbourResults;
	typedef std::vector<NeighbourResults> SegmentNeighbourResults;
	typedef std::vector<SegmentNeighbourResults> GlobalNeighbourResults;

	Pimpl()
		: sourceExtend("RndfSourceExtend", "", 0.1)
		, sourceLeft("RndfSourceLeft", "", 0.0)
		, targetExtend("RndfTargetExtend", "", 0.1)
		, targetLeft("RndfFTargetLeft", "", 0.0)
		, apex("RndfApex", "", 0.66)
		, apexTop("RndfApexTop", "", 0.0)
		, mutex(QMutex::Recursive) {
		bag.addProperty(sourceExtend);
		bag.addProperty(sourceLeft);
		bag.addProperty(targetExtend);
		bag.addProperty(targetLeft);
		bag.addProperty(apex);
		bag.addProperty(apexTop);

		// init shortcuts to edge data and vertex data
		vertexDataMap = get(vertex_data_t(), boostGraph);
		edgeDataMap = get(edge_data_t(), boostGraph);
	}

	RTT::PropertyBag bag;
	RTT::Property<math::flt> sourceExtend, sourceLeft, targetExtend, targetLeft, apex, apexTop;

	// graph data as boost graph
	aGraph boostGraph;

	// edge data map (it's a shortcut)
	boost::property_map<aGraph, edge_data_t>::type edgeDataMap;

	// vertex data map (it's a shortcut)
	boost::property_map<aGraph, vertex_data_t>::type vertexDataMap;

	// vector containing all checkpoints
	std::vector<vertex_descr> checkpoints;
	ZoneInfo zoneInfo;

	reverse_map_type reverseMap;

	// graph hierarchically organised. Contains Segments, Lanes and Waypoints
	graph_vec_type mGraphHierarchy;

	// mutex
	QMutex mutex;

	vertex_mirror_map_type vertexMirrorMap;

	SplineKdTree nearestSpline;
	SplineKdTree::box_vector splineBBoxes;

	/////////////////////////////////////// outline methods /////////////////////////////////////////

	void splinify();
	bool findSandwitchingEdges(std::vector<math::flt> const & distances,
							   uint & fromEdge, uint & toEdge) const;

	bool getDirectNeighbouredEdge(math::flt param, edge_descr const & edge, bool left,
								  edge_descr & neighbourEdge, math::Vec2 & neighbourPos,  math::flt & closestParm,
								  math::flt & closestSquaredDist) const;

	bool getDirectNeighbouredEdge(math::Vec2 const & pos, edge_descr const & edge, bool left,
								  edge_descr & neighbourEdge, math::Vec2 & neighbourPos,  math::flt & closestParm,
								  math::flt & closestSquaredDist) const;

	std::pair <math::flt, math::flt> getDirectNeighbouredEdge(math::flt param, edge_descr const & edge, edge_descr & rLeft, edge_descr & rRight, math::Vec2 & rAvoidPosLeft, math::Vec2 & rAvoidPosRight) const;

	/////////////////////////////////////// inline methods /////////////////////////////////////////

	// find all outgoing connections
	// please note, that in a zone also zone connections will be found
	std::vector<edge_descr> getOutgoingConnections(vertex_descr v) {
		std::vector<edge_descr> result;
		BOOST_FOREACH(edge_descr const & e, out_edges(v, boostGraph)) {
			if (edgeDataMap[e].isConnection) {
				result.push_back(e);
			}
		}
		return result;
	}

	void promoteInsertedPoints(uint segmentID, uint laneID) {
		// get data
		std::vector<vertex_descr> wayPoints = getLaneWaypoints(segmentID, laneID);

		// iterate on the way points and update IDs
		for (uint waypointID = 0; waypointID < wayPoints.size(); ++waypointID) {
			VertexData & vData = vertexDataMap[wayPoints[waypointID]];
			vData.id = waypointID + 1;
			vData.vertexType.unset(VertexData::INSERTED);
			vData.vertexType.set(VertexData::WAY_POINT);
		}
	}

	/**
	  returns a vector of vertex_descr containing all the waypoints of a given laneID in segmentID
	  **/
	std::vector<vertex_descr> getLaneWaypoints(uint segmentID, uint laneID) {
		// add source vertex of first edge
		std::vector<vertex_descr> result;
		result.push_back(source(mGraphHierarchy.first[segmentID - 1][laneID - 1][0], boostGraph));

		// iterate on the edges, and add their target vertices
		BOOST_FOREACH(edge_descr const & edge, mGraphHierarchy.first[segmentID - 1][laneID - 1]) {
			result.push_back(target(edge, boostGraph));
		}
		return result;
	}

	void sortReferencesBySource() {
		//boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), const_cast<aGraph const &>(boostGraph));

		BOOST_FOREACH(reverse_map_type::reference rev, reverseMap) {
			std::vector<edge_descr> & references = rev.second.first;

			BOOST_FOREACH(edge_descr const & e, references) {
				assert(edgeDataMap[e].sourceParam <= edgeDataMap[e].targetParam);
			}

			std::sort(references.begin(), references.end(), BySourceParam(edgeDataMap));
			std::vector<edge_descr> & rreferences = rev.second.second;

			BOOST_FOREACH(edge_descr const & e, rreferences) {
				assert(edgeDataMap[e].sourceParam >= edgeDataMap[e].targetParam);
			}

			std::sort(rreferences.begin(), rreferences.end(), BySourceParam(edgeDataMap));
		}
	}

	void rectify() {
		math::LaneSpline::DomVec xs;
		math::LaneSpline::ImgVec ys;
		BOOST_FOREACH(reverse_map_type::const_reference val, reverseMap) {
			math::LaneSpline & spline = const_cast<math::LaneSpline &>(*val.first);
			math::LaneSpline::DomVec const & oldxs = spline.abscissae();
			xs.clear();
			ys.clear();
			xs.reserve(oldxs.size());
			ys.reserve(oldxs.size());
			xs.push_back(0.0f);
			ys.push_back(spline(oldxs.front()));

			for (math::LaneSpline::DomVec::const_iterator xit = oldxs.begin(); xit != oldxs.end() - 1; ++xit) {
				math::flt from = *xit, to = *boost::next(xit);
				math::flt numerical = ::math::adaptive_simpsons_rule(spline, from, to, 1e-4f);
				xs.push_back(xs.back() + numerical);
				ys.push_back(spline(to));
			}

			std::vector<edge_descr> const & edges = val.second.first;
			assert(val.second.second.empty());

			BOOST_FOREACH(edge_descr const & edge, edges) {
				uint sourceIdx = std::find(oldxs.begin(), oldxs.end(), edgeDataMap[edge].sourceParam) - oldxs.begin();
				uint targetIdx = std::find(oldxs.begin(), oldxs.end(), edgeDataMap[edge].targetParam) - oldxs.begin();

				edgeDataMap[edge].sourceParam = xs[sourceIdx];
				edgeDataMap[edge].targetParam = xs[targetIdx];
			}

			spline = math::LaneSpline(xs, ys);
		}
	}

	// isALeftOfBWeight > 0 -> a left of b with strength abs(isALeftOfBWeight)
	static void addStatistic(SegmentStatistics & ns, uint a , uint b, math::flt isALeftOfBWeight) {
		if (ns.size() < a) {
			ns.resize(a + 1);
		}

		if (ns[a - 1].size() < b) {
			ns[a - 1].resize(b + 1);
		}

		ns[a - 1][b - 1] += isALeftOfBWeight;
	}

	/**
	  votes which splines are neighbor
	  **/
	void neighbourSplines() {
		bool success;
		math::flt const defaultLaneWidth = 2.5f;

		GlobalNeighbourStatistics nsv(mGraphHierarchy.first.size());


		//look at each vertex in graph (v1)
		BOOST_FOREACH(vertex_descr v1, vertices(boostGraph)) {
			VertexData const & v1Data = vertexDataMap[v1];

			//if vertex is in zone do nothing
			if (v1Data.vertexType.isSet(VertexData::IN_ZONE)) {
				continue;
			}

			//get inEdge (if we have one) else outEdge (if we have one)
			//else (if we have no in or out edge) do nothing
			std::pair<edge_descr, bool> v1Edge = inEdge(v1);

			if (!v1Edge.second) {
				v1Edge = outEdge(v1);
			}

			if (!v1Edge.second) {
				continue;
			}

			// get direction vector of lane at the vertex position
			math::Vec2 trackDir1;
			success = trackDir(trackDir1, v1);

			if (!success) {
				continue;
			}

			// get the edge data and reference to the statistics for the segment
			EdgeData const & edgeData1 = edgeDataMap[v1Edge.first];
			SegmentStatistics & ns = nsv[edgeData1.segment - 1];

			//get lane width
			math::flt const laneWidth1 = v1Data.laneWidth < 0.1f ? defaultLaneWidth : v1Data.laneWidth;


			//get each other vertex from the graph (v2)
			BOOST_FOREACH(vertex_descr v2, vertices(boostGraph)) {
				if (v2 == v1) {
					continue;
				}

				VertexData const & v2Data = vertexDataMap[v2];

				// we only want vertices in our segment
				if (v1Data.l0 != v2Data.l0) {
					continue;
				}

				// and only other lanes
				if (v1Data.l1 == v2Data.l1) {
					continue;
				}

				// get squared distance between vertices v1 and v2
				const math::flt dist2 = math::ssd(v1Data.pos, v2Data.pos);

				if (dist2 > 10.f * 10.f) {
					continue;
				}

				std::pair<edge_descr, bool> v2Edge = inEdge(v2);

				if (!v2Edge.second) {
					v2Edge = outEdge(v2);
				}

				if (!v2Edge.second) {
					DLOG("offending vertex, from: " << v1Data.name << " to: " << v2Data.name << "!");
					assert(v2Edge.second);
				}

				EdgeData const & edgeData2 = edgeDataMap[v2Edge.first];

				math::flt const laneWidth2 = (v2Data.laneWidth < 0.1f) ? defaultLaneWidth : v2Data.laneWidth;
				math::flt const laneDistance = 0.5f * (laneWidth1 + laneWidth2);

				// we have 2 vertices not to far from each other,
				// that belong to the same street segment, but different lanes
				math::flt const dist = std::abs(laneDistance - sqrt(dist2));

				if (dist < 2.5f) {
					math::Vec2 trackDir2;
					success = trackDir(trackDir2, v2);
					assert(success);

					bool v2LeftofV1 = ::math::leftOfTest(v2Data.pos, v1Data.pos, v1Data.pos + trackDir1);
					bool v1LeftofV2 = ::math::leftOfTest(v1Data.pos, v2Data.pos, v2Data.pos + trackDir2);

					math::flt const weight = 1.f / (1.f + dist);
					math::flt const v2LeftofV1Weight = (v2LeftofV1 ? 1.f : -1.f) * weight;
					math::flt const v1LeftofV2Weight = (v1LeftofV2 ? 1.f : -1.f) * weight;

					addStatistic(ns, edgeData1.lane, edgeData2.lane, v2LeftofV1Weight);
					addStatistic(ns, edgeData2.lane, edgeData1.lane, v1LeftofV2Weight);
				}
			}
		}

		// Tally the votes in each segment..
		GlobalNeighbourResults results(mGraphHierarchy.first.size());

		for (uint segment = 0; segment < nsv.size(); ++segment) {
			SegmentStatistics const & ss = nsv[segment];
			SegmentNeighbourResults & snr = results[segment];

			snr.resize(ss.size());

			// ...and for each lane
			for (uint lane = 0; lane < ss.size(); ++lane) {
				NeighbourStatistics const & ns = ss[lane];
				math::flt bestLeft = 0.0f, bestRight = 0.0f;
				NeighbourResults & res = snr[lane];
				res = std::make_pair(0, 0);

// 				assert(ns.size() < 4);		WARNING check if this is ok to comment out
				for (uint otherlane = 0; otherlane < ns.size(); ++otherlane) {
					if (ns[otherlane] > bestLeft) {
						res.first = otherlane + 1;
						bestLeft = ns[otherlane];
					}
					else if (ns[otherlane] < bestRight) {
						res.second = otherlane + 1;
						bestRight = ns[otherlane];
					}
				}
			}
		}

		// Now set the correspondencies
		BOOST_FOREACH(edge_descr const & e, edges(boostGraph)) {
			EdgeData & edgeData = edgeDataMap[e];

			if (edgeData.isConnection
					|| edgeData.isMirrored
					|| edgeData.inZone
					|| edgeData.isUTurn
			   ) {
				continue;
			}

			VertexData const & v1 = vertexDataMap[source(e, boostGraph)];
			VertexData const & v2 = vertexDataMap[target(e, boostGraph)];

			SegmentNeighbourResults const & snr = results[edgeData.segment - 1];

			if (snr.empty()) {
				continue;
			}

			assert(edgeData.lane <= snr.size());

			NeighbourResults res = snr[edgeData.lane - 1];

			assert(mGraphHierarchy.first.size() >= edgeData.segment);

			if (res.first > 0) {
				if (!(mGraphHierarchy.first[edgeData.segment - 1].size() >= res.first)) {
					std::cout << "Segment ID: " << edgeData.segment << "GraphHierachy Size: " << mGraphHierarchy.first[edgeData.segment - 1].size() << "left Neighbor ID: " << res.first << std::endl;
				}

				assert(mGraphHierarchy.first[edgeData.segment - 1].size() >= res.first);
				lane_vec_type const & lane = mGraphHierarchy.first[edgeData.segment - 1][res.first - 1];
				computeParam(edgeDataMap, lane, edgeData.leftLaneSpline(), edgeData.leftLaneParamRange, v1.pos, v2.pos);
			}

			if (res.second > 0) {
				assert(mGraphHierarchy.first[edgeData.segment - 1].size() >= res.second);
				lane_vec_type const & lane = mGraphHierarchy.first[edgeData.segment - 1][res.second - 1];
				computeParam(edgeDataMap, lane, edgeData.rightLaneSpline(), edgeData.rightLaneParamRange, v1.pos, v2.pos);
			}
		}
	}

	static void computeParam(boost::property_map<aGraph, edge_data_t>::type & edgeDataMap, lane_vec_type const & lane, boost::shared_ptr<math::LaneSpline> & ls, std::pair<math::flt, math::flt> & paramRange, math::Vec2 const & v1pos, math::Vec2 const & v2pos) {
		unsigned int ix = 0;
		math::flt sourceMinDist = std::numeric_limits<math::flt>::max();
		math::flt targetMinDist = std::numeric_limits<math::flt>::max();
		assert(!lane.empty());
		boost::shared_ptr<math::LaneSpline> const & lsp(edgeDataMap[lane.front()].laneSpline());

		ls = lsp;

		BOOST_FOREACH(lane_vec_type::const_reference e, lane) {
			ix++;
			EdgeData const & otherEdgeData = edgeDataMap[e];
			assert(lsp == otherEdgeData.laneSpline());	// The lane-spline shouldn't change in a lane

			math::flt dist, param1;
			boost::tie(dist, param1) = ::math::findClosestPoint(*lsp,
									   otherEdgeData.sourceParam,
									   otherEdgeData.targetParam, 0.5f * (otherEdgeData.sourceParam + otherEdgeData.targetParam),
									   v1pos, 0.01f);

			if (dist < sourceMinDist) {
				sourceMinDist = dist;
				paramRange.first = param1;
			}

			boost::tie(dist, param1) = ::math::findClosestPoint(*lsp,
									   otherEdgeData.sourceParam,
									   otherEdgeData.targetParam, 0.5f * (otherEdgeData.sourceParam + otherEdgeData.targetParam),
									   v2pos, 0.01f);

			if (dist < targetMinDist) {
				targetMinDist = dist;
				paramRange.second = param1;
			}
		}

		if (paramRange.first > paramRange.second) {
			std::swap(paramRange.first, paramRange.second);
		}

		if (paramRange.second - paramRange.first < 0.01f) {
			paramRange.second = 0.0f;
			paramRange.first  = 0.0f;
			ls.reset();
		}
	}

	vertex_descr addWaypoint(uint l0, uint l1, uint id, math::Vec2 const & pos) {
		vertex_descr v = add_vertex(boostGraph);
		VertexData & vData = vertexDataMap[v];
		vData.pos = pos;
		vData.l0 = l0;
		vData.l1 = l1;
		vData.id = id;
		return v;
	}

	void addWaypoint(uint l0, uint l1, aa::modules::models::rndf::io::Waypoint & w) {
		assert(!w.vertex_is_set);

		math::Vec3d transformed = math::geodetic::WGS84toENU(math::Vec2d(w.longitude, w.latitude));

		w.vertex = addWaypoint(l0, l1, w.id, math::Vec2(transformed(0), transformed(1)));
		w.vertex_is_set = true;

		VertexData & vertexData = vertexDataMap[w.vertex];
		vertexData.checkpoint = w.checkpoint_id;
		vertexData.laneWidth = w.laneWidth * ::math::FT_2_M;;

		// VertexData::ENTRY is set in buildGraph
//		vertexData.vertexType.set(VertexData::EXIT, !w.exit_to.empty());
		vertexData.vertexType.set(VertexData::STOP_SIGN, w.stopsign);
		vertexData.vertexType.set(VertexData::TRAFFIC_LIGHT, w.trafficLight);
		vertexData.vertexType.set(VertexData::TRAFFIC_LIGHT_TWO_PHASES, w.trafficLightTwoPhases);
		vertexData.vertexType.set(VertexData::DECISION_POINT, w.decisionPoint);
		vertexData.vertexType.set(VertexData::GIVE_WAY, w.giveWay);
		vertexData.vertexType.set(VertexData::PERIMETER_POINT, w.isPerimeterPoint);
		vertexData.vertexType.set(VertexData::PARKINGSPOT, w.isSpot);
		vertexData.vertexType.set(VertexData::WAY_POINT, !w.isPerimeterPoint);
		vertexData.vertexType.set(VertexData::CHECK_POINT, w.checkpoint_id > 0);
		vertexData.vertexType.set(VertexData::IN_ZONE, w.isPerimeterPoint || w.isSpot || w.zoneLaneWP);

		if (vertexData.checkpoint > 0) {
			if (checkpoints.size() < vertexData.checkpoint) {
				checkpoints.resize(vertexData.checkpoint);
			}

			checkpoints[vertexData.checkpoint - 1] = w.vertex;
		}

		updateVertexName(w.vertex);
	}

	std::pair<edge_descr, bool> inEdge(vertex_descr v) const {
		//boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), boostGraph);

		BOOST_FOREACH(edge_descr const & e, in_edges(v, boostGraph)) {
			EdgeData const & inEdgeData = edgeDataMap[e];

			if (inEdgeData.isConnection) {
				continue;
			}

			return std::make_pair(e, true);
		}

		return std::make_pair(edge_descr(), false);
	}

	bool inDir(math::Vec2 & dir, vertex_descr v) const {
		std::pair<edge_descr, bool> e = inEdge(v);

		if (!e.second) {
			return false;
		}

		EdgeData const & edgeData = get(edge_data_t(), boostGraph)[e.first];

		math::flt const len =	edgeData.paramDir() * (edgeData.targetParam - edgeData.sourceParam);
		math::flt const step = edgeData.paramDir() * std::min(math::flt(0.1) * len, math::flt(0.1));

		if (edgeData.laneSpline().get()) {
			dir = edgeData.paramDir() * math::normalized(edgeData.laneSpline()->firstDerivative(edgeData.targetParam - step));
		}
		else {
			VertexData const & v1 = get(vertex_data_t(), boostGraph)[source(e.first, boostGraph)];
			VertexData const & v2 = get(vertex_data_t(), boostGraph)[target(e.first, boostGraph)];
			dir = v2.pos - v1.pos;
			dir.normalize();
		}

		assert(std::abs(dir.squaredNorm() - 1.0f) < 1e-5f);

		return true;
	}

	std::pair<edge_descr, bool> outEdge(vertex_descr v) const {
		//boost::property_map<aGraph, edge_data_t>::const_type	edgeDataMap = get(edge_data_t(), boostGraph);

		BOOST_FOREACH(edge_descr const & e, out_edges(v, boostGraph)) {
			EdgeData const & outEdgeData = edgeDataMap[e];

			if (outEdgeData.isConnection) {
				continue;
			}

			return std::make_pair(e, true);
		}

		return std::make_pair(edge_descr(), false);
	}

	bool outDir(math::Vec2 & dir, vertex_descr v) const {
		std::pair<edge_descr, bool> e = outEdge(v);

		if (!e.second) {
			return false;
		}

		EdgeData const & edgeData = get(edge_data_t(), boostGraph)[e.first];

		math::flt const len =	edgeData.paramDir() * (edgeData.targetParam - edgeData.sourceParam);
		math::flt const step =	edgeData.paramDir() * std::min(math::flt(0.1) * len, math::flt(0.1));

		if (edgeData.laneSpline().get()) {
			dir = edgeData.paramDir() * math::normalized(edgeData.laneSpline()->firstDerivative(edgeData.sourceParam + step));
		}
		else {
			VertexData const & v1 = get(vertex_data_t(), boostGraph)[source(e.first, boostGraph)];
			VertexData const & v2 = get(vertex_data_t(), boostGraph)[target(e.first, boostGraph)];
			dir = v2.pos - v1.pos;
			dir.normalize();
		}

		assert(std::abs(dir.squaredNorm() - math::flt(1)) < math::flt(1e-5));

		return true;
	}

	bool trackDir(math::Vec2 & dir, vertex_descr v) const {
		return outDir(dir, v) || inDir(dir, v);
	}

	math::Vec2 perimeterDir(VertexData const & v1Data, boost::property_map<aGraph, vertex_data_t>::type const & vertexDataMap);

	/// First pass: Create all points and fill out their properties
	void createVertices(aa::modules::models::rndf::io::RNDFData & r) {

		// iterate on segments
		BOOST_FOREACH(Segment & seg, r.segments) {
			BOOST_FOREACH(Lane & lane, seg.lanes) {
				BOOST_FOREACH(Waypoint & w, lane.waypoints) {
					addWaypoint(seg.id, lane.id, w);
				}
			}
		}

		// iterate on zones
		BOOST_FOREACH(Zone & zone, r.zones) {
			BOOST_FOREACH(Waypoint & w, zone.perimeterpoints) {
				addWaypoint(zone.id, 0, w);
			}

			BOOST_FOREACH(Spot & spot, zone.spots) {
				BOOST_FOREACH(Waypoint & w, spot.waypoints) {
					addWaypoint(zone.id, spot.id, w);
				}
			}
		}
	}

	edge_descr addConnectingEdge(vertex_descr source, vertex_descr target) {
		// check precondition, that source != target
		assert(source != target);

		// Check precondition, that the edge does not already exist
		bool success = false;
		edge_descr ed;
		boost::tie(ed, success) = edge(source, target, boostGraph);
		assert(!success);

		// get vertex data
		VertexData & sourceData = vertexDataMap[source];
		VertexData & targetData = vertexDataMap[target];

		// setting source vertexType to EXIT
		sourceData.vertexType.set(VertexData::EXIT);
		// setting target vertexType to ENTRY
		targetData.vertexType.set(VertexData::ENTRY);


		// add edge to the boost graph and set its weight
		boost::tie(ed, success) = add_edge(source, target, boostGraph);
		assert(success);
		get(boost::edge_weight_t(), boostGraph)[ed] = (sourceData.pos - targetData.pos).norm();
		assert(get(boost::edge_weight_t(), boostGraph)[ed] > 0.f);

		// set edge data properties and return
		edgeDataMap[ed].isConnection = true;
		// not needed anymore, uturns should be expensive without using the uturn flag
		//edgeDataMap[ed].isUTurn = (sourceData.l0 == targetData.l0 && sourceData.l1 != targetData.l1);
		return ed;
	}

	uint createNewSegment() {
		// resize our segments vector
		mGraphHierarchy.first.resize(mGraphHierarchy.first.size() + 1);

		// iterate on zones and shift indices
		BOOST_FOREACH(zone_vec_type zone, mGraphHierarchy.second) {
			// iterate on perimeter points
			BOOST_FOREACH(vertex_descr v, zone.first) {
				vertexDataMap[v].l0++;
			}

			// iterate on spots
			BOOST_FOREACH(edge_descr const & spot, zone.second) {
				edgeDataMap[spot].segment++;
				vertexDataMap[source(spot, boostGraph)].l0++;
				vertexDataMap[target(spot, boostGraph)].l0++;
			}
		}

		// return the new segment ID
		return mGraphHierarchy.first.size();
	}

	uint createNewLane(uint segmentID) {
		// assert, that the segment ID is valid
		assert(segmentID <= mGraphHierarchy.first.size());

		// create a new lane and return the lane ID

		mGraphHierarchy.first[segmentID - 1].resize(mGraphHierarchy.first[segmentID - 1].size() + 1);
		return mGraphHierarchy.first[segmentID - 1].size();
	}

	edge_descr addSpotLane(vertex_descr source, vertex_descr target, aa::modules::models::rndf::io::Spot const & spot) {
		bool success = false;
		edge_descr ed;

		// check preconditions
		assert(source != target);
		assert(spot.zone != 0);
		assert(spot.id != 0);

		// Check, that the edge does not already exist
		boost::tie(ed, success) = edge(source, target, boostGraph);
		assert(!success);

		// check, that the zone already exists
		uint zoneIndex = spot.zone - mGraphHierarchy.first.size() - 1;
		assert(zoneIndex < mGraphHierarchy.second.size());

		// check, that the spot not yet exists
		zone_vec_type & zoneVec = mGraphHierarchy.second[zoneIndex];
		assert(spot.id == zoneVec.second.size() + 1);

		// add edge to the boost graph
		boost::tie(ed, success) = add_edge(source, target, boostGraph);
		assert(success);

		// add edge to the graph
		zoneVec.second.push_back(ed);

		math::flt const spotWidth = spot.width * ::math::FT_2_M;

		// set edge data
		EdgeData & edgeData = get(edge_data_t(), boostGraph)[ed];
		edgeData.inZone = true;
		edgeData.boundaryLeft = LANE_BOUNDARY_BROKEN_WHITE;
		//edgeData.width = spot.width;
		edgeData.segment = spot.zone;
		edgeData.lane = spot.id;


		boost::property_map<aGraph, vertex_data_t>::type vertexDataMap = get(vertex_data_t(), boostGraph);
		/* No other edge will determine the width */
		vertexDataMap[source].laneWidth = spotWidth;          //reenable spot width
		vertexDataMap[target].laneWidth = spotWidth;          //reenable spot width

		// return edge
		return ed;
	}

	edge_descr addLaneEdge(vertex_descr source, vertex_descr target, aa::modules::models::rndf::Lane const & lane) {
		// Local scope for edge
		bool success = false;
		edge_descr ed;

		// check preconditions
		if (source == target) {
			RTT::log(RTT::Error) << vertexDataMap[source].name << " to " << vertexDataMap[target].name << RTT::endlog();
		}

		assert(source != target);
		assert(lane.segment != 0);
		assert(lane.id != 0);

		// Check, that the edge does not already exist
		boost::tie(ed, success) = edge(source, target, boostGraph);
		assert(!success);

		// check, that the segment already exists
		assert(lane.segment <= mGraphHierarchy.first.size());

		// get the lanes vector for our segment and check, that the lane already exists
		segment_vec_type & laneVec = mGraphHierarchy.first[lane.segment - 1];
		assert(lane.id <= laneVec.size());

		// add edge to the boost graph
		boost::tie(ed, success) = add_edge(source, target, boostGraph);
		assert(success);

		// add edge to the lane
		laneVec[lane.id - 1].push_back(ed);

		// get the vertex data and the edge data
		VertexData const & v0Data = vertexDataMap[source];
		VertexData const & v1Data = vertexDataMap[target];
		EdgeData & edgeData = get(edge_data_t(), boostGraph)[ed];

		// copy properties from the lane parameter to the edge data

		edgeData.boundaryLeft = lane.left_boundary;
		edgeData.boundaryRight = lane.right_boundary;
		edgeData.inZone = lane.zone_lane;
		edgeData.isConnection = false;
		edgeData.isHidden = lane.isHidden;
		edgeData.segment = lane.segment;
		edgeData.lane = lane.id;
		edgeData.name = l1string(lane.segment, lane.id);

		// set the edge weight
		get(boost::edge_weight_t(), boostGraph)[ed] = (v0Data.pos - v1Data.pos).norm();

		// Update the edge parameters
		//math::flt const laneWidth = lane.width * ::math::FT_2_M;
		//vertexDataMap[source].laneWidth = laneWidth;
		//vertexDataMap[target].laneWidth = laneWidth;

		// return the edge to the caller
		return ed;
	}

	edge_descr addZoneEdge(vertex_descr v0, vertex_descr v1) {
		bool success = false;
		edge_descr ed;
		assert(v0 != v1);

		boost::tie(ed, success) = edge(v0, v1, boostGraph);	// Check, that the edge does not already exists
		assert(!success);

		boost::tie(ed, success) = add_edge(v0, v1, boostGraph);
		assert(success);

		VertexData const & v0Data = vertexDataMap[v0];
		VertexData const & v1Data = vertexDataMap[v1];


		EdgeData & edgeData = get(edge_data_t(), boostGraph)[ed];

		boost::tokenizer<boost::char_separator<char> > tok(v0Data.name, boost::char_separator<char>("."));

		if (v0Data.vertexType.isSet(VertexData::IN_ZONE)) {
			edgeData.name = l1string(v0Data.l0, v0Data.l1);
			edgeData.segment = v0Data.l0;
		}
		else {
			edgeData.name = l1string(v1Data.l0, v1Data.l1);
			edgeData.segment = v1Data.l0;
		}

		edgeData.name = *(tok.begin()) + "." + *(++tok.begin());
		edgeData.inZone = true;
		edgeData.boundaryLeft = LANE_BOUNDARY_BROKEN_WHITE;
		edgeData.boundaryRight = LANE_BOUNDARY_BROKEN_WHITE;

		// weight HIGHER, so ASTAR should prefer the fake ZoneLanes to go through Zones !!!
		get(boost::edge_weight_t(), boostGraph)[ed] = 100.0f * (v0Data.pos - v1Data.pos).norm();

		// add to zones

		return ed;
	}

	void mirrorEdge(edge_descr const & e) {

		bool success;
		edge_descr mirrorEdge;

		boost::tie(mirrorEdge, success) = add_edge(vertexMirrorMap[target(e, boostGraph)], vertexMirrorMap[source(e, boostGraph)], boostGraph);

		assert(success);

		EdgeData const & edgeData = edgeDataMap[e];
		EdgeData & mirrorEdgeData = edgeDataMap[mirrorEdge];
		// Now, we have to adopt the data from the original edge
		get(boost::edge_weight_t(), boostGraph)[mirrorEdge] = 10.0f * get(boost::edge_weight_t(), boostGraph)[e];

		mirrorEdgeData.setParamDir(-1.0f);
		mirrorEdgeData.isMirrored	=	true;
		mirrorEdgeData.name			= edgeData.name + "m";
		mirrorEdgeData.segment		= edgeData.segment;
		mirrorEdgeData.lane			= edgeData.lane;
// 		mirrorEdgeData.width			= edgeData.width;
		mirrorEdgeData.laneSpline()		= boost::const_pointer_cast<math::LaneSpline>(edgeData.laneSpline());

		mirrorEdgeData.rightLaneSpline()	= boost::const_pointer_cast<math::LaneSpline>(edgeData.leftLaneSpline());
		mirrorEdgeData.leftLaneSpline()	= boost::const_pointer_cast<math::LaneSpline>(edgeData.rightLaneSpline());
		mirrorEdgeData.rightLaneParamRange	= edgeData.leftLaneParamRange;
		mirrorEdgeData.leftLaneParamRange	= edgeData.rightLaneParamRange;
		mirrorEdgeData.sourceParam		= edgeData.targetParam;
		mirrorEdgeData.targetParam		= edgeData.sourceParam;
		mirrorEdgeData.boundaryRight	= edgeData.boundaryLeft;
		mirrorEdgeData.boundaryLeft		= edgeData.boundaryRight;


		reverseMap[mirrorEdgeData.laneSpline().get()].second.push_back(mirrorEdge);
	}

	void buildMirrorGraph() {
		DLOG("Number of vertices: " << num_vertices(boostGraph));
		BOOST_FOREACH(vertex_descr vertex, vertices(boostGraph)) {
			VertexData const & vertexData = vertexDataMap[vertex];

			if (vertexData.vertexType.isSet(VertexData::IN_ZONE)) {	// No mirror-graph in zones
				continue;
			}

			if (vertexMirrorMap.count(vertex) > 0) {	// Already mirrored
				continue;
			}

			vertex_descr mirrorVertex = add_vertex(boostGraph);
			// vertexData can become invalid, due to add_vertex

			// Store the mirror-relation
			vertexMirrorMap[vertex] = mirrorVertex;
			vertexMirrorMap[mirrorVertex] = vertex;

			// Copy the vertex properties, but some flags, which are invalid for an mirror-point, because it can't be connected to other segments
			VertexData & mirrorVertexData = vertexDataMap[mirrorVertex];
			mirrorVertexData = vertexData;
			mirrorVertexData.name += "m";
			mirrorVertexData.vertexType.unset(VertexData::STOP_SIGN | VertexData::TRAFFIC_LIGHT | VertexData::TRAFFIC_LIGHT_TWO_PHASES | VertexData::DECISION_POINT | VertexData::GIVE_WAY | VertexData::CHECK_POINT | VertexData::EXIT | VertexData::ENTRY);
			mirrorVertexData.vertexType.set(VertexData::MIRROR_VERTEX);
		}

		//boost::property_map<aGraph, edge_data_t>::const_type	edgeDataMap = get(edge_data_t(), const_cast<aGraph const &>(boostGraph));

		// After creating the mirror-vertices, we now connect their edges
		// We can't go over the edges directly, as the global edge iterator will be invalid after an add_edge call
		BOOST_FOREACH(vertex_descr vertex, vertices(boostGraph)) {
			VertexData const & vertexData = get(vertex_data_t(), boostGraph)[vertex];

			if (vertexData.vertexType.isSet(VertexData::IN_ZONE)) {	// No mirror-graph in zones
				continue;
			}

			BOOST_FOREACH(edge_descr const & e, out_edges(vertex, boostGraph)) {
				EdgeData const & edgeData = edgeDataMap[e];

				if (edgeData.isConnection || edgeData.isMirrored) {
					continue;
				}

				mirrorEdge(e);
			}
		}
	}

	void buildSKDTree() {
		splineBBoxes.clear();
		BOOST_FOREACH(reverse_map_type::const_reference rev, reverseMap) {
			math::LaneSpline const * const pSpline = rev.first;
			math::LaneSpline::DomVec const & as = pSpline->abscissae();
			math::LaneSpline::CoeffVec const & cs = pSpline->coeffs();
			splineBBoxes.reserve(splineBBoxes.size() + as.size());

			// For the interval between two abscissae, determine the extrema
			// The extrema yield the bounding box of the spline segment
			for (uint i = 0; i < as.size() - 1; ++i) {
                BoundingBox bb;
				bb.spline = pSpline;
				bb.from = as[i];
				bb.to = as[i + 1];
				bb.lowerBound = cs[i][3];
				bb.upperBound = cs[i][3];
				math::flt delta = bb.to - bb.from;

				// y = 3.0f*cs[i][0] * x^2 + 2.0f * cs[i][1] * x^1 + cs[i][2] * x^0
				// x0/1 = -p/2 +- sqrt(p^2/4 - q)
				// p/2 = cs[i][1] / (3.0f*cs[i][0])
				// q    = cs[i][2] / (3.0f*cs[i][0])
				for (uint j = 0; j < 2; ++j) {
					bb.lowerBound(j) = std::min(bb.lowerBound(j), cs[i + 1][3](j));
					bb.upperBound(j) = std::max(bb.upperBound(j), cs[i + 1][3](j));

					if (cs[i][0](j) != 0.0f) {	// Otherwise it is linear, and we already have the bounds
						math::flt mp_2 = -cs[i][1](j) / (3.0f * cs[i][0](j));
						math::flt q = cs[i][2](j) / (3.0f * cs[i][0](j));
						math::flt p2 = mp_2 * mp_2;

						if (p2 > q) {
							math::flt sq = sqrt(p2 - q);
							math::flt x0 = mp_2 + sq;
							math::flt x1 = mp_2 - sq;

							if (0 < x0 && x0 < delta) {
								math::flt y = (*pSpline)(bb.from + x0)(j);
								bb.lowerBound(j) = std::min(bb.lowerBound(j), y);
								bb.upperBound(j) = std::max(bb.upperBound(j), y);
							}

							if (0 < x1 && x1 < delta) {
								math::flt y = (*pSpline)(bb.from + x1)(j);
								bb.lowerBound(j) = std::min(bb.lowerBound(j), y);
								bb.upperBound(j) = std::max(bb.upperBound(j), y);
							}
						}
						else if (p2 == q) {
							math::flt x = mp_2;

							if (0 < x && x < delta) {
								math::flt y = (*pSpline)(bb.from + x)(j);
								bb.lowerBound(j) = std::min(bb.lowerBound(j), y);
								bb.upperBound(j) = std::max(bb.upperBound(j), y);
							}
						}
					}
				}

				splineBBoxes.push_back(bb);
			}
		}

		nearestSpline.rebuildTree(&splineBBoxes);
	}

	void initConfidences() {
		BOOST_FOREACH(reverse_map_type::const_reference rev, reverseMap) {
			const_cast<math::LaneSpline &>(*rev.first).initialiseCovariance();
		}
	}

	static void mergeSplineData(boost::shared_ptr<math::LaneSpline> & mergedPointer, std::pair<math::flt, math::flt> & mergedParam,
								const LaneSplinePtr & inEdgePointer, const std::pair<math::flt, math::flt> & inEdgeParam,
								const LaneSplinePtr & outEdgePointer, const std::pair<math::flt, math::flt> & outEdgeParam) {

		mergedPointer = boost::const_pointer_cast<math::LaneSpline>(inEdgePointer ? inEdgePointer : outEdgePointer);

		if (inEdgePointer && outEdgePointer) {
			mergedParam.first = std::min(inEdgeParam.first, outEdgeParam.first);
			mergedParam.second = std::max(inEdgeParam.second, outEdgeParam.second);
		}
		else if (inEdgePointer) {
			mergedParam = inEdgeParam;
		}
		else {
			mergedParam = outEdgeParam;
		}
	}

	void pruneVertices() {
		//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);
		boost::property_map<aGraph, boost::edge_weight_t>::type edgeWeightMap = get(boost::edge_weight_t(), boostGraph);

		bool modified = true;

		unsigned long total = num_vertices(boostGraph);
		unsigned long vi = 0;

		while (vi < total) {
			vertex_descr v = vertex(vi, boostGraph);
			vi++;

			if (out_degree(v, boostGraph) != 1
					|| in_degree(v, boostGraph) != 1) {
				continue;
			}

			VertexData const & vertexData = vertexDataMap[v];

			if (vertexData.checkpoint > 0
					|| vertexData.vertexType.isSet(VertexData::STOP_SIGN)
					|| vertexData.vertexType.isSet(VertexData::TRAFFIC_LIGHT)
					|| vertexData.vertexType.isSet(VertexData::TRAFFIC_LIGHT_TWO_PHASES)
					|| vertexData.vertexType.isSet(VertexData::DECISION_POINT)
					|| vertexData.vertexType.isSet(VertexData::GIVE_WAY)) {
				continue;
			}

			edge_descr inedge = *in_edges(v, boostGraph).first;
			edge_descr outedge = *out_edges(v, boostGraph).first;
			EdgeData const & inEdgeData = edgeDataMap[inedge];
			EdgeData const & outEdgeData = edgeDataMap[outedge];

			if (inEdgeData.isConnection
					|| inEdgeData.isUTurn
					|| inEdgeData.inZone
					|| outEdgeData.isConnection
					|| outEdgeData.isUTurn
					|| outEdgeData.inZone
					|| inEdgeData.laneSpline() != outEdgeData.laneSpline()) {
				continue;
			}

			bool success;
			edge_descr newEdge;
			boost::tie(newEdge, success) = add_edge(source(inedge, boostGraph), target(outedge, boostGraph), boostGraph);
			assert(success);

			EdgeData & newEdgeData = edgeDataMap[newEdge];
			newEdgeData = inEdgeData;

			newEdgeData.targetParam = outEdgeData.targetParam;

			mergeSplineData(newEdgeData.leftLaneSpline(), newEdgeData.leftLaneParamRange, inEdgeData.leftLaneSpline(), inEdgeData.leftLaneParamRange, outEdgeData.leftLaneSpline(), outEdgeData.leftLaneParamRange);
			mergeSplineData(newEdgeData.rightLaneSpline(), newEdgeData.rightLaneParamRange, inEdgeData.rightLaneSpline(), inEdgeData.rightLaneParamRange, outEdgeData.rightLaneSpline(), outEdgeData.rightLaneParamRange);

			edgeWeightMap[newEdge] = std::abs(newEdgeData.targetParam - newEdgeData.sourceParam);

			clear_vertex(v, boostGraph);
			remove_vertex(v, boostGraph);

			--total;
			--vi;
		}

		// The vertex descriptors have become invalid

		std::vector<segment_vec_type> & segs(mGraphHierarchy.first);
		unsigned int const nSegments = segs.size();
		segs.clear();
		segs.resize(nSegments);
		BOOST_FOREACH(edge_descr const & e, edges(boostGraph)) {
			EdgeData const & edgeData = edgeDataMap[e];
			vertex_descr src(source(e, boostGraph));
			vertex_descr dst(source(e, boostGraph));
			VertexData const & srcData(vertexDataMap[src]);
			VertexData const & dstData(vertexDataMap[dst]);

			math::flt const dist = edgeData.laneSpline() ?
								   std::abs(edgeData.targetParam - edgeData.sourceParam)
								   : sqrt(math::ssd(srcData.pos, dstData.pos));

			if (edgeData.isMirrored || edgeData.inZone || edgeData.isUTurn) {
				edgeWeightMap[e] = 10.f * dist;
				continue;
			}

			edgeWeightMap[e] = dist;

			if (!edgeData.isConnection) {
				atr(segs[edgeData.segment - 1], edgeData.lane - 1).push_back(e);
			}
		}

		BOOST_FOREACH(segment_vec_type & s, segs) {
			BOOST_FOREACH(lane_vec_type & l, s) {
				std::sort(l.begin(), l.end(), BySourceParam(edgeDataMap));
			}
		}

		BOOST_FOREACH(vertex_descr v, vertices(boostGraph)) {
			VertexData const & vertexData = vertexDataMap[v];

			if (vertexData.checkpoint > 0) {
				assert(checkpoints.size() > vertexData.checkpoint - 1); // The number of checkpoints shouldn't be affected
				checkpoints[vertexData.checkpoint - 1] = v;
			}
		}

		zoneInfo.computeZones();
	}

	void fillReverseMap() {
		reverseMap.clear();

		//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);

		BOOST_FOREACH(edge_descr const & edge, edges(boostGraph)) {
			EdgeData & edgeData = edgeDataMap[edge];
			math::LaneSpline * pSpline = edgeData.laneSpline().get();

			if (!edgeData.isMirrored) {
				reverseMap[pSpline].first.push_back(edge);
			}
			else {
				reverseMap[pSpline].second.push_back(edge);
			}
		}

		sortReferencesBySource();
	}

	std::pair<edge_descr, bool> findNonMirroredEdge(math::flt param, boost::shared_ptr< ::math::LaneSpline const> laneSpline) const {
		std::vector<edge_descr> const & references = reverseMap.find(laneSpline.get())->second.first;
		return findEdge<CompareToTarget>(references, param, edgeDataMap);
	}

	std::pair<edge_descr, bool> findMirroredEdge(math::flt param, boost::shared_ptr< ::math::LaneSpline const> laneSpline) const {
		std::vector<edge_descr> const & references = reverseMap.find(laneSpline.get())->second.second;
		return findEdge<CompareToTarget>(references, param, edgeDataMap);
	}

	void addLaneChangingEdges() {
		static const math::flt laneChangeLength = 15.0f;
		//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);
		fillReverseMap();

		math::Vec2 avoidPosLeft, avoidPosRight;

		// We may add additional vertices, which should be ignored
		unsigned long const nvertices = num_vertices(boostGraph);

		for (unsigned long iv = 0; iv < nvertices; ++iv) {
			vertex_descr v = vertex(iv, boostGraph);
			VertexData const & vertexData = vertexDataMap[v];

			if (vertexData.vertexType.isSet(VertexData::PERIMETER_POINT | VertexData::IN_ZONE)) {
				continue;
			}

			std::pair<edge_descr, bool> e = inEdge(v);

			if (!e.second) {
				continue;
			}

			EdgeData const & edgeData = edgeDataMap[e.first];
			assert(edgeData.laneSpline().get()); // Otherwise, it wouldn't be an in-edge

			if (edgeData.isMirrored) {
				continue;
			}

			////// add lane changing edges on this edge
			if (out_degree(v, boostGraph) > 0) { // Not a dead-end
				// done and todo lists for lane changing edges on this edge (necessary for recursively adding lane changing edges, if there are more than 2 neighbour lanes )
				std::set<std::pair<int, int> > doneOn;
				std::deque<std::pair<edge_descr, math::flt> > todoOn;

				// Walk laneChangeLength along the spline backwards and get the edge from there. if it exists, put it in the todo lists
				math::flt const param = edgeData.targetParam - edgeData.paramDir() * laneChangeLength;
				std::pair<edge_descr, bool> edgeToProcess = findNonMirroredEdge(param, edgeData.laneSpline());

				if (edgeToProcess.second) {
					EdgeData const & edgeToProcessData = edgeDataMap[edgeToProcess.first];

					if (edgeToProcessData.segment == edgeData.segment) {
						//the new edge is on the same segment, so we can push it in the todo list
						if (out_degree(v, boostGraph) > 0) {
							//only add lane changing edges on this edge, if the lane goes on
							todoOn.push_back(std::make_pair(edgeToProcess.first, param));
						}
					}
				}


				// recursively add lane changing edges on this edge
				while (!todoOn.empty()) {
					edge_descr toEdge;
					math::flt toStartParam; // The parameter of a point neighbour to the hypothetical starting points
					boost::tie(toEdge, toStartParam) = todoOn.front();
					todoOn.pop_front();

					EdgeData const & toEdgeData = edgeDataMap[toEdge];
					std::pair<int, int> const id = std::make_pair(toEdgeData.segment, toEdgeData.lane);

					doneOn.insert(id);

					edge_descr leftFrom, rightFrom;
					std::pair<math::flt, math::flt> fromStartParam = getDirectNeighbouredEdge(toStartParam, toEdge, leftFrom, rightFrom, avoidPosLeft, avoidPosRight);

					// check if we have a neighboring lane to the left
					if (edge_descr() != leftFrom) {
						EdgeData const & leftFromData = edgeDataMap[leftFrom];
						math::flt leftStartParam = fromStartParam.first;

						if (!leftFromData.isConnection && doneOn.count(std::make_pair(leftFromData.segment, leftFromData.lane)) == 0) {
							edge_descr newedge = addLaneChange(leftFrom, leftStartParam, toEdge, toEdgeData.targetParam);
							std::pair<edge_descr, bool> laneChangeTargetInEdge = inEdge(source(newedge, boostGraph));

							if (laneChangeTargetInEdge.second) {
								EdgeData const & laneChangeTargetInEdgeData = edgeDataMap[laneChangeTargetInEdge.first];
								math::flt nextLaneChangeTargetParam = laneChangeTargetInEdgeData.targetParam - laneChangeTargetInEdgeData.paramDir() * laneChangeLength;
								std::pair<edge_descr, bool> nextLaneChangeTargetEdge = findNonMirroredEdge(nextLaneChangeTargetParam, laneChangeTargetInEdgeData.laneSpline());

								if (nextLaneChangeTargetEdge.second) {
									std::pair<edge_descr, math::flt>  nextTodoEdgePair = std::make_pair(nextLaneChangeTargetEdge.first, nextLaneChangeTargetParam);
									todoOn.push_back(nextTodoEdgePair);
								}
							}
						}
					}

					// check if we have a neighboring lane to the right
					if (edge_descr() != rightFrom) {
						EdgeData const & rightFromData = edgeDataMap[rightFrom];
						math::flt rightStartParam = fromStartParam.second;

						if (!rightFromData.isConnection && doneOn.count(std::make_pair(rightFromData.segment, rightFromData.lane)) == 0) {
							edge_descr newedge = addLaneChange(rightFrom, rightStartParam, toEdge, toEdgeData.targetParam);
							std::pair<edge_descr, bool> laneChangeTargetInEdge = inEdge(source(newedge, boostGraph));

							if (laneChangeTargetInEdge.second) {
								EdgeData const & laneChangeTargetInEdgeData = edgeDataMap[laneChangeTargetInEdge.first];
								math::flt nextLaneChangeTargetParam = laneChangeTargetInEdgeData.targetParam - laneChangeTargetInEdgeData.paramDir() * laneChangeLength;
								std::pair<edge_descr, bool> nextLaneChangeTargetEdge = findNonMirroredEdge(nextLaneChangeTargetParam, laneChangeTargetInEdgeData.laneSpline());

								if (nextLaneChangeTargetEdge.second) {
									std::pair<edge_descr, math::flt>  nextTodoEdgePair = std::make_pair(nextLaneChangeTargetEdge.first, nextLaneChangeTargetParam);
									todoOn.push_back(nextTodoEdgePair);
								}
							}
						}
					}
				} // while (!todoOn.empty())
			} // if (out_degree(v, boostGraph) > 0)

			////// add lane changing edges off of this edge
			if (!outEdge(v).second) { // the lane ends (the vertex has no outgoing edges except connecting)
				// Calling addLaneChange modifies the fromEdge descriptor, so we cannot keep a todo-list
				// Re-get the inEdge, as it may have been modified.
				e = inEdge(v);
				assert(e.second);
				math::flt const startParam = edgeData.targetParam - edgeData.paramDir() * laneChangeLength;

				EdgeData const & fromEdgeData = edgeDataMap[e.first];

				// walk laneChangeLength along the spline and (if we are still on the same segment) get the neighbour edges/params from there
				edge_descr leftTo, rightTo;
				math::flt startParamPlusLaneChangeLengthParam = startParam + fromEdgeData.paramDir() * laneChangeLength;
				std::pair<edge_descr, bool> fromEdgePlusLaneChangeLengthEdge = findNonMirroredEdge(startParamPlusLaneChangeLengthParam, fromEdgeData.laneSpline());

				std::pair<math::flt, math::flt> toEndParam;

				if (fromEdgePlusLaneChangeLengthEdge.second) {
					EdgeData const & fromEdgePlusLaneChangeLengthEdgeData = edgeDataMap[fromEdgePlusLaneChangeLengthEdge.first];

					if (fromEdgePlusLaneChangeLengthEdgeData.segment == fromEdgeData.segment) {
						toEndParam = getDirectNeighbouredEdge(startParamPlusLaneChangeLengthParam, fromEdgePlusLaneChangeLengthEdge.first, leftTo, rightTo, avoidPosLeft, avoidPosRight);
					}
				}

				// check if we have a neighboring lane to the left
				if (edge_descr() != leftTo) {
					EdgeData const & leftToData = edgeDataMap[leftTo];
					math::flt leftEndParam = toEndParam.first;

					if (!leftToData.isConnection) {
						// fromEdge may become invalid after this call
						if (startParam < fromEdgeData.sourceParam) {
							addLaneChange(e.first, fromEdgeData.sourceParam, leftTo, leftEndParam);
						}
						else {
							addLaneChange(e.first, startParam, leftTo, leftEndParam);
						}

						// So we have to get the value again
						std::pair<edge_descr, bool> ex = inEdge(v);
						assert(ex.second);
						e.first = ex.first;
					}
				}

				// check if we have a neighboring lane to the right
				if (edge_descr() != rightTo) {
					EdgeData const & rightToData = edgeDataMap[rightTo];
					math::flt rightEndParam = toEndParam.second;

					if (!rightToData.isConnection) {
						if (startParam < edgeDataMap[e.first].sourceParam) {
							// fromEdge could have been changed
							addLaneChange(e.first , edgeDataMap[e.first].sourceParam, rightTo, rightEndParam);
						}
						else {
							addLaneChange(e.first , startParam, rightTo, rightEndParam);
						}
					}
				}
			} //if (!outEdge(v).second)
		} // for (unsigned long iv = 0; iv < nvertices; ++iv)
	}

	// DEPRECATED
	// TODO: refactor by moving code from RNDFEditor to RNDFGraph
	void removeEdge(edge_descr const & edge) {
		// delete the edge in the boost graph. Cleanup should be done by the caller.
		remove_edge(edge, boostGraph);
	}

	/**
	  * \param e The edge, where the vertex has to be found
	  * \param[in,out] param The parameter at which the point should be inserted, and after call, was inserted
	  * \return a point on the edge e at the given parameter param within the given tolerance, may create a new one
	  */
	vertex_descr getVertexAtParam(edge_descr const & e, math::flt & param, math::flt tolerance) {
		EdgeData const & edgeData = edgeDataMap[e];
		math::flt const sourceDelta = std::abs(edgeData.sourceParam - param);
		math::flt const targetDelta = std::abs(edgeData.targetParam - param);

		if (sourceDelta < tolerance) {
			//we can use the source waypoint of the edge
			param = edgeData.sourceParam;
			return source(e, boostGraph);
		}
		else if (targetDelta < tolerance) {
			//we can use the target waypoint of the edge
			param = edgeData.targetParam;
			return target(e, boostGraph);
		}
		else {
			//we have to insert a new point on fromEdge
			return insertPoint(e, param, VertexData::INSERTED);
			// src is a newly inserted point at fromParam
		}
	}

	/**
	* \return The newly created lane change edge.
	*/
	edge_descr addLaneChange(edge_descr const & fromEdge, math::flt fromParam, edge_descr const & toEdge, math::flt toParam) {
		static const math::flt mindist = 2.0f;
		// 		boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);
		vertex_descr src = getVertexAtParam(fromEdge, fromParam, mindist);
		vertex_descr dst = getVertexAtParam(toEdge, toParam, mindist);
		// 		DLOG("added lane change from " << fromEdgeData.name << " (" << vertexDataMap[src].name <<") to " << toEdgeData.name << " (" <<vertexDataMap[dst].name << ")");


		// check if an edge like this already exists
		edge_descr newEdge;
		bool edgeExists;
		tie(newEdge, edgeExists) = edge(src, dst, boostGraph);

		if (!edgeExists) {
			bool success;
			tie(newEdge, success) = add_edge(src, dst, boostGraph);
			assert(success);

			VertexData const & v0Data = vertexDataMap[src];
			VertexData const & v1Data = vertexDataMap[dst];

			math::flt const length = (v0Data.pos - v1Data.pos).norm();

			EdgeData & newEdgeData = edgeDataMap[newEdge];
			newEdgeData.name = "lanechange";
			newEdgeData.isConnection = true;
			newEdgeData.isMirrored = true;
			newEdgeData.setParamDir(1.0f);

			// weight a little HIGHER, so ASTAR should prefer earlier lane changes than edge lane change !!!
			//		get(boost::edge_weight_t(), boostGraph)[newEdge] =  10.0f * length;
			get(boost::edge_weight_t(), boostGraph)[newEdge] =  2.0 * std::max(15.0, length);
//			std::cout << length << std::endl;
//			assert(length > 14);
			//		get(boost::edge_weight_t(), boostGraph)[newEdge] =  length;
		}

		return newEdge;
	}


	vertex_descr insertPoint(edge_descr const & edge, VertexData::vertex_type vertexType) {
		// insert a point in the middle of the spline
		//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);
		EdgeData eData = edgeDataMap[edge];
		math::flt insertionParam = eData.sourceParam + (eData.targetParam - eData.sourceParam) / 2;
		return insertPoint(edge, insertionParam, vertexType);
	}

	vertex_descr insertPoint(edge_descr const & edge, math::flt param, VertexData::vertex_type vertexType) {
		// an identifier is needed for program parts that use the name for looking up vertices
		// TODO: refactor with a global ID
		static int identifier = 0;

		// get graph data
		//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);
		EdgeData const & edgeData = edgeDataMap[edge];

		// assertions
		assert(!edgeData.isConnection);
		assert(edgeData.paramDir() * edgeData.sourceParam <= edgeData.paramDir() * param);
		assert(edgeData.paramDir() * param <= edgeData.paramDir() * edgeData.targetParam);

		// compute position for insertion
		math::Vec2 pos = (*edgeData.laneSpline())(param);

		unsigned long const number = num_vertices(boostGraph);
		// add new vertex to the graph
		vertex_descr newVertex	= add_vertex(boostGraph);

		// adjust the vertex data
		VertexData & vertexData = vertexDataMap[newVertex];
		vertexData.pos = pos;
		vertexData.vertexType.set(vertexType);
		vertexData.checkpoint = 0;
		vertexData.l0 = edgeData.segment;
		vertexData.l1 = edgeData.lane;
		vertexData.laneWidth = laneWidth(edge, param);
		put(boost::vertex_index_t(), boostGraph, newVertex, number);

		// prepare for adding/removing the edges
		vertex_descr to = target(edge, boostGraph);
		vertex_descr from = source(edge, boostGraph);
		edge_descr inedge, outedge;

		// add new edges
		bool success;
		tie(inedge, success) = add_edge(from, newVertex, boostGraph);
		assert(success);
		tie(outedge, success) = add_edge(newVertex, to, boostGraph);
		assert(success);

		// copy edge data
		EdgeData & inEdgeData	= edgeDataMap[inedge];
		EdgeData & outEdgeData	= edgeDataMap[outedge];
		inEdgeData = edgeData;
		outEdgeData = edgeData;
		inEdgeData.targetParam = param;
		outEdgeData.sourceParam = param;

		put(boost::edge_weight_t(), boostGraph, inedge, std::abs(inEdgeData.targetParam - inEdgeData.sourceParam));
		put(boost::edge_weight_t(), boostGraph, outedge, std::abs(outEdgeData.targetParam - outEdgeData.sourceParam));

		// check the vertex type
		if (vertexType == VertexData::INSERTED) {
			// vertex ID == 0 indicates, that this is an inserted point
			vertexData.id = 0;

			// adjust the name
			std::stringstream sstr;
			sstr << "inserted." << identifier;

			if (inEdgeData.isMirrored) {
				sstr << ".m";
			}

			vertexData.name = sstr.str();

			identifier++;
		}
		else {
			// call our generic update method
			updateLaneFromGraph(inedge);

		}

		// Do not use edgeData after call to remove_edge()
		// remove old edge
		remove_edge(from, to, boostGraph);

		// fill reverse map
		// TODO: Remove and add the edges selective
		fillReverseMap();

		// new vertex inserted.
		return newVertex;
	}

	void addCheckpoint(vertex_descr v) {
		boost::property_map<aGraph, vertex_data_t>::type vertexDataMap = get(vertex_data_t(), boostGraph);
		VertexData & vertexData = vertexDataMap[v];

		//if the vertex is already a checkpoint, do nothing
		if (vertexData.vertexType.isSet(VertexData::CHECK_POINT)) {
			return;
		}

		//update vertex data
		vertexData.vertexType.set(VertexData::CHECK_POINT);
		vertexData.checkpoint = checkpoints.size() + 1; //give it the next free id
		updateVertexName(v);

		//insert vertex in checkpoints vector
		checkpoints.push_back(v);
	}

	void removeCheckpoint(vertex_descr v) {
		boost::property_map<aGraph, vertex_data_t>::type vertexDataMap = get(vertex_data_t(), boostGraph);
		VertexData & vertexData = vertexDataMap[v];

		//if the vertex is no checkpoint, do nothing
		if (!vertexData.vertexType.isSet(VertexData::CHECK_POINT)) {
			return;
		}

		//remove vertex from checkpoints vector
		checkpoints.erase(checkpoints.begin() + (vertexData.checkpoint - 1));

		//decrement ids of checkpoints with higher id
		for (unsigned int i = (vertexData.checkpoint - 1); i < checkpoints.size(); ++i) {
			VertexData & vd = vertexDataMap[checkpoints[i]];
			--vd.checkpoint;
			updateVertexName(checkpoints[i]);
		}

		//update vertex data
		vertexData.vertexType.unset(VertexData::CHECK_POINT);
		vertexData.checkpoint = 0;
		updateVertexName(v);
	}

	void getLane(std::vector<vertex_descr> & verticesResultVec, std::vector<edge_descr> & edgesResultVec, edge_descr const & edge) const {
		// get graph data and IDs
		vertex_descr start = target(edge, boostGraph);
		vertex_descr v = source(edge, boostGraph);
		//boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), boostGraph);

		// empty the return vectors and push the target node
		verticesResultVec.clear();
		verticesResultVec.push_back(start);
		edgesResultVec.clear();
		edgesResultVec.push_back(edge);

		// traverse the lane in opposite direction and collect all vertices
		bool predecessorFound;

		do {
			// push the source vertex
			predecessorFound = false;
			verticesResultVec.push_back(v);

			// iterate on the inedges
			BOOST_FOREACH(edge_descr const & e, in_edges(v, boostGraph)) {
				// check, that we have a lane edge
				if (!edgeDataMap[e].isConnection) {
					// assert, that there is exactly 1 lane edge
					assert(!predecessorFound);

					// edge is from our lane. Save edge and vertex, and continue traversing.
					v = source(e, boostGraph);
					edgesResultVec.push_back(e);
					// assert(segment.end() == std::find(segment.begin(), segment.end(), src));	// No loop (Expensive, use it with care)
					predecessorFound = true;
				}
			}
		}
		while (predecessorFound);	// if we cannot find a predecessor, we reached the start of the lane.
	}

	void getWholeLane(std::vector<vertex_descr> & verticesResultVec, std::vector<edge_descr> & edgesResultVec, edge_descr const & edge) const {
		// get graph data and IDs
		vertex_descr v = target(edge, boostGraph);
		//boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), boostGraph);

		// init our return vectors with the head of the lane.
		getLane(verticesResultVec, edgesResultVec, edge);
		std::reverse(verticesResultVec.begin(), verticesResultVec.end());
		std::reverse(edgesResultVec.begin(), edgesResultVec.end());

		// traverse the lane until its end
		bool successorFound;

		do {
			successorFound = false;

			// iterate on the out edges
			BOOST_FOREACH(edge_descr const & e, out_edges(v, boostGraph)) {
				// check, that we have a lane edge.
				if (!edgeDataMap[e].isConnection) {
					// assert, that there is exactly 1 lane edge
					assert(!successorFound);

					// Add edge to our vector and continue traversing.
					v = target(e, boostGraph);
					// assert(lane.end() == std::find(lane.begin(), lane.end(), v));	// No loop (Expensive, use it with care)
					verticesResultVec.push_back(v);
					edgesResultVec.push_back(e);
					successorFound = true;
				}
			}
		}
		while (successorFound);

		// if we cannot find a predecessor, we reached the end of the lane.
	}

	void updateLaneFromGraph(edge_descr const & edge) {
		// get graph data maps
		//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);

		// get IDs
		int segmentID = edgeDataMap[edge].segment;
		int laneID = edgeDataMap[edge].lane;

		// get segment and lane
		segment_vec_type & segmentVec = mGraphHierarchy.first[segmentID - 1];
		lane_vec_type & laneEdges = segmentVec[laneID - 1];

		// get all way points of this lane
		std::vector<vertex_descr> laneVertices;
		getWholeLane(laneVertices, laneEdges, edge);

		// idCounter for waypoint IDs
		uint idCounter = 0;

		// iterate on all vertices
		BOOST_FOREACH(vertex_descr const & v, laneVertices) {
			VertexData & vData = vertexDataMap[v];
			vData.l0 = segmentID;
			vData.l1 = laneID;

			// set ID according to the vertex type
			vData.id = (vData.vertexType.isSet(VertexData::INSERTED)) ? 0 : ++idCounter;

			// adjust name
			updateVertexName(v);
		}

		// iterate on all edges
		BOOST_FOREACH(edge_descr const & e, laneEdges) {
			EdgeData & eData = edgeDataMap[e];
			eData.lane = laneID;
			eData.segment = segmentID;
			eData.name = l1string(segmentID, laneID);
		}
	}

	/*
	void updateRNDFBase () {
	    math::Vec2d maxPos(-numeric_limits<math::flt>::max(), -numeric_limits<math::flt>::max());
	    math::Vec2d minPos(numeric_limits<math::flt>::max(), numeric_limits<math::flt>::max());

	    BOOST_FOREACH(vertex_descr const & v, vertices(boostGraph)) {
	        const math::Vec2d pos(v.longitude, v.latitude);
	        for (uint i = 0; i < 2; i++) {
	                maxPos(i) = max(maxPos(i), pos(i));
	                minPos(i) = min(minPos(i), pos(i));
	        }
	    }
	    math::Vec2d globalZero = (maxPos + minPos) * 0.5;
	    theGlobalZero::instance().set2D(globalZero[0], globalZero[1]);
	}
	*/

	static math::Vec2d computeRNDFBase(const aa::modules::models::rndf::io::RNDFData & r) {
		math::Vec2d maxPos(-std::numeric_limits<math::flt>::max(), -std::numeric_limits<math::flt>::max());
		math::Vec2d minPos(std::numeric_limits<math::flt>::max(), std::numeric_limits<math::flt>::max());

		BOOST_FOREACH(Segment const & seg, r.segments) {
			BOOST_FOREACH(Lane const & lane, seg.lanes) {
				BOOST_FOREACH(Waypoint const & w, lane.waypoints) {
					math::Vec2d pos(w.longitude, w.latitude);

					for (uint i = 0; i < 2; i++) {
						maxPos(i) = std::max(maxPos(i), pos(i));
						minPos(i) = std::min(minPos(i), pos(i));
					}
				}
			}
		}

		BOOST_FOREACH(Zone const & zone, r.zones) {
			BOOST_FOREACH(Waypoint const & w, zone.perimeterpoints) {
				math::Vec2d pos(w.longitude, w.latitude);

				for (uint i = 0; i < 2; i++) {
					maxPos(i) = std::max(maxPos(i), pos(i));
					minPos(i) = std::min(minPos(i), pos(i));
				}
			}

			BOOST_FOREACH(Spot const & spot, zone.spots) {
				BOOST_FOREACH(Waypoint const & w, spot.waypoints) {
					math::Vec2 pos(w.longitude, w.latitude);

					for (uint i = 0; i < 2; i++) {
						assert(pos(i) <= maxPos(i));
						assert(pos(i) >= minPos(i));
					}
				}
			}
		}

		return (maxPos + minPos) * 0.5;
	}

	void updateVertexName(vertex_descr v) {
		// get vertex data
		VertexData & vData = vertexDataMap[v];

		// compose the name in a stringstream
		std::stringstream s;

		// TODO: handle inserted vertices

		// compute name by IDs
		s << vData.l0 << '.' << vData.l1 << '.' << vData.id;

		// set the name
		vData.name = s.str();
	}

	bool removeLaneVertex(vertex_descr v) {
		// get data maps
		//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);
		VertexData & vData = vertexDataMap[v];

		// check preconditions
		if (vData.vertexType.isSet(VertexData::CHECK_POINT)
				|| vData.vertexType.isSet(VertexData::IN_ZONE)
				|| mGraphHierarchy.first[vData.l0 - 1][vData.l1 - 1].size() == 1
		   ) {
			return false;
		}

		// we need to find a lane edge for the final update of the lane
		// if a new egde will be created, then we will take this one.
		// otherwise, an incoming of the predecessor or the outgoing of the successor will do it.
		edge_descr edge4update;
		bool inEdgeFound(false);
		// find the incoming lane edge, if any, and delete all incoming connecting edges
		edge_descr incomingLaneEdge;
		BOOST_FOREACH(edge_descr const & e, in_edges(v, boostGraph)) {
			// check, if this is a connecting edge
			if (edgeDataMap[e].isConnection) {
				// connecting edge. remove it.
				removeConnectingEdge(e);
			}
			else {
				// lane edge. There should be only 1.
				assert(!inEdgeFound);

				// now find an edge for update
				BOOST_FOREACH(edge_descr const & ee, in_edges(source(e, boostGraph), boostGraph)) {
					if (!edgeDataMap[ee].isConnection) {
						edge4update = ee;
						break;
					}
				}

				inEdgeFound = true;
				//save incoming lane edge.
				incomingLaneEdge = e;
			}
		}

		// find the outgoing lane edge, if any, and delete all outgoing connecting edges
		bool outEdgeFound(false);
		edge_descr outgoingLaneEdge;
		BOOST_FOREACH(edge_descr const & e, out_edges(v, boostGraph)) {
			// check, if this is a connecting edge
			if (edgeDataMap[e].isConnection) {
				// connecting edge. remove it.
				removeConnectingEdge(e);
			}
			else {
				// lane edge. There should be only 1.
				assert(!outEdgeFound);

				// now find an edge for update
				BOOST_FOREACH(edge_descr const & ee, out_edges(target(e, boostGraph), boostGraph)) {
					if (!edgeDataMap[ee].isConnection) {
						edge4update = ee;
						break;
					}
				}

				outEdgeFound = true;
				//save outgoing lane edge
				outgoingLaneEdge = e;
			}
		}

		// if the edge was inside the lane, we need to create a new edge
		if (inEdgeFound && outEdgeFound) {
			// create and add new edge from predecessor to successor
			edge_descr newEdge;
			bool success;
			tie(newEdge, success) = add_edge(source(incomingLaneEdge, boostGraph), target(outgoingLaneEdge, boostGraph), boostGraph);
			assert(success);

			// copy properties from the incoming lane edge to the new edge
// 			edgeDataMap[newEdge].width = edgeDataMap[incomingLaneEdge.get()].width;
			edgeDataMap[newEdge].boundaryLeft = edgeDataMap[incomingLaneEdge].boundaryLeft;
			edgeDataMap[newEdge].boundaryRight = edgeDataMap[incomingLaneEdge].boundaryRight;
			edgeDataMap[newEdge].segment = edgeDataMap[incomingLaneEdge].segment;
			edgeDataMap[newEdge].lane = edgeDataMap[incomingLaneEdge].lane;

			// save new edge for update
			edge4update = newEdge;
		}

		// delete vertex and edges
		if (inEdgeFound) {
			remove_edge(incomingLaneEdge, boostGraph);
		}

		if (outEdgeFound) {
			remove_edge(outgoingLaneEdge, boostGraph);
		}

		remove_vertex(v, boostGraph);

		// update lane
		updateLaneFromGraph(edge4update);

		return true;
	}

	void removeConnectingEdge(edge_descr const & edge) {
		// check, that edge is a connection
		//boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), const_cast<aGraph const &>(boostGraph));
		assert(edgeDataMap[edge].isConnection);

		// remove the edge in the boost graph
		remove_edge(edge, boostGraph);

		// unset source vertexType EXIT if there are no other outgoing connecting edges
		bool hasExit = false;
		BOOST_FOREACH(edge_descr const & e, out_edges(source(edge, boostGraph), boostGraph)) {
			if (edgeDataMap[e].isConnection) {
				hasExit = true;
			}
		}

		if (!hasExit) {
			vertexDataMap[source(edge, boostGraph)].vertexType.unset(VertexData::EXIT);
		}

		// unset target vertexType ENTRY if there are no other incoming connecting edges
		bool hasEntry = false;
		BOOST_FOREACH(edge_descr const & e, in_edges(target(edge, boostGraph), boostGraph)) {
			if (edgeDataMap[e].isConnection) {
				hasEntry = true;
			}
		}

		if (!hasEntry) {
			vertexDataMap[target(edge, boostGraph)].vertexType.unset(VertexData::ENTRY);
		}
	}

	void lock() {
		mutex.lock();
	}

	void unlock() {
		mutex.unlock();
	}

	zone_vec_type const & getZone(uint zoneID) const {
		return mGraphHierarchy.second[zoneID - mGraphHierarchy.first.size() - 1];
	}

	void mergeSegments(uint segmentID1, uint segmentID2) {
		// iterate on the lanes of the higher segment
		BOOST_FOREACH(lane_vec_type & lane, mGraphHierarchy.first[segmentID2 - 1]) {
			//get new lane id (in the lower segment)
			uint laneID = mGraphHierarchy.first[segmentID1 - 1].size() + 1;

			//insert lane (egdes) in lower segment
			mGraphHierarchy.first[segmentID1 - 1].push_back(lane);

			// get the first edge in the lane and adjust the IDs
			EdgeData & eData = edgeDataMap[lane[0]];
			eData.segment = segmentID1;
			eData.lane = laneID;

			// update the whole lane
			updateLaneFromGraph(lane[0]);
		}

		// delete the empty segment
		mGraphHierarchy.first.erase(mGraphHierarchy.first.begin() + segmentID2 - 1);

		// now adjust the segment IDs
		// iterate on all segments > segmentID2
		for (uint segID = segmentID2; segID < mGraphHierarchy.first.size() + 1; segID++) {
			// iterate on all lanes of the segment
			BOOST_FOREACH(lane_vec_type & lane, mGraphHierarchy.first[segID - 1]) {
				// get the first edge in the lane and adjust the IDs
				EdgeData & eData = edgeDataMap[lane[0]];
				eData.segment = segID;

				// update the whole lane
				updateLaneFromGraph(lane[0]);
			}
		}

		// iterate on the zones
		BOOST_FOREACH(zone_vec_type & zone, mGraphHierarchy.second) {
			// iterate on perimeter points
			BOOST_FOREACH(vertex_descr & v, zone.first) {
				vertexDataMap[v].l0--;
				updateVertexName(v);
			}

			// iterate on spots
			BOOST_FOREACH(edge_descr & e, zone.second) {
				// adjust edge
				edgeDataMap[e].name = l1string(--edgeDataMap[e].segment, edgeDataMap[e].lane);

				// adjust source vertex
				vertexDataMap[source(e, boostGraph)].l0--;
				updateVertexName(source(e, boostGraph));

				// adjust target vertex

				vertexDataMap[target(e, boostGraph)].l0--;
				updateVertexName(target(e, boostGraph));
			}
		}
	} // end of mergeSegments

	uint moveLaneToNewSegment(uint segmentID, uint laneID) {
		// assert, that the segment will not be empty
		assert(mGraphHierarchy.first[segmentID - 1].size() > 1);

		// create new segment
		uint newSegmentID = createNewSegment();

		// copy lane to the new segment, and remove it from the old one
		mGraphHierarchy.first[newSegmentID - 1].push_back(mGraphHierarchy.first[segmentID - 1][laneID - 1]);
		mGraphHierarchy.first[segmentID - 1].erase(mGraphHierarchy.first[segmentID - 1].begin() + laneID - 1);

		//update (decrease) all lane indices bigger than the removed lane
		for (int idx = laneID - 1; idx < mGraphHierarchy.first[segmentID - 1].size(); idx++) {
			lane_vec_type lane = mGraphHierarchy.first[segmentID - 1][idx];

			// get the first edge in the lane and adjust the ID
			EdgeData & eData = edgeDataMap[lane[0]];
			eData.lane--;

			// update the whole lane
			updateLaneFromGraph(lane[0]);
		}

		// update lane indices
		lane_vec_type & lane = mGraphHierarchy.first[newSegmentID - 1][0];

		// adjust first vertex in lane
		vertexDataMap[source(lane[0], boostGraph)].l0 = newSegmentID;
		vertexDataMap[source(lane[0], boostGraph)].l1 = 1;
		updateVertexName(source(lane[0], boostGraph));

		// iterate on all edges
		BOOST_FOREACH(edge_descr const & e, lane) {
			// adjust edge
			edgeDataMap[e].segment = newSegmentID;
			edgeDataMap[e].lane = 1;
			edgeDataMap[e].name = l1string(newSegmentID, 1);

			// adjust target vertex
			vertex_descr const dst(target(e, boostGraph));
			vertexDataMap[dst].l0 = newSegmentID;
			vertexDataMap[dst].l1 = 1;
			updateVertexName(dst);
		}



		// return the new segment ID
		return newSegmentID;
	}

	math::flt laneWidth(edge_descr const & ed, math::flt param) const {
		boost::property_map<aGraph, vertex_data_t>::const_type	vertexDataMap = get(vertex_data_t(), const_cast<aGraph const &>(boostGraph)); // TODO: fix const_cast
		boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), const_cast<aGraph const &>(boostGraph)); // TODO: fix const_cast

		math::flt const
		sourceWidth(vertexDataMap[source(ed, boostGraph)].laneWidth),
					targetWidth(vertexDataMap[target(ed, boostGraph)].laneWidth);

		if (sourceWidth <= 0.0 || targetWidth <= 0.0) {
			return 0.0;
		}

		math::flt const
		sourceParam(edgeDataMap[ed].sourceParam),
					targetParam(edgeDataMap[ed].targetParam);

		return (param - sourceParam) / (targetParam - sourceParam) * (targetWidth - sourceWidth) + sourceWidth;
	}

	void buildGraphHierarchy() {
		//mGraphHierarchy.first.resize(number of segments);
		//mGraphHierarchy.first[segmentid].resize(number of lanes);
		//mGraphHierarchy.first[segmentid][laneid].resize(number of waypoints);
		//mGraphHierarchy.second.resize(number of zones);

		//vertex.l0 segment id OR zone id
		//vertex.l1 lane id

		BOOST_FOREACH(vertex_descr const & v, vertices(boostGraph)) {
			VertexData & vertexData = vertexDataMap[v];

			if (vertexData.vertexType.isSet(VertexData::IN_ZONE)) {
				// vertex is in a zone
				// adjust the mGraphHierarchy to hold this zone
				// note: the zoneid's follow up on the segment id's
				if (vertexData.l0 - mGraphHierarchy.first.size() > mGraphHierarchy.second.size()) {
					zone_vec_type zoneVec;
					mGraphHierarchy.second.resize(vertexData.l0 - mGraphHierarchy.first.size());
					mGraphHierarchy.second[vertexData.l0 - mGraphHierarchy.first.size() - 1] = zoneVec;
				}

				zone_vec_type zoneVec = mGraphHierarchy.second[vertexData.l0 - mGraphHierarchy.first.size() - 1];
				zoneVec.first.push_back(v);
			}
			else {
				// vertex is not in a zone
				// adjust mGraphHierarchy to hold this vertex segment (if needed)
				if (vertexData.l0 > mGraphHierarchy.first.size()) {
					mGraphHierarchy.first.resize(vertexData.l0);
				}

				// adjust mGraphHierarchy to hold this vertex lane (if needed)
				if (vertexData.l1 > mGraphHierarchy.first[vertexData.l0 - 1].size()) {
					mGraphHierarchy.first[vertexData.l0 - 1].resize(vertexData.l1);
				}
			}
		}

		BOOST_FOREACH(edge_descr const & e, edges(boostGraph)) {
			EdgeData & edgeData = edgeDataMap[e];

			if (edgeData.segment > 0 && edgeData.lane > 0) {
				mGraphHierarchy.first[edgeData.segment - 1][edgeData.lane - 1].push_back(e);
			}
		}

	}

	void buildCheckpoints() {
		BOOST_FOREACH(vertex_descr const & v, vertices(boostGraph)) {
			VertexData & vertexData = vertexDataMap[v];

			if (vertexData.checkpoint > 0) {
				if (checkpoints.size() < vertexData.checkpoint) {
					checkpoints.resize(vertexData.checkpoint);
				}

				checkpoints[vertexData.checkpoint - 1] = v;
			}
		}
	}

	void variousGraphFixes() {
		// some fixes
		// playRNDFLogger does not set vertex names correctly
		// sometimes there are lanes with unreasonable lanewidth
		BOOST_FOREACH(vertex_descr const & v, vertices(boostGraph)) {
			updateVertexName(v);
			VertexData  & vertexData(vertexDataMap[v]);

			if (vertexData.laneWidth < 2.1f) {
				vertexData.laneWidth = 2.1f;
				//			std::cout << "minWidth fix " << std::endl;
			}
			else if (vertexData.laneWidth > 30.0f) {
				vertexData.laneWidth = 30.0f;
				//			std::cout << "maxWidth fix " << std::endl;
			}
		}

		// we have segments with zero lanes
		int i = 1;
		int offset = 0;
		BOOST_FOREACH(segment_vec_type const segment, mGraphHierarchy.first) {
			std::cout << "segment: " << i << " num lanes:" << segment.size() << std::endl;

			if (segment.size() == 0) {

				// segment has 0 lanes .. maybe only a vertex?
				std::cout << "Segment " << i << " has no lanes in graphierachy " << std::endl;
				BOOST_FOREACH(vertex_descr const & v, vertices(boostGraph)) {
					VertexData & vertexData(vertexDataMap[v]);

					if (vertexData.l0 == i) {
						std::cout << "has vertex:" << vertexData.name << std::endl;
					}

				}
				offset++;

				/*
					// now adjust the segment IDs
					// move all segments > i to pos i
					for (uint segID = i; segID < p->mGraphHierarchy.first.size(); segID++) {
							// iterate on all lanes of the segment
							cout << "moving  "<< segID+1 << " to " << segID  << std::endl;
							BOOST_FOREACH(lane_vec_type & lane, p->mGraphHierarchy.first[segID]) {
									EdgeData & eData = p->edgeDataMap[lane[0]];
									cout << "was: " << eData.name << std::endl;
									// get the first edge in the lane and adjust the IDs
									eData.segment = segID+1;
									// update the whole lane
									updateLaneFromGraph(lane[0]);
									cout << "to: "<< eData.name << std::endl;
							}
					}

				*/
			}

			for (uint lane_index =  0 ; lane_index < segment.size(); lane_index++) {

				bool laneNotFound = true;

				for (uint edge_index = 0 ; edge_index < segment[lane_index].size() ; edge_index++) {
					laneNotFound = false;
					std::cout << "num edges:" << segment[lane_index].size() << std::endl;
					edge_descr const & theEdgeDescr(segment[lane_index][edge_index]) ;
					EdgeData & edgeData(edgeDataMap[theEdgeDescr]);

					if (offset > 0) {
						std::cout << "moving " << edgeData.name;
					}

					edgeData.segment = i - offset;
					edgeData.name = l1string(i - offset, edgeData.lane);

					std::vector<vertex_descr> laneVertices;
					lane_vec_type laneEdges;
					getWholeLane(laneVertices, laneEdges, theEdgeDescr);

					uint idCounter = 0;

					// iterate on all vertices
					BOOST_FOREACH(vertex_descr const & v, laneVertices) {
						VertexData & vData = vertexDataMap[v];
						vData.l0 = edgeData.segment;
						vData.l1 = edgeData.lane;

						// set ID according to the vertex type
						vData.id = (vData.vertexType.isSet(VertexData::INSERTED)) ? 0 : ++idCounter;

						// adjust name
						updateVertexName(v);
					}

					if (offset > 0) {
						std::cout << " to " << edgeData.name << std::endl;
					}
				}

				std::vector<vertex_descr> toRemove;
				BOOST_FOREACH(vertex_descr const & v, vertices(boostGraph)) {
					VertexData & vertexData(vertexDataMap[v]);

					if (vertexData.l0 == i) {
						std::cout << "has vertex:" << vertexData.name << std::endl;

						if (laneNotFound) {
							toRemove.push_back(v);
						}
					}
				}

				BOOST_FOREACH(vertex_descr const & v, toRemove) {
					BOOST_FOREACH(edge_descr const & e, out_edges(v, boostGraph)) {
						remove_edge(e, boostGraph);
					}
					BOOST_FOREACH(edge_descr const & e, in_edges(v, boostGraph)) {
						remove_edge(e, boostGraph);
					}
					remove_vertex(v, boostGraph);
				}
			}

			i++;
		}

		std::cout << "offset is: " << offset << std::endl;

		mGraphHierarchy.first.clear();
		mGraphHierarchy.second.clear();

		buildGraphHierarchy();

		i = 1;
		BOOST_FOREACH(segment_vec_type segment, mGraphHierarchy.first) {
			std::cout << "segment: " << i << " num lanes:" << segment.size() << std::endl;

			if (segment.size() == 0) {

				// segment has 0 lanes .. maybe only a vertex?
				std::cout << "Segment " << i << " has no lanes in graphierachy " << std::endl;
			}

			i++;
		}


		// fix ENTRY and EXIT tags
		BOOST_FOREACH(vertex_descr const & v, vertices(boostGraph)) {

			VertexData & vertexData(vertexDataMap[v]);

			bool hasExit = false;
			BOOST_FOREACH(edge_descr const & e, out_edges(v, boostGraph)) {
				if (edgeDataMap[e].isConnection) {
					hasExit = true;
				}
			}

			if (!hasExit) {
				vertexData.vertexType.unset(VertexData::EXIT);
			}
			else {
				vertexData.vertexType.set(VertexData::EXIT);
			}

			bool hasEntry = false;
			BOOST_FOREACH(edge_descr const & e, in_edges(v, boostGraph)) {
				if (edgeDataMap[e].isConnection) {
					hasEntry = true;
				}
			}

			if (!hasEntry) {
				vertexData.vertexType.unset(VertexData::ENTRY);
			}
			else {
				vertexData.vertexType.set(VertexData::ENTRY);
			}

		}
	}

};// end of class impl

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// RNDFGraph //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

::aa::modules::models::rndf::RNDFGraph::RNDFGraph()
	: p(new Pimpl())
{}

RNDFGraph::~RNDFGraph()
{
}

//////////////////////////// forwarding methods ////////////////////////////////

void RNDFGraph::lock() const
{
	p->lock();
}

void RNDFGraph::unlock() const
{
	p->unlock();
}

uint RNDFGraph::moveLaneToNewSegment(uint segmentID, uint laneID)
{
	p->lock();
	uint newSegmentID = p->moveLaneToNewSegment(segmentID, laneID);
	p->unlock();
	return newSegmentID;
}

void RNDFGraph::mergeSegments(uint segmentID1, uint segmentID2)
{
	p->lock();
	p->mergeSegments(segmentID1, segmentID2);
	p->unlock();
}

graph_vec_type const & RNDFGraph::getGraphVector() const
{
	return p->mGraphHierarchy;
}

void RNDFGraph::removeEdge(edge_descr const & edge)
{
	p->lock();
	p->removeEdge(edge);
	p->unlock();
}

void RNDFGraph::removeConnectingEdge(edge_descr const & edge)
{
	p->lock();
	p->removeConnectingEdge(edge);
	p->unlock();
}

vertex_descr RNDFGraph::insertPoint(edge_descr const & edge, VertexData::vertex_type vertexType)
{
	p->lock();
	vertex_descr result = p->insertPoint(edge, vertexType);
	p->unlock();
	return result;
}

void RNDFGraph::removeCheckpoint(vertex_descr v)
{
	p->lock();
	p->removeCheckpoint(v);
	p->unlock();
}

void RNDFGraph::addCheckpoint(vertex_descr v)
{
	p->lock();
	p->addCheckpoint(v);
	p->unlock();
}


aGraph const & RNDFGraph::getBoostGraph() const
{
	return p->boostGraph;
}

std::vector<vertex_descr> const & RNDFGraph::checkpoints() const
{
	return p->checkpoints;
}

ZoneInfo const & RNDFGraph::zoneInfo() const
{
	return p->zoneInfo;
}

vertex_descr RNDFGraph::addWaypoint(uint l0, uint l1, uint id, math::Vec2 const & pos)
{
	p->lock();
	vertex_descr result = p->addWaypoint(l0, l1, id, pos);
	p->unlock();
	return result;
}


bool RNDFGraph::removeLaneVertex(vertex_descr v)
{
	p->lock();
	bool retval = p->removeLaneVertex(v);
	p->unlock();
	return retval;
}

math::flt RNDFGraph::laneWidth(edge_descr const & ed, math::flt param) const
{
	return p->laneWidth(ed, param);
}


uint RNDFGraph::numSegments() const
{
	p->lock();
	uint result = p->mGraphHierarchy.first.size();
	p->unlock();
	return result;
}

edge_descr RNDFGraph::addLaneEdge(vertex_descr v0, vertex_descr v1, Lane const & lane)
{
	p->lock();
	edge_descr result = p->addLaneEdge(v0, v1, lane);
	p->unlock();
	return result;
}

edge_descr RNDFGraph::addConnectingEdge(vertex_descr v0, vertex_descr v1)
{
	p->lock();
	edge_descr result = p->addConnectingEdge(v0, v1);
	p->unlock();
	return result;
}

uint RNDFGraph::createNewSegment()
{
	p->lock();
	uint result = p->createNewSegment();
	p->unlock();
	return result;
}

uint RNDFGraph::createNewLane(uint segmentID)
{
	p->lock();
	uint result = p->createNewLane(segmentID);
	p->unlock();
	return result;
}

void RNDFGraph::getLane(std::vector<vertex_descr> & verticesResultVec, std::vector<edge_descr> & edgesResultVec, edge_descr edge) const
{
	p->lock();
	p->getLane(verticesResultVec, edgesResultVec, edge);
	p->unlock();
}

RTT::PropertyBag const & RNDFGraph::getProperties() const
{
	return p->bag;
}

void RNDFGraph::getWholeLane(std::vector<vertex_descr> & verticesResultVec, std::vector<edge_descr> & edgesResultVec, edge_descr edge) const
{
	p->getWholeLane(verticesResultVec, edgesResultVec, edge);
}

bool RNDFGraph::trackDir(math::Vec2 & dir, vertex_descr v) const
{
	return p->trackDir(dir, v);
}

bool RNDFGraph::inDir(math::Vec2 & dir, vertex_descr v) const
{
	return p->inDir(dir, v);
}

bool RNDFGraph::outDir(math::Vec2 & dir, vertex_descr v) const
{
	return p->outDir(dir, v);
}

SplineKdTree const & RNDFGraph::splineKDTree() const
{
	return p->nearestSpline;
}

void RNDFGraph::updateVertexName(vertex_descr v)
{
	p->updateVertexName(v);
}

void RNDFGraph::updateLaneFromGraph(edge_descr const & edge)
{
	p->updateLaneFromGraph(edge);
}

//////////////////////////// functions with body ////////////////////////////////

vertex_descr RNDFGraph::addVertex(math::Vec2 const & pos)
{
	boost::property_map<aGraph, boost::vertex_index_t>::type vertexIndexMap = get(boost::vertex_index_t(), p->boostGraph);

	unsigned int number = num_vertices(p->boostGraph);
	vertex_descr vertex = add_vertex(p->boostGraph);
	VertexData & vertexData = p->vertexDataMap[vertex];
	vertexIndexMap[vertex] = number;
	vertexData.pos = pos;
	return vertex;
}

void RNDFGraph::resplinify()
{
	//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), p->boostGraph);

	p->reverseMap.clear();

	std::vector<edge_descr> toberemoved;
	BOOST_FOREACH(edge_descr const & e, edges(p->boostGraph)) {
		EdgeData & edgeData = p->edgeDataMap[e];
		edgeData.laneSpline().reset();
		edgeData.leftLaneSpline().reset();
		edgeData.rightLaneSpline().reset();

		if (edgeData.isMirrored) {
			toberemoved.push_back(e);
		}
	}

	BOOST_FOREACH(edge_descr const & e, toberemoved) {
		remove_edge(e, p->boostGraph);
	}
	/*
	std::vector<vertex_descr> islands;
	BOOST_FOREACH(vertex_descr v, vertices(p->boostGraph)) {
		if (0 == out_degree(v, p->boostGraph) + in_degree(v, p->boostGraph)) {
			islands.push_back(v);
		}
	}
	*/
	p->splinify();
}

void RNDFGraph::resetGraph()
{
	p->boostGraph.clear();
	p->checkpoints.clear();
// 	p->zoneInfo.computeZones()
	p->reverseMap.clear();
	p->mGraphHierarchy.first.clear();
	p->mGraphHierarchy.second.clear();
	p->vertexMirrorMap.clear();
	p->buildSKDTree();
	signalGraphChanged(*this);
}

void RNDFGraph::buildGraph(boost::shared_ptr<RNDFData> const & pRNDFData, bool optimise)
{
	// if graph not empty then return
	if (num_vertices(p->boostGraph) > 0 || !pRNDFData) {
		return;
	}

	RNDFData & rndfData = *pRNDFData;
	math::Vec2d rndfBase = Pimpl::computeRNDFBase(rndfData);	// Centre in (longitude, latitude)
	math::geodetic::theGlobalZero::instance().set2D(rndfBase(0), rndfBase(1));
//	DLOG("Zero is now: " << theGlobalZero::instance().getLLH() << " (" << WGS84toENU(rndfBase) << ')');

	p->createVertices(rndfData);

	//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), p->boostGraph);

// 	DLOG("Segments: " << r.segments.size());

	BOOST_FOREACH(Segment const & seg, rndfData.segments) {
		uint segmentID = createNewSegment();
		//assert (segmentID == seg.id);

		BOOST_FOREACH(Lane const & lane, seg.lanes) {
			uint laneID = createNewLane(segmentID);
			//assert (laneID == lane.id);

			BOOST_FOREACH(Waypoint const & w0, lane.waypoints) {
				assert(w0.vertex_is_set);
				VertexData & vertexData = p->vertexDataMap[w0.vertex];

				if (lane.waypoints.size() > w0.id) {	// We have a successor
					Waypoint const & w1 = lane.waypoints[w0.id];
					addLaneEdge(w0.vertex, w1.vertex, lane);
				}

				BOOST_FOREACH(WaypointID const & wp, w0.exit_to) {
					DLOG("parsing WP exit " << vertexData.l0 << '.' << vertexData.l1 << '.' << vertexData.id);

					uint const es = wp.segment;
					uint const el = wp.lane;
					uint const ew = wp.waypoint;

					// check indices
					// note: if zone perimeter point, then laneID == 0
					assert(es > 0 && el >= 0 && ew > 0);

					vertex_descr to;

					if (es <= rndfData.segments.size()) {
						// Possibly exit to a segment
						if (rndfData.segments[es - 1].id != es) {	// Should never happen
							RTT::log(RTT::Error) << "Found undefined segment " << es << ", but IDs should be consecutive according to the RNDF-spec" << RTT::endlog();
							continue;
						}

						std::vector<Lane> const & lanes = rndfData.segments[es - 1].lanes;

						if (lanes.size() < el
								|| lanes[el - 1].id != el) {
							RTT::log(RTT::Error) << "Target-Lane " << es << '.' << el << " does not exist, as exit from " << seg.id << '.' <<  lane.id << '.' << w0.id << " stipulates" << RTT::endlog();
							continue;
						}

						std::vector<Waypoint> const & waypoints = lanes[el - 1].waypoints;

						if (waypoints.size() < ew
								|| waypoints[ew - 1].id != ew) {
							RTT::log(RTT::Error) << "Target-Waypoint " << es << '.' << el << '.' << ew << " does not exist, as exit from " << seg.id << '.' <<  lane.id << '.' << w0.id << " stipulates" << RTT::endlog();
							continue;
						}

						to = waypoints[ew - 1].vertex;
					}
					else {
						uint zidx = es - rndfData.segments.size();

						if (zidx > rndfData.zones.size()) {
							RTT::log(RTT::Error) << "Non-existant zone " << es << RTT::endlog();
							continue;
						}

						Zone const & zone = rndfData.zones[zidx - 1];

						if (zone.id != es) {	// Should never happen
							RTT::log(RTT::Error) << "Found undefined zone " << es << ", but IDs should be consecutive according to the RNDF-spec" << RTT::endlog();
							continue;
						}

						if (ew > zone.perimeterpoints.size()
								|| zone.perimeterpoints[ew - 1].id != ew) {
							RTT::log(RTT::Error) << "Target-Waypoint " << es << '.' << el << '.' << ew << " does not exist, as exit from " << seg.id << '.' <<  lane.id << '.' << w0.id << " stipulates" << RTT::endlog();
							continue;
						}

						to = zone.perimeterpoints[ew - 1].vertex;
					}

					// create edge von Waypoint w0 to exit Waypoint;
					p->addConnectingEdge(w0.vertex, to);
				}
			}
		}
	}

	// Add Zone-internal edges to the Graph, by connecting perimeter exits with spots
	BOOST_FOREACH(Zone const & zone, rndfData.zones) {
		// check, that we have perimeter points
		assert(!zone.perimeterpoints.empty());

		zone_vec_type zoneVec;

		for (uint z1 = 0; z1 < zone.perimeterpoints.size(); z1++) {
			Waypoint const & w0 = zone.perimeterpoints[z1];
			bool z1IsEntry = in_degree(w0.vertex, p->boostGraph) > 0; // As we haven't connected the spots yet, it can be
			bool z1IsExit = w0.exit_to.size() > 0;

			if (z1IsEntry) {
				p->vertexDataMap[w0.vertex].vertexType.set(VertexData::ENTRY);
			}

			if (z1IsExit) {
				p->vertexDataMap[w0.vertex].vertexType.set(VertexData::EXIT);
			}

			// add perimeter point to the zone vector
			zoneVec.first.push_back(w0.vertex);

			for (uint z2 = z1 + 1; z2 < zone.perimeterpoints.size(); z2++) {
				Waypoint const & w1 = zone.perimeterpoints[z2];
				bool z2IsEntry = in_degree(w1.vertex, p->boostGraph) > 0; // As we haven't connected the spots yet, it can be
				bool z2IsExit = w1.exit_to.size() > 0;

				if (z2IsEntry) {
					p->vertexDataMap[w1.vertex].vertexType.set(VertexData::ENTRY);
				}

				if (z2IsExit) {
					p->vertexDataMap[w1.vertex].vertexType.set(VertexData::EXIT);
				}

				if (z1IsEntry && !z1IsExit && z2IsExit) {
					p->addZoneEdge(w0.vertex, w1.vertex);
				}

				if (z1IsExit && z2IsEntry && !z2IsExit) {
					p->addZoneEdge(w1.vertex, w0.vertex);
				}
			}
		}

		// add zone in the zones vector
		p->mGraphHierarchy.second.push_back(zoneVec);

		BOOST_FOREACH(Waypoint const & w0, zone.perimeterpoints) {
			//determine if the current perimeter-waypoint is an entry point
			bool pointIsEntry = in_degree(w0.vertex, p->boostGraph) > 0; // As we haven't connected the spots yet, it can be only connections from lanes
			// and/or an exit
			bool pointIsExit = w0.exit_to.size() > 0;

			if (pointIsEntry && !pointIsExit) {	// Then, the point to all entries of spots
				BOOST_FOREACH(Spot const & spot, zone.spots) {
// 					DLOG("Handling Perimeter " << wp0s << "-> Spot: " << spot.id);
					Waypoint const & w1 = spot.waypoints[0];
					p->addZoneEdge(w0.vertex, w1.vertex);
				}
			}

			if (pointIsExit) {	// Then, the point from all exits of spots
				BOOST_FOREACH(Spot const & spot, zone.spots) {
					Waypoint const & w1 = spot.waypoints[1];
					p->addZoneEdge(w1.vertex, w0.vertex);
				}
			}

			BOOST_FOREACH(WaypointID const & wp, w0.exit_to) {
				uint const es = wp.segment;
				uint const el = wp.lane;
				uint const ew = wp.waypoint;
				vertex_descr to;

				if (es - 1 < rndfData.segments.size()) {
					// Possibly exit to a segment
					if (rndfData.segments[es - 1].id != es) {	// Should never happen
						RTT::log(RTT::Error) << "Found undefined segment " << es << ", but IDs should be consecutive according to the RNDF-spec" << RTT::endlog();
						continue;
					}

					std::vector<Lane> const & lanes = rndfData.segments[es - 1].lanes;

					if (lanes.size() < el
							|| lanes[el - 1].id != el) {
						RTT::log(RTT::Error) << "Target-lane " << es << '.' << el << " does not exists, as exit from perimeter-point " << zone.id << ".0." << w0.id << " stipulates"  << RTT::endlog();
						continue;
					}

					std::vector<Waypoint> const & waypoints = lanes[el - 1].waypoints;

					if (waypoints.size() < ew
							|| waypoints[ew - 1].id != ew) {
						RTT::log(RTT::Error) << "Target-waypoint " << es << '.' << el << '.' << ew << " does not exists, as exit from perimeter-point " << zone.id << ".0." << w0.id << " stipulates" << RTT::endlog();
						continue;
					}

					to = waypoints[ew - 1].vertex;

				}
				else {
					uint zidx = es - rndfData.segments.size();

					if (zidx > rndfData.zones.size()) {
						RTT::log(RTT::Error) << "Non-existant zone " << es << RTT::endlog();
						continue;
					}

					Zone const & zone = rndfData.zones[zidx - 1];

					if (zone.id != es) {	// Should never happen
						RTT::log(RTT::Error) << "Found undefined zone " << es << ", but IDs should be consecutive according to the RNDF-spec" << RTT::endlog();
						continue;
					}

					if (ew > zone.perimeterpoints.size()
							|| zone.perimeterpoints[ew - 1].id != ew) {
						RTT::log(RTT::Error) << "Target-Waypoint " << es << '.' << el << '.' << ew << " does not exists, as exit from " << zone.id << ".0." << w0.id << " stipulates" << RTT::endlog();
						continue;
					}

					to = zone.perimeterpoints[ew - 1].vertex;
				}

				// create edge von Waypoint w0 to exit Waypoint;
				p->addConnectingEdge(w0.vertex, to);
			}
		}

		BOOST_FOREACH(Spot const & spot, zone.spots) {
// 			DLOG("Handling Spot " << spot.id << "in zone " << zone.id <<  " (" << sptIt-zone.spots.begin() << ")");
			assert(spot.id != 0);
			Waypoint const & w1 = spot.waypoints[0];
			Waypoint const & w2 = spot.waypoints[1];

			// add spot lane
			p->addSpotLane(w1.vertex, w2.vertex, spot);
		}
	}

	////////////////////////////////////////////////////////////////////////////
	// Add splines, which should fit more naturally the curves of the real roads
	p->splinify();

	p->buildSKDTree();
	p->initConfidences();
	p->fillReverseMap();

#if 0
	DLOG("Constructed RNDF graph with"
		 << "\n\t" << num_vertices(p->boostGraph) << " vertices "
		 << "\n\t " << num_edges(p->boostGraph) << " edges"
		 << "\n\t " << theGlobalZero::instance().getLLH() << " as long lat basepos");
#endif

	// info: the optimization function calls are moved to 'optimiseGraph'
	// 'optimiseGraph' can also be triggered by CTRL-O in the RNDF editor
	if (optimise) {
		optimiseGraph();
	}

	boost::property_map<aGraph, boost::vertex_index_t>::type vertexIndexMap = get(boost::vertex_index_t(), p->boostGraph);
	unsigned int i = 0;
	BOOST_FOREACH(vertex_descr v, vertices(p->boostGraph)) {
		vertexIndexMap[v] = i++;
	}

	signalGraphChanged(*this);
}

struct OrderByQual {
	typedef std::pair<vertex_descr, std::pair<math::flt, edge_descr> > value_type;

	bool operator()(value_type const & a, value_type const & b) const {
		return a.second.first > b.second.first;
	}
};

math::Vec2 RNDFGraph::Pimpl::perimeterDir(VertexData const & v1Data, boost::property_map<aGraph, vertex_data_t>::type const & vertexDataMap)
{
	zone_vec_type zone = getZone(v1Data.l0);
	std::vector<vertex_descr> const & perimeterpoints = zone.first;
	unsigned long const nPerimeterPoints = perimeterpoints.size();
	vertex_descr const succ = perimeterpoints[(v1Data.id - 1 + 1) % nPerimeterPoints];
	vertex_descr const pred = perimeterpoints[(v1Data.id - 1 + nPerimeterPoints - 1) % nPerimeterPoints];
	math::Vec2 dir = vertexDataMap[pred].pos - vertexDataMap[succ].pos;
	math::Vec2 orth(dir(1), -dir(0));
	orth.normalize();
	assert(std::abs(orth.squaredNorm() - 1.f) < 1e-5);
	return orth;
}

void RNDFGraph::Pimpl::splinify()
{
	//boost::property_map<aGraph, edge_data_t>::type edgeDataMap = get(edge_data_t(), boostGraph);
	boost::property_map<aGraph, boost::edge_weight_t>::type edgeWeightMap = get(boost::edge_weight_t(), boostGraph);

	math::LaneSpline::DomVec params;
	math::LaneSpline::ImgVec pos;
	std::vector<vertex_descr> laneVertices;
	std::vector<edge_descr> edgeVertices;
	// Iterate over all segments

	std::vector<edge_descr> connections, zoneConnections, uturns, openEndings, mirrorEdges;

	zoneInfo.computeZones();

	BOOST_FOREACH(edge_descr const & currEdge, edges(boostGraph)) {
		EdgeData & edgeData = edgeDataMap[currEdge];
		vertex_descr src = source(currEdge, boostGraph);
		vertex_descr dst = target(currEdge, boostGraph);
		VertexData const & v1 = vertexDataMap[src];
		VertexData const & v2 = vertexDataMap[dst];

		if (edgeData.isMirrored) {	// We will handle all the mirror edges after all other edges
			mirrorEdges.push_back(currEdge);
			continue;
		}

		if (edgeData.laneSpline()			// We have already a spline, so we are done with the edge
				|| edgeData.inZone)	{	// No splines in unstructured areas/zones
			continue;
		}

		if (edgeData.isConnection) {
			// We postpone handling the connecting-edges after each lane has its spline
			// First check: Is it a U-Turn? Then it is not a real connection and will be handled separately
			// A UTurn is a connection between two lanes in the same segment

			if (edgeData.isUTurn) {
				uturns.push_back(currEdge);
			}
			else if (v1.vertexType.isSet(VertexData::IN_ZONE) || v2.vertexType.isSet(VertexData::IN_ZONE)) {
				zoneConnections.push_back(currEdge);
			}
			else {
				connections.push_back(currEdge);
			}

			continue;
		}

// 		DLOG("New spline for segment " << edgeData.name << " (Edge " << *ei << ")");

		getWholeLane(laneVertices, edgeVertices, currEdge);	// Fetch the whole segment


// 		assert(segment.size() > 1);		// We expect an edge

		if (laneVertices.size() > 2) {	// Ignore all segments
			edge_descr inedge = edge(laneVertices.front(), laneVertices[1], boostGraph).first;
			edge_descr outedge = edge(*(laneVertices.rbegin() + 1), laneVertices.back(), boostGraph).first;

			// unused variables? why?
			vertex_descr first = source(inedge, boostGraph);
			vertex_descr last = target(outedge, boostGraph);

// 			DLOG("Segment: " << vertexDataMap[first].name << " " << inedge << " " <<  outedge << " " << vertexDataMap[last].name << " Edge: " << *ei);
			assert(openEndings.end() == std::find(openEndings.begin(), openEndings.end(), inedge));
			assert(openEndings.end() == std::find(openEndings.begin(), openEndings.end(), outedge));
			openEndings.push_back(inedge);
			openEndings.push_back(outedge);
		}

		edgeData.laneSpline() = boost::shared_ptr<math::LaneSpline>(new math::LaneSpline());
		edgeData.setParamDir(1.0f);
		std::vector<edge_descr> & references = reverseMap[edgeData.laneSpline().get()].first;
		assert(reverseMap[edgeData.laneSpline().get()].second.empty());

		params.push_back(0.0f);
		pos.push_back(vertexDataMap[laneVertices.front()].pos);
		vertex_descr last = laneVertices.front();


		// calculate source and target params
		for (std::vector<vertex_descr>::const_iterator it = laneVertices.begin() + 1; it != laneVertices.end(); ++it) {
			vertex_descr curr = *it;
			bool success;
			edge_descr currentEdge;

			// get edge connecting last and curr
			tie(currentEdge, success) = edge(last, curr, boostGraph);
			assert(success);

			references.push_back(currentEdge);

			EdgeData & currentEdgeData = edgeDataMap[currentEdge];
			VertexData & currentVertexData = vertexDataMap[curr];

			math::Vec2 const & curpos = currentVertexData.pos;
			currentEdgeData.sourceParam = params.back();

			math::Vec2 dir = curpos - pos.back();
			math::flt dist = dir.norm();

			if (dist == 0.0f) {
				DLOG("Edge with 0 length? " << edgeData.name << " " << vertexDataMap[last].name << "(" << last << ")" << "->" << currentVertexData.name << "(" << curr << ")");
			}

			assert(dist > 0.0f);
			currentEdgeData.targetParam = currentEdgeData.sourceParam + dist;

			params.push_back(currentEdgeData.targetParam);

			pos.push_back(curpos);

			currentEdgeData.laneSpline() = edgeData.laneSpline();
			last = curr;
		}

		assert(params.size() > 1);

		// MARK: QUICK FIX: insert middle point
		// Bug seems to lie in math::LaneSpline() with only two points
		// Should tracks consisting of only two points be a line?
		if (params.size() == 2) {
			params.push_back(params[1]);
			params[1] = math::flt(0.5) * (params[0] + params[1]);

			pos.push_back(pos[1]);
			pos[1] = math::flt(0.5) * (pos[0] + pos[1]);
		}

		*edgeData.laneSpline() = math::LaneSpline(params, pos);

		edgeWeightMap[currEdge] = std::abs(params.back() - params.front());
		assert(edgeData.laneSpline().use_count() == ssize_t(laneVertices.size() - 1)); // All edges of the segment should point to the lane
		params.clear();
		pos.clear();
	}

// 	DLOG("Connections: " << connections.size() << " Splines: " << reverseMap.size());

	typedef tr1::unordered_map<vertex_descr, std::pair<math::flt, edge_descr> > best_edge_map_type;

	best_edge_map_type bestEdgeMatch;

	// Next, try to find matching ends of crossings and unify the splines
	BOOST_FOREACH(edge_descr const & ed, connections) {
		// For each connection, find the incoming and outgoing edge
		vertex_descr src = source(ed, boostGraph);	// source of connection
		vertex_descr dst = target(ed, boostGraph);	// destination of connection

		std::pair<edge_descr, bool> inedge = inEdge(src);
		std::pair<edge_descr, bool> outedge = outEdge(dst);

		if (!inedge.second || !outedge.second) { // There should be one
			continue;
		}

		EdgeData const & inEdgeData = edgeDataMap[inedge.first];
		EdgeData const & outEdgeData = edgeDataMap[outedge.first];

		if (inEdgeData.targetParam != inEdgeData.laneSpline()->domain().second		// Either in-
				|| outEdgeData.sourceParam != outEdgeData.laneSpline()->domain().first	// or out-edge is not the end of the lane, so we can't join it
				|| inEdgeData.segment == outEdgeData.segment) {	// Better do not join to edges of the same segment, as it will either
			// lead to the unification of two different lanes or, even worse, a loop

			continue;
		}

// 		DLOG("Checking from " << inEdgeData.name << " " << inedge << " to " << outEdgeData.name << " "  << outedge);

		boost::shared_ptr<math::LaneSpline const> inSpline(inEdgeData.laneSpline());
		boost::shared_ptr<math::LaneSpline const> outSpline(outEdgeData.laneSpline());

		assert(inSpline);
		assert(outSpline);

		math::Vec2 indir(vertexDataMap[target(inedge.first, boostGraph)].pos - vertexDataMap[source(inedge.first, boostGraph)].pos);
		math::Vec2 outdir(vertexDataMap[target(outedge.first, boostGraph)].pos - vertexDataMap[source(outedge.first, boostGraph)].pos);

//   math::Vec2
// 			indir((*inSpline).firstDerivative(inEdgeData.targetParam)),
// 			outdir((*outSpline).firstDerivative(outEdgeData.sourceParam));

		math::flt parallel = dot_product(indir, outdir) / sqrt(indir.squaredNorm() * outdir.squaredNorm());	// May be have to use the real angle
		// Order by the outgoing edge
// 		DLOG(" Quality: " << parallel);

		bool entryFound = (bestEdgeMatch.find(src) != bestEdgeMatch.end());
		best_edge_map_type::mapped_type & el = bestEdgeMatch[src];

		if (!entryFound || parallel > el.first) {
// 			DLOG(" Storing for " << src << " from " << inEdgeData.name << " to " << outEdgeData.name << " " << dst);
			el = std::make_pair(parallel, ed);
		}
	}

// 	DLOG("Connections in total:  " << connections.size());

	// Now join the best matches in the order of their quality
	typedef std::vector<std::pair<vertex_descr, std::pair<math::flt, edge_descr> > >  best_edge_vector_type;
	best_edge_vector_type orderedMatches(bestEdgeMatch.begin(), bestEdgeMatch.end());
	std::sort(orderedMatches.begin(), orderedMatches.end(), OrderByQual());

	BOOST_FOREACH(best_edge_vector_type::const_reference best, orderedMatches) {
		edge_descr connectingEdge = best.second.second;
		vertex_descr src = best.first;
		vertex_descr dst = target(connectingEdge, boostGraph);

		std::pair<edge_descr, bool> inedge = inEdge(src);
		std::pair<edge_descr, bool> outedge = outEdge(dst);

		if (!inedge.second || !outedge.second) { // There should be one
			continue;
		}

		boost::shared_ptr<math::LaneSpline> combinedSpline(edgeDataMap[inedge.first].laneSpline());
		boost::shared_ptr<math::LaneSpline> oldSpline(edgeDataMap[outedge.first].laneSpline());

		assert(oldSpline);
		assert(combinedSpline);


		if (combinedSpline == oldSpline) {	// We have a loop
			continue;
		}

		if (edgeDataMap[inedge.first].targetParam != combinedSpline->domain().second	// Not the end of the spline
				|| edgeDataMap[outedge.first].sourceParam != 0			// Not the beginning of the spline
		   ) {
			continue;
		}


		connections.erase(std::find(connections.begin(), connections.end(), connectingEdge));

		openEndings.erase(std::find(openEndings.begin(), openEndings.end(), inedge.first));
		openEndings.erase(std::find(openEndings.begin(), openEndings.end(), outedge.first));

		math::flt const distance = combinedSpline->domain().second + (vertexDataMap[src].pos - vertexDataMap[dst].pos).norm();

		assert(oldSpline->domain().first == 0.0f);

		oldSpline->translateDomain(distance);

		edgeDataMap[connectingEdge].laneSpline() = combinedSpline;
		edgeDataMap[connectingEdge].sourceParam = combinedSpline->domain().second;
		edgeDataMap[connectingEdge].targetParam = distance;

		std::vector<edge_descr>	& oldreferences = reverseMap[oldSpline.get()].first;
		std::vector<edge_descr>	& newrefences = reverseMap[combinedSpline.get()].first;

		assert(reverseMap[oldSpline.get()].second.empty() && reverseMap[combinedSpline.get()].second.empty()); // No mirror edges

		newrefences.push_back(connectingEdge);

		// Rebase the old spline-references
		BOOST_FOREACH(edge_descr const & ed , oldreferences) {
			edgeDataMap[ed].sourceParam += distance;
			edgeDataMap[ed].targetParam += distance;
			edgeDataMap[ed].laneSpline() = combinedSpline;
			newrefences.push_back(ed);
		}

		reverseMap.erase(oldSpline.get());
		assert(oldSpline.unique()); // All references except the one in the map and a local one should be removed

		combinedSpline->append(*oldSpline);
		assert(combinedSpline->domain().second == oldSpline->domain().second);

// 		DLOG("Merged " << edgeDataMap[inedge].name << " " << inedge << " with " << edgeDataMap[outedge].name << " "  << outedge << " "  << vertexDataMap[src].name << "->" << vertexDataMap[dst].name);
	}

// 	DLOG("Merging done");

// 	DLOG("Remaining splines: " << reverseMap.size());

#if defined(CIRCULAR_CURVES)
	// Make all curves circular

	static math::flt const minSquaredRadius = 215.f;
	BOOST_FOREACH(edge_descr const & currEdge, edges(boostGraph)) {
		EdgeData & edgeData = edgeDataMap[currEdge];
		vertex_descr
		src = source(currEdge, boostGraph),
		dst = target(currEdge, boostGraph);

		VertexData
		& v1Data = vertexDataMap[src],
		  & v2Data = vertexDataMap[dst];

		if (edgeData.isMirrored) {	// We skip all the mirror edges
			continue;
		}

		if ((1 == in_degree(src, boostGraph))
				&& (1 == out_degree(dst, boostGraph))) {
			edge_descr const inedge = *in_edges(src, boostGraph).first;
			edge_descr const outedge = *out_edges(dst, boostGraph).first;
			EdgeData const & inEdgeData = edgeDataMap[inedge];
			EdgeData const & outEdgeData = edgeDataMap[outedge];

			if (edgeData.laneSpline() && inEdgeData.laneSpline() && outEdgeData.laneSpline()) {
				math::flt const inlen =	inEdgeData.targetParam - inEdgeData.sourceParam;
				math::flt const instep =	std::min(math::flt(0.1) * inlen, math::flt(0.1));
				math::flt const outlen =	outEdgeData.targetParam - outEdgeData.sourceParam;
				math::flt const outstep =	std::min(math::flt(0.1) * outlen, math::flt(0.1));
				math::Vec2 const inpos = v1Data.pos;
				math::Vec2 const outpos = v2Data.pos;
				math::Vec2 indir = inEdgeData.laneSpline()->firstDerivative(inEdgeData.targetParam - instep);
				math::Vec2 outdir = outEdgeData.laneSpline()->firstDerivative(outEdgeData.sourceParam + outstep);
				math::flt const parallel = dot_product(indir, outdir) / sqrt(indir.squaredNorm() * outdir.squaredNorm());

				if (abs(parallel) < math::flt(0.2)) {	// Almost 90° angles
					// We have a curve here, extend the spline to be more circular

					math::flt const len =	edgeData.targetParam - edgeData.sourceParam;
					math::flt const step =	std::min(math::flt(0.1) * len, math::flt(0.1));
					math::flt const r0 =	::math::squaredRadius(*edgeData.laneSpline(), edgeData.sourceParam + step);
					math::flt const r1 =	::math::squaredRadius(*edgeData.laneSpline(), edgeData.targetParam - step);

					if (std::max(r0, r1) > minSquaredRadius) {
						continue;
					}

					indir.normalize();
					outdir.normalize();
					math::flt const diag = (v1Data.pos - v2Data.pos).norm();
					edgeData.laneSpline()->insertPoint(math::flt(0.5) * (edgeData.sourceParam + edgeData.targetParam),
													   math::flt(0.5) * (inpos + outpos) + math::flt(M_SQRT1_2 - 0.5) * diag * (indir - outdir));
				}
			}
		}
	}

#endif

#if defined(CIRCULAR_CONNECTION_HEURISTIC)
	math::LaneSpline::DomVec tx(5);
	math::LaneSpline::ImgVec px(5);

	// Now connect the curves at the crossings
// 	DLOG("Remaining connections: " << connections.size());
	connections.insert(connections.end(), zoneConnections.begin(), zoneConnections.end());
	zoneConnections.clear();

	BOOST_FOREACH(edge_descr const & connectingEdge, connections) {
#if !defined(TEST)
#else
		tx.resize(4);
		px.resize(4);
#endif
		EdgeData & connectingEdgeData = edgeDataMap[connectingEdge];

		vertex_descr src = source(connectingEdge, boostGraph);	// source of connection
		vertex_descr dst = target(connectingEdge, boostGraph);	// destination of connection

		VertexData const
		& v1Data = vertexDataMap[src],
		  & v2Data = vertexDataMap[dst];

		math::Vec2
		indir,
		outdir;

		if (v1Data.vertexType.isSet(VertexData::IN_ZONE)) {
			indir = perimeterDir(v1Data, vertexDataMap);
		}
		else if (!inDir(indir, src) && !outDir(indir, src)) {
			continue;
		}

		if (v2Data.vertexType.isSet(VertexData::IN_ZONE)) {
			outdir = -perimeterDir(v2Data, vertexDataMap);
		}
		else if (!outDir(outdir, dst) && !inDir(outdir, dst)) {
			continue;
		}

		assert(indir.squaredNorm() > 0.0);
		assert(outdir.squaredNorm() > 0.0);

#if !defined(TEST)
		px.front() = vertexDataMap[src].pos,
		px.back() = vertexDataMap[dst].pos;

		math::flt const dist = (px.back() - px.front()).norm();
		assert(dist > 0.0);
		tx.front() = 0.0f;

		tx[1] = sourceExtend * dist;
		tx[2] = 0.5f * dist;
		tx[3] = (1.0f - targetExtend) * dist;
		tx.back() = dist;
		px[1] = px.front() + tx[1] * indir + math::flt(sourceLeft) *  outdir;
		px[3] = px.back() - math::flt(targetExtend) * outdir + math::flt(targetLeft) * indir;

		math::flt const secondDist = (px[1] - px[3]).norm();
		assert(secondDist > 0.0);

		assert(is_sorted(tx.begin(), tx.end()));
		indir.normalize();
		outdir.normalize();
		static math::flt const diagToDiam = math::flt(M_SQRT1_2 - 0.5) ;
		math::Vec2 apexPoint = 0.5f * (px[1] + px[3]) + (math::flt(apex) * diagToDiam * secondDist) * (indir -  outdir) + math::flt(apexTop) * indir;
		px[2] = apexPoint;
#else
		px[0] = vertexDataMap[src].pos - indir;
		px[1] = vertexDataMap[src].pos;
		px[2] = vertexDataMap[dst].pos;
		px[3] = vertexDataMap[dst].pos + outdir;

		connectingEdgeData.sourceParam	= -indir.norm();
#endif

		for (unsigned int i = 1; i < tx.size(); ++i) {
			tx[i] = sqrt(math::ssd(px[i - 1], px[i])) + tx[i - 1];
			assert(tx[i] > tx[i - 1]);
		}

#if !defined(TEST)
		connectingEdgeData.targetParam	= tx.back();
#else
		connectingEdgeData.targetParam	= tx[2];
#endif
		boost::shared_ptr<math::LaneSpline> ls(new math::LaneSpline(tx, px));
		connectingEdgeData.laneSpline() = ls;
		reverseMap[ls.get()].first.push_back(connectingEdge);
	}
#else
	math::LaneSpline::DomVec tx(6);
	math::LaneSpline::ImgVec px(6);

	// Now connect the curves at the crossings
// 	DLOG("Remaining connections: " << connections.size());
	connections.insert(connections.end(), zoneConnections.begin(), zoneConnections.end());
	zoneConnections.clear();

	BOOST_FOREACH(edge_descr const & connectingEdge, connections) {
		EdgeData & connectingEdgeData = edgeDataMap[connectingEdge];

		vertex_descr src = source(connectingEdge, boostGraph);	// source of connection
		vertex_descr dst = target(connectingEdge, boostGraph);	// destination of connection

		VertexData const
		& v1Data = vertexDataMap[src],
		  & v2Data = vertexDataMap[dst];

		math::Vec2
		indir,
		outdir;

		if (v1Data.vertexType.isSet(VertexData::IN_ZONE)) {
			indir = perimeterDir(v1Data, vertexDataMap);
		}
		else if (!inDir(indir, src) && !outDir(indir, src)) {
			continue;
		}

		if (v2Data.vertexType.isSet(VertexData::IN_ZONE)) {
			outdir = -perimeterDir(v2Data, vertexDataMap);
		}
		else if (!outDir(outdir, dst) && !inDir(outdir, dst)) {
			continue;
		}

		assert(indir.squaredNorm() > 0.0);
		assert(outdir.squaredNorm() > 0.0);

		math::flt const dist = math::ssd(vertexDataMap[src].pos, vertexDataMap[dst].pos);
		math::flt const indirLen = indir.norm();
		math::flt const outdirLen = outdir.norm();
		static math::flt const scalelen = 1e-3f;

		px[0] = vertexDataMap[src].pos - 2.0f * scalelen / indirLen * indir;
		px[1] = vertexDataMap[src].pos - 1.0f * scalelen / indirLen * indir;
		px[2] = vertexDataMap[src].pos;
		px[3] = vertexDataMap[dst].pos;
		px[4] = vertexDataMap[dst].pos + 1.0f * scalelen / outdirLen * outdir;
		px[5] = vertexDataMap[dst].pos + 2.0f * scalelen / outdirLen * outdir;



		tx.front() = -2.0f * scalelen * dist / indirLen ;

		for (unsigned int i = 1; i < tx.size(); ++i) {
			tx[i] = sqrt(math::ssd(px[i - 1], px[i])) + tx[i - 1];
			assert(tx[i] > tx[i - 1]);
		}

		connectingEdgeData.sourceParam	= tx[2];
		connectingEdgeData.targetParam	= tx[3];

		boost::shared_ptr<math::LaneSpline> ls(new math::LaneSpline(tx, px));
		connectingEdgeData.laneSpline() = ls;
		reverseMap[ls.get()].first.push_back(connectingEdge);
	}
#endif

#if defined(SPLINES_FOR_UTURNS)
	// Create splines for U-Turns
	tx.resize(4);
	px.resize(4);

	BOOST_FOREACH(edge_descr const & currEdge, uturns) {
		EdgeData & edgeData = edgeDataMap[currEdge];

		assert(!edgeData.laneSpline() && !edgeData.isMirrored);

		if (edgeData.laneSpline() || edgeData.isMirrored) {
			continue;
		}

		VertexData const & v1 = vertexDataMap[source(currEdge, boostGraph)];
		VertexData const & v2 = vertexDataMap[target(currEdge, boostGraph)];
		math::flt dist = sqrt(math::ssd(v1.pos, v2.pos));
		math::Vec2 dir = v2.pos - v1.pos;
		tx[0] = -dist;
		tx[1] = 0.0f;
		tx[2] = dist;
		tx[3] = 2.0f * dist;
		px[0] = v1.pos - dir;
		px[1] = v1.pos;
		px[2] = v2.pos;
		px[3] = v2.pos + dir;
		edgeData.isUTurn = true;
		boost::shared_ptr<math::LaneSpline> ls(new math::LaneSpline(tx, px));
		edgeData.laneSpline() = ls;
		reverseMap[ls.get()].first.push_back(currEdge);
		edgeData.sourceParam = tx[1];
		edgeData.targetParam = tx[2];
	}
#endif

	// Iteratively adopt the parametric length of the spline to match
	// the integral length of it. 4-5 iterations seemed to work fine, 6 to be sure
	for (uint i = 0; i < 6; ++i) {
		rectify();
	}

	neighbourSplines();

	sortReferencesBySource();
}


bool RNDFGraph::Pimpl::findSandwitchingEdges(std::vector<math::flt> const & distances, uint & fromEdge, uint & toEdge) const
{
	bool minusPlus[2];
	minusPlus[0] = minusPlus[1] = false;
	math::flt maxLowerZero = -100000.f;
	math::flt minGreaterOne = 100000.f;

	for (uint i = 0; i < distances.size(); i++) {
		if (distances[i] < 0.f && distances[i] > maxLowerZero) {
			minusPlus[0] = true;
			maxLowerZero = distances[i];
			fromEdge = i;
		}
		else if (distances[i] < minGreaterOne) {
			minusPlus[1] = true;
			minGreaterOne = distances[i];
			toEdge = i;
		}
	}

	return (minusPlus[0] && minusPlus[1]);
}

bool RNDFGraph::getDirectNeighbouredEdge(math::flt param, edge_descr const & edge, bool left,
		edge_descr & neighbourEdge, math::Vec2 & neighbourPos,
		math::flt & closestParm, math::flt & closestSquaredDist) const
{
	return p->getDirectNeighbouredEdge(param, edge, left, neighbourEdge, neighbourPos, closestParm, closestSquaredDist);
}

std::pair<math::flt, math::flt> RNDFGraph::getDirectNeighbouredEdge(math::Vec2 const & pos, edge_descr const & edge, edge_descr & left, edge_descr & right, math::Vec2 & avoidPosLeft, math::Vec2 & avoidPosRight) const
{
	//aGraph const & rGraph = getBoostGraph();

	//boost::property_map<aGraph, edge_data_t>::const_type	edgeDataMap = get(edge_data_t(), rGraph);
	//boost::property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);

	EdgeData const & edgeData = p->edgeDataMap[edge];
	math::flt closestSquaredDist, closestParm;
	math::flt const from = std::min(edgeData.sourceParam, edgeData.targetParam);
	math::flt const to   = std::max(edgeData.sourceParam, edgeData.targetParam);
	boost::tie(closestSquaredDist, closestParm) = ::math::findClosestPoint(*edgeData.laneSpline(), from, to, 0.5f * (from + to), pos, 0.1f);

	return getDirectNeighbouredEdge(closestParm, edge, left, right, avoidPosLeft, avoidPosRight);
}

bool RNDFGraph::Pimpl::getDirectNeighbouredEdge(math::flt param, edge_descr const & edge, bool left,
		edge_descr & neighbourEdge, math::Vec2 & neighbourPos,
		math::flt & closestParm, math::flt & closestSquaredDist) const
{
	//aGraph const & rGraph = boostGraph;

	//boost::property_map<aGraph, edge_data_t>::const_type	edgeDataMap = get(edge_data_t(), rGraph);
	//boost::property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);

	EdgeData const & edgeData = edgeDataMap[edge];

	bool retval = false;

	if (edgeData.isConnection
// 				|| edgeData.isMirrored
			|| edgeData.inZone
			|| edgeData.isUTurn) {
		return false;
	}

	std::pair<math::Vec2, math::Vec2> posAndDir;
	edgeData.laneSpline()->valueAndFirstDerivative(posAndDir, param);
	math::Vec2 pos = posAndDir.first;
	math::Vec2 ownDir = edgeData.paramDir() * posAndDir.second;

	if (left && edgeData.leftLaneSpline()) {
		math::LaneSpline const & laneSpline = *edgeData.leftLaneSpline();

		math::flt const from	= edgeData.leftLaneParamRange.first;
		math::flt const to		= edgeData.leftLaneParamRange.second;

		boost::tie(closestSquaredDist, closestParm) = ::math::findClosestPoint(laneSpline, from, to, 0.5f * (from + to), pos, 0.1f);
		neighbourPos = laneSpline(closestParm);

		if (math::ssd(neighbourPos, pos) > 100) {
			return false;
		}

		math::Vec2 const otherDir = laneSpline.firstDerivative(closestParm);
		int const antiparallel = std::signbit(dot_product(ownDir, otherDir));	// Nonzero, if antiparallel (dot_product < 0)

		if (antiparallel) {
			boost::tie(neighbourEdge, retval) = findMirroredEdge(closestParm, edgeData.leftLaneSpline());
		}
		else {
			boost::tie(neighbourEdge, retval) = findNonMirroredEdge(closestParm, edgeData.leftLaneSpline());
		}
	}

	if (!left && edgeData.rightLaneSpline()) {
		math::LaneSpline const & laneSpline = *edgeData.rightLaneSpline();

		math::flt const from	= edgeData.rightLaneParamRange.first;
		math::flt const to	= edgeData.rightLaneParamRange.second;

		boost::tie(closestSquaredDist, closestParm) = ::math::findClosestPoint(laneSpline, from, to, 0.5f * (from + to), pos, 0.1f);

		neighbourPos = laneSpline(closestParm);

		if (math::ssd(neighbourPos, pos) > 100) {
			return false;
		}

		math::Vec2 const otherDir = laneSpline.firstDerivative(closestParm);
		int const antiparallel = std::signbit(dot_product(ownDir, otherDir));
		// Nonzero, if antiparallel (dot_product < 0)

		if (antiparallel) {
			boost::tie(neighbourEdge, retval) = findMirroredEdge(closestParm, edgeData.rightLaneSpline());
		}
		else {
			boost::tie(neighbourEdge, retval) = findNonMirroredEdge(closestParm, edgeData.rightLaneSpline());
		}
	}

	return retval;
}

std::pair<math::flt, math::flt> RNDFGraph::getDirectNeighbouredEdge(math::flt param, edge_descr const & edge, edge_descr & rLeft, edge_descr & rRight, math::Vec2 & rAvoidPosLeft, math::Vec2 & rAvoidPosRight) const
{
	return p->getDirectNeighbouredEdge(param, edge, rLeft, rRight, rAvoidPosLeft, rAvoidPosRight);
}


std::pair <math::flt, math::flt> RNDFGraph::Pimpl::getDirectNeighbouredEdge(math::flt param, edge_descr const & edge, edge_descr & rLeft, edge_descr & rRight, math::Vec2 & rAvoidPosLeft, math::Vec2 & rAvoidPosRight) const
{
	std::pair<math::flt, math::flt> retval;
	math::flt closestSquaredDist;

	bool foundLeft = getDirectNeighbouredEdge(param, edge, true, rLeft, rAvoidPosLeft, retval.first, closestSquaredDist);

	if (!foundLeft) {
		retval.first = -std::numeric_limits<math::flt>::max();
	}

	bool foundRight = getDirectNeighbouredEdge(param, edge, false, rRight, rAvoidPosRight, retval.second, closestSquaredDist);

	if (!foundRight) {
		retval.second = -std::numeric_limits<math::flt>::max();
	}


// 	assert(foundLeft || foundRight);
	/* if (!(retval.first || retval.second)) {
		RTT::log(RTT::Debug) << "getDirectNeighbouredEdge: From " << vertexDataMap[source(edge, p->boostGraph)].name
					<< " to " << vertexDataMap[target(edge, p->boostGraph)].name << " Neighbours: " << edgeData.leftLaneSpline() <<  ", " << edgeData.rightLaneSpline() <<  RTT::endlog();
	}*/

	return retval;
}

vertex_descr RNDFGraph::getMirroredVertex(vertex_descr v) const
{
	vertex_mirror_map_type::const_iterator pos = p->vertexMirrorMap.find(v);

	if (p->vertexMirrorMap.end() != pos) {
		return pos->second;
	}

	return GraphTraits::null_vertex();
}

edge_descr RNDFGraph::getMirroredEdge(edge_descr const & ed) const
{
	aGraph const & rGraph = p->boostGraph;

	//boost::property_map<aGraph, edge_data_t>::const_type	edgeDataMap = get(edge_data_t(), rGraph);
	//boost::property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);

	vertex_descr a = source(ed, rGraph);
	vertex_descr b = target(ed, rGraph);

// 	DLOG("getMirroredEdge:");
// 	DLOG("a:" << vertexDataMap[a].name);
// 	DLOG("b:" << vertexDataMap[b].name);

	vertex_descr a_mirror = getMirroredVertex(a);
	vertex_descr b_mirror = getMirroredVertex(b);
// 	if (a_mirror)
// 		DLOG("a_mirror:" << vertexDataMap[a_mirror.get()].name);
// 	if (b_mirror)
// 		DLOG("b_mirror:" << vertexDataMap[b_mirror.get()].name);

	edge_descr ed_mirror;
	bool exists = false;

	if (a_mirror == GraphTraits::null_vertex() ||  b_mirror == GraphTraits::null_vertex()) {
		boost::tie(ed_mirror, exists) = edge(b_mirror, a_mirror, p->boostGraph);
	}

	if (exists) {
		return ed_mirror;
	}
	else {
		RTT::log(RTT::Error) << "SHIT!!!!!!!!!!!!!!!!" << RTT::endlog();
		return edge_descr();
	}
}

vertex_descr RNDFGraph::getVertex(std::string const & name) const
{
	//boost::property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), const_cast<aGraph const &>(p->boostGraph)); // TODO: fix const_cast

	BOOST_FOREACH(vertex_descr v, vertices(p->boostGraph)) {
		if (p->vertexDataMap[v].name == name) {
			return v;
		}
	}

	return GraphTraits::null_vertex();
}


vertex_descr RNDFGraph::getVertex(uint l0, uint l1, uint l2) const
{
	// get graph data
//	boost::property_map<aGraph, vertex_data_t>::const_type	vertexDataMap = get(vertex_data_t(), const_cast<aGraph const &>(p->boostGraph));
//	boost::property_map<aGraph, edge_data_t>::const_type	edgeDataMap = get(edge_data_t(), const_cast<aGraph const &>(p->boostGraph));
	// lane waypoint
	if (l0 - 1 < p->mGraphHierarchy.first.size()) {
		// find the correct lane in the correct segment
		segment_vec_type const & segment = p->mGraphHierarchy.first[l0 - 1];

		if (l1 - 1 >= segment.size()) {
			return GraphTraits::null_vertex();
		}

		lane_vec_type const & lane = segment[l1 - 1];

		// check the start node
		vertex_descr const v = source(lane[0], p->boostGraph);

		if (l2 == p->vertexDataMap[v].id) {
			return v;
		}

		// check all other nodes
		BOOST_FOREACH(edge_descr const & e, lane) {
			vertex_descr targetv(target(e, p->boostGraph));

			if (l2 == p->vertexDataMap[targetv].id) {
				return targetv;
			}
		}
		// node not found.
		return GraphTraits::null_vertex();
	}

	// check, that l1 not out of range
	uint zoneID = l0 - p->mGraphHierarchy.first.size();
	assert(zoneID <= p->mGraphHierarchy.second.size());

	// we are in a zone
	zone_vec_type & zoneVec = p->mGraphHierarchy.second[zoneID - 1];

	// perimeter point
	if (l1 == 0) {
		return zoneVec.first[l2 - 1];
	}

	// spot
	assert(l1 <= zoneVec.second.size());
	assert(l2 == 1 || l2 == 2);
	edge_descr spotLane = zoneVec.second[l1 - 1];
	return l2 == 1 ? source(spotLane, p->boostGraph) : target(spotLane, p->boostGraph);
}

struct CheckDirOrMirrored {
	explicit CheckDirOrMirrored(math::Vec2 const & _dir, reverse_map_type const & _revMap)
		: dir(_dir)
		, revMap(_revMap)
	{}

	bool operator()(::math::LaneSpline const * spline, math::flt param, math::flt dist) const {
		if (dot_product(spline->firstDerivative(param), dir) > 0.0) {
			return true;
		}

		reverse_map_type::const_iterator it = revMap.find(spline);
		// We know about that spline, and there is a mirrored edge (assuming that the mirrored edge spans the full spline
		return (revMap.end() != it && !it->second.second.empty());
	}

	math::Vec2 dir;
	reverse_map_type const & revMap;
};

boost::tuple<edge_descr, math::flt, math::flt> RNDFGraph::getClosestSameDirectedEdge(math::Vec2 const & pos, math::Vec2 const & carDir) const
{
	//aGraph const & rGraph = p->boostGraph;

	// boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rGraph);

	SplineKdTree::spline_hit_vector hits;
	math::flt dist = p->nearestSpline.findClosestQualified(hits, pos, CheckDirOrMirrored(carDir, p->reverseMap));
	math::flt dist2 = dist * dist;

	BOOST_FOREACH(SplineKdTree::spline_hit_vector::const_reference hit, hits) {
		math::LaneSpline const * pSpline = hit.first;
		math::flt const closestParm = hit.second;

		math::Vec2 const splineDir = pSpline->firstDerivative(closestParm);
		std::vector<edge_descr>::const_iterator pos;

		if (dot_product(splineDir, carDir) > 0) {
			std::vector<edge_descr> const & ref = p->reverseMap[pSpline].first;	// Non-mirrored ones
			pos = std::lower_bound(ref.begin() + 1, ref.end(), closestParm, CompareToSource(p->edgeDataMap));
		}
		else {
			std::vector<edge_descr> const & ref = p->reverseMap[pSpline].second;	// Mirrored ones

			if (ref.empty()) {
				return make_tuple(edge_descr(), -std::numeric_limits<math::flt>::max(), -std::numeric_limits<math::flt>::max());
			}

			pos = std::lower_bound(ref.begin() + 1, ref.end(), closestParm, CompareToTarget(p->edgeDataMap));
		}

		return make_tuple(*(pos - 1), closestParm, dist);
	}

	return make_tuple(edge_descr(), -std::numeric_limits<math::flt>::max(), -std::numeric_limits<math::flt>::max());
}

struct CheckDir {
	explicit CheckDir(math::Vec2 const & _dir)
		: dir(_dir)
	{}

	bool operator()(::math::LaneSpline const * spline, math::flt param, math::flt dist) const {
		return dot_product(spline->firstDerivative(param), dir) > 0.0;
	}
	math::Vec2 dir;
};

boost::tuple<edge_descr, math::flt, math::flt> RNDFGraph::closestEdgeSameDirNonMirrored(math::Vec2 const & pos, math::Vec2 const & carDir) const
{
	//aGraph const & rGraph = p->boostGraph;

	//boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rGraph);

	SplineKdTree::spline_hit_vector hits;
	math::flt dist = p->nearestSpline.findClosestQualified(hits, pos, CheckDir(carDir));
	math::flt dist2 = dist * dist;

	BOOST_FOREACH(SplineKdTree::spline_hit_vector::const_reference hit, hits) {
		math::LaneSpline const * pSpline = hit.first;
		math::flt const closestParm = hit.second;

		std::vector<edge_descr> const & ref = p->reverseMap[pSpline].first;	// Non-mirrored ones
		assert(!ref.empty()); // The edges should follow the direction of the spline
		std::vector<edge_descr>::const_iterator it = std::lower_bound(ref.begin() + 1, ref.end(), closestParm, CompareToSource(p->edgeDataMap));

		return make_tuple(*(it - 1), closestParm, dist);
	}

	return make_tuple(edge_descr(), -std::numeric_limits<math::flt>::max(), -std::numeric_limits<math::flt>::max());
}

/** returns the list of edges that is nearest to the given point, the second value is the distance
 */

math::flt RNDFGraph::getNearestEdges(std::vector< std::pair< edge_descr, math::flt > >& ret, const math::Vec2 & punkt, math::flt maxDist, math::flt accuracy) const
{
	//aGraph const & rGraph = p->boostGraph;

	//boost::property_map<aGraph, edge_data_t>::const_type	edgeDataMap = get(edge_data_t(), rGraph);

	SplineKdTree::spline_hit_vector hits;
	math::flt dist = p->nearestSpline.findClosest(hits, punkt, maxDist, accuracy);
	math::flt dist2 = dist * dist;

	BOOST_FOREACH(SplineKdTree::spline_hit_vector::const_reference hit, hits) {
		math::LaneSpline const * pSpline = hit.first;
		math::flt closestParm = hit.second;
		std::vector<edge_descr> & references = p->reverseMap[pSpline].first;	// Non-mirrored ones
		std::vector<edge_descr>::const_iterator pos = std::lower_bound(references.begin() + 1, references.end(), closestParm, CompareToSource(p->edgeDataMap)) - 1;
		assert(pos != references.end());
		ret.push_back(std::make_pair(*pos, closestParm));
	}

	return dist2;
}

template<class Archive>
void RNDFGraph::serialize(Archive & ar, const unsigned int version)
{
	//////////////////////////////
	///////// just a bit of a hack ... somehow the z coordinate is set?!?
	//math::Vec3 globalZero = math::geodetic::theGlobalZero::instance().getLLH();
	//std::cout << "global zero is: " << globalZero[0] << ", " << globalZero[1] << ", " << globalZero[2] << std::endl;
#if 0
	theGlobalZero::instance().set(globalZero);
	// deactivated hack
	// GlobalZero::load(...) calls GlobalZero::update() which resolves the problem
	// see math/Geodetic.h
#endif

//        if ( globalZero[2] != 0 ){
//		cout << "global zero is: " << globalZero[0] << ", " << globalZero[1] << ", " << globalZero[2] << std::endl;
//		cout << "resetting z to 0" << std::endl;
//		globalZero[2] = 0;
//		theGlobalZero::instance().set(globalZero);
//	}
	////////////////
	ar & p->boostGraph;
}


void RNDFGraph::load(std::string const & fname)
{
	if (fname.rfind(".arnd") == std::string::npos) {
		return;
	}

	std::ifstream inputFileStream(fname.c_str());
	boost::archive::binary_iarchive boostArchive(inputFileStream);
	boostArchive & math::geodetic::theGlobalZero::instance();
	boostArchive & *this;

	/*
	std::vector<vertex_descr> toRemove;
	BOOST_FOREACH (vertex_descr const & v, vertices( p->boostGraph )) {
		std::pair<edge_descr, bool> inedge = p->inEdge(v);
		std::pair<edge_descr, bool> outedge = p->outEdge(v);
		if (!inedge.second && !outedge.second ){
			toRemove.push_back(v);
		}
	}

	BOOST_FOREACH (vertex_descr const & v, toRemove){
		BOOST_FOREACH(edge_descr const & e, out_edges(v, p->boostGraph)) {
			remove_edge(e, p->boostGraph);
		}
		BOOST_FOREACH(edge_descr const & e, in_edges(v, p->boostGraph)) {
			remove_edge(e, p->boostGraph);
		}
		remove_vertex (v, p->boostGraph);
	}

	p->buildGraphHierarchy();

	p->variousGraphFixes();

	boost::property_map<aGraph, vertex_index_t>::type vertexIndexMap = get(vertex_index_t(), p->boostGraph);
	unsigned int i = 0;
	BOOST_FOREACH(vertex_descr const & v, vertices(p->boostGraph)) {
		vertexIndexMap[v] = i++;
	}

	BOOST_FOREACH (vertex_descr const & v, vertices( p->boostGraph )) {
		VertexData vertexData(p->vertexDataMap[v]);
		if ( vertexData.l0 == 114){
			cout << vertexData.l1 << "." << vertexData.id << std::endl;
		}
	}

	BOOST_FOREACH (edge_descr const & v, edges( p->boostGraph )) {
		EdgeData edgeData(p->edgeDataMap[v]);
		if ( edgeData.segment == 114){
			cout << edgeData.lane << "." << std::endl;
		}
	}
	*/
//	boost::property_map<aGraph, edge_index_t>::type edgeIndexMap = get(edge_index_t(), p->boostGraph);
//	i = 0;
//	BOOST_FOREACH(edge_descr v, edges(p->boostGraph)) {
//		edgeIndexMap[v] = i++;
//	}

	p->buildGraphHierarchy();
	p->buildCheckpoints();
	resplinify();
	p->buildSKDTree();
//	p->initConfidences();
	p->fillReverseMap();
//	p->splinify();

	signalGraphChanged(*this);
//	~boostArchive();
	inputFileStream.close();
}

void RNDFGraph::save(std::string const & fname)
{
	if (fname.rfind(".arnd") != std::string::npos) {
		std::ofstream outputFileStream(fname.c_str());
		boost::archive::binary_oarchive boostArchive(outputFileStream);
		boostArchive & math::geodetic::theGlobalZero::instance();
		boostArchive & *this;
		//	~boostArchive();
		outputFileStream.close();
		return;
	}

	// get graph data
// 	boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), const_cast<aGraph const &>(p->boostGraph));
	boost::property_map<aGraph, vertex_data_t>::const_type	vertexDataMap = get(vertex_data_t(), const_cast<aGraph const &>(p->boostGraph));

	// open stream
	std::ofstream os;
	os.open(fname.c_str());
	os.precision(30);

	// write rndf header
	os << "RNDF_name\t" << fname << std::endl;
	os << "num_segments\t" << numSegments() << std::endl;
	os << "num_zones\t" << p->mGraphHierarchy.second.size() << std::endl;
	os << "format_version\t" << "1.0" << std::endl;
	time_t t = time(0);

	char timeString[18];
	strftime(timeString, 18, "%Y-%m-%d_%H:%M", localtime(&t));
	os << "creation_date\t" << timeString << std::endl;

	// promote all inserted points to real way points
	for (uint segmentID = 1; segmentID <= p->mGraphHierarchy.first.size(); ++segmentID) {
		for (uint laneID = 1; laneID <= p->mGraphHierarchy.first[segmentID - 1].size(); ++laneID) {
			p->promoteInsertedPoints(segmentID, laneID);
		}
	}

	// iterate on the segments
	uint segmentID = 0;
	BOOST_FOREACH(segment_vec_type segment, p->mGraphHierarchy.first) {
		// increment segment ID and write segment header
		os << "segment\t" << ++segmentID << std::endl;
		os << "num_lanes\t" << segment.size() << std::endl;
		os << "segment_name\t" << std::endl; //TODO: when loading RNDF, also save segment names

		// iterate on the lanes
		for (uint laneID = 1; laneID <= segment.size(); ++laneID) {
			// get the lane
			lane_vec_type lane = segment[laneID - 1];

			// write lane header
			os << "lane\t" <<  segmentID << '.' << laneID << std::endl;
			os << "num_waypoints\t" << lane.size() + 1 << std::endl;

			// How do we treat inserted points?
			// We may have lanes that only consist out of inserted points
			// but we don't like to export empty lanes.
			// So we promote inserted points to 'real' way points
			// which makes sence, as we only name points as inserted
			// because we want to keep IDs persistent in runtime.
			// As a result, way point IDs may change.

			// get lane properties
			math::flt width = vertexDataMap[source(lane[0], p->boostGraph)].laneWidth;
			BoundaryType leftBoundary = p->edgeDataMap[lane[0]].boundaryLeft;
			BoundaryType rightBoundary = p->edgeDataMap[lane[0]].boundaryRight;

			// write lane width
			if (width != 0 && !ENHANCED_RNDF) {
				os << "lane_width\t" << int(width * ::math::M_2_FT + 0.5) << std::endl;
			}

			// write lane properties
			if (leftBoundary != LANE_BOUNDARY_NONE) {
				os << "left_boundary\t" << boundaryType_str[leftBoundary] << std::endl;
			}

			if (rightBoundary != LANE_BOUNDARY_NONE) {
				os << "right_boundary\t" << boundaryType_str[rightBoundary] << std::endl;
			}

			// iterate on the way points and find all checkpoints
			std::vector<vertex_descr> waypoints = p->getLaneWaypoints(segmentID, laneID);
			BOOST_FOREACH(vertex_descr v, waypoints) {
				uint checkpointID = p->vertexDataMap[v].checkpoint;

				if (checkpointID > 0) {
					// write check point
					os << "checkpoint\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << '\t' << checkpointID << std::endl;
				}
			}

			// iterate on the way points and find all stop signs
			BOOST_FOREACH(vertex_descr v, waypoints) {
				if (p->vertexDataMap[v].vertexType.isSet(VertexData::STOP_SIGN)) {
					// write stop sign
					os << "stop\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << std::endl;
				}
			}

			// iterate on the way points and find all traffic lights
			BOOST_FOREACH(vertex_descr v, waypoints) {
				if (p->vertexDataMap[v].vertexType.isSet(VertexData::TRAFFIC_LIGHT)) {
					// write traffic light
					if (ENHANCED_RNDF) {
						os << "#traffic_light\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << std::endl;
					}
					else {
						os << "/* #traffic_light\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << "*/" << std::endl;
					}
				}
			}

			// iterate on the way points and find all decision points
			BOOST_FOREACH(vertex_descr v, waypoints) {
				if (p->vertexDataMap[v].vertexType.isSet(VertexData::DECISION_POINT)) {
					// write decision point
					if (ENHANCED_RNDF) {
						os << "#decision_point\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << std::endl;
					}
					else {
						os << "/* #decision_point\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << "*/" << std::endl;
					}
				}
			}

			// iterate on the way points and find all give way signs
			BOOST_FOREACH(vertex_descr v, waypoints) {
				if (p->vertexDataMap[v].vertexType.isSet(VertexData::GIVE_WAY)) {
					// write give way sign
					if (ENHANCED_RNDF) {
						os << "#give_way\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << std::endl;
					}
					else {
						os << "/* #give_way\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << "*/" << std::endl;
					}
				}
			}

			// iterate on the way points and find all exits
			BOOST_FOREACH(vertex_descr v, waypoints) {
				BOOST_FOREACH(edge_descr const & e, p->getOutgoingConnections(v)) {
					// write exit
					VertexData targetData = p->vertexDataMap[target(e, p->boostGraph)];
					os << "exit\t" << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << '\t';
					os << targetData.l0 << '.' << targetData.l1 << '.' << targetData.id << std::endl;
				}
			}

			// iterate on the way points
			math::flt laneWidth = 0.0f;
			BOOST_FOREACH(vertex_descr v, waypoints) {
				// write lane width, if necessary
				if (p->vertexDataMap[v].laneWidth != laneWidth && ENHANCED_RNDF) {
					laneWidth = p->vertexDataMap[v].laneWidth;
					os << "#lane_width\t" << uint(laneWidth * ::math::M_2_FT + 0.5f) << std::endl;
				}

				// convert from ENU to WGS84 coordinates system
				math::Vec2d dpos(p->vertexDataMap[v].pos(0), p->vertexDataMap[v].pos(1));
				math::Vec3d posTrans = math::geodetic::ENUtoWGS84(dpos);

				// write formatted longitude and latitude
				os << segmentID << '.' << laneID << '.' << p->vertexDataMap[v].id << '\t';
				os.precision(8);
				os << posTrans(1) << '\t' << posTrans(0) << std::endl;
			}

			// write lane footer
			os << "end_lane" << std::endl << std::endl;
		}

		// write segment footer
		os << "end_segment" << std::endl << std::endl << std::endl;
	}

	// iterate on the zones
	BOOST_FOREACH(zone_vec_type zone, p->mGraphHierarchy.second) {
		// increment segment ID and write zone header
		os << "zone" << '\t' << ++segmentID << std::endl;
		os << "num_spots" << '\t' << zone.second.size() << std::endl;

		// write perimeter header
		os << "perimeter" << '\t' << segmentID << ".0" << std::endl;
		os << "num_perimeterpoints" << '\t' << zone.first.size() << std::endl;

		// iterate on perimeter points and find all exits
		BOOST_FOREACH(vertex_descr v, zone.first) {
			// iterate on the outgoing connections
			BOOST_FOREACH(edge_descr const & e, p->getOutgoingConnections(v)) {
				// check, whether we have a real exit (helper-connections may also be found)
				VertexData targetData = p->vertexDataMap[target(e, p->boostGraph)];

				if (targetData.l0 != segmentID) {
					// write exit
					os << "exit\t" << segmentID << ".0." << p->vertexDataMap[v].id << '\t';
					os << targetData.l0 << '.' << targetData.l1 << '.' << targetData.id << std::endl;
				}
			}
		}

		//iterate on the perimeter points
		BOOST_FOREACH(vertex_descr v, zone.first) {
			// convert from ENU to WGS84 coordinates system
			math::Vec2d dpos(p->vertexDataMap[v].pos(0), p->vertexDataMap[v].pos(1));
			math::Vec3d posTrans = math::geodetic::ENUtoWGS84(dpos);

			// write formatted longitude and latitude
			os << segmentID << ".0." << p->vertexDataMap[v].id << '\t';
			os.precision(8);
			os << posTrans(1) << '\t' << posTrans(0) << std::endl;
		}

		// write perimeter footer
		os << "end_perimeter" << std::endl << std::endl;

		// iterate on spots
		uint spotID = 0;
		BOOST_FOREACH(edge_descr const & e, zone.second) {
			// increment spot ID and write spot header
			os << "spot\t" << segmentID << '.' << ++spotID << std::endl;
			uint width = uint(p->vertexDataMap[source(e, p->boostGraph)].laneWidth * ::math::M_2_FT + 0.5);
			os << "spot_width\t" << width << std::endl;

			// write check point for spot target
			uint checkpointID = p->vertexDataMap[source(e, p->boostGraph)].checkpoint;

			if (checkpointID > 0) {
				// write check point
				os << "checkpoint\t" << segmentID << '.' << spotID <<  '.' << p->vertexDataMap[source(e, p->boostGraph)].id << '\t' << checkpointID << std::endl;
			}

			// write check point for spot source
			checkpointID = p->vertexDataMap[target(e, p->boostGraph)].checkpoint;

			if (checkpointID > 0) {
				// write check point
				os << "checkpoint\t" << segmentID << '.' << spotID << '.' << p->vertexDataMap[target(e, p->boostGraph)].id << '\t' << checkpointID << std::endl;
			}

			// write first spot point
			math::Vec2d dpos1(p->vertexDataMap[source(e, p->boostGraph)].pos(0), p->vertexDataMap[source(e, p->boostGraph)].pos(1));
			math::Vec3d posTrans = math::geodetic::ENUtoWGS84(dpos1);
			os << segmentID << '.' << spotID << '.' << p->vertexDataMap[source(e, p->boostGraph)].id << '\t';
			os.precision(8);
			os << posTrans(1) << '\t' << posTrans(0) << std::endl;

			// write 2nd spot point
			math::Vec2d dpos2(p->vertexDataMap[target(e, p->boostGraph)].pos(0), p->vertexDataMap[target(e, p->boostGraph)].pos(1));
			posTrans = math::geodetic::ENUtoWGS84(dpos2);
			os << segmentID << '.' << spotID << '.' << p->vertexDataMap[target(e, p->boostGraph)].id << '\t';
			os.precision(8);
			os << posTrans(1) << '\t' << posTrans(0) << std::endl;

			// write spot footer
			os << "end_spot" << std::endl;
		}

		// write zone footer
		os << "end_zone" << std::endl << std::endl;
	}

	// write rndf footer
	os << "end_file" << std::endl;

	// close stream
	os.flush();
	os.close();
}

#if 0
// this function maybe can adjust the coordinates of a given rndf graph by newly recorded data
// it is not used currently, but may be in the future, so better keep the code
void RNDFGraph::repositionGraph(RNDFGraph * other)
{
	//boost::property_map<aGraph, vertex_data_t>::type vertexDataMapOther = get(vertex_data_t(), const_cast<aGraph &>(other->getBoostGraph()));

	std::vector<std::pair<vertex_descr, math::flt> > possibles;

	// first get a checkpoint from our graph
	BOOST_FOREACH(vertex_descr v, vertices(p->boostGraph)) {
		if (vertexDataMap[v].checkpoint > 0) {
			possibles.clear();
			// search for a checkpoint near ours in the other graph
			other->getNearestVertices(possibles, vertexDataMap[v].pos, 20.0f * 20.0f);

			// now check for the nearest one that is a checkpoint
			vertex_descr vert(GraphTraits::null_vertex());

			while (!possibles.empty()) {
				vert = possibles.front().first;

				// we could check if the checkpoint id is the same but that
				// seems unrealistic for recorded data
				if (vertexDataMapOther[vert.get()].checkpoint > 0) {
					break;
				}

				pop_heap(possibles.begin(), possibles.end(), OrderBySecond< pair<vertex_descr, math::flt>, std::greater_equal<math::flt> >());
				possibles.pop_back();
			}

			if (vert != GraphTraits::null_vertex()) {
				// we found a checkpoint in the other graph that is near to ours.
				// move our checkpoint there
				math::Vec2 offsetDir = vertexDataMap[v].pos - vertexDataMapOther[*vert].pos;
				vertexDataMap[v].pos = vertexDataMapOther[*vert].pos;

				// Now what we do with the normal waypoints???
				// We could move all other waypoints the same that we moved this one with a distance
				// falloff. Then hopefully together with the other checkpoints everything
				// is streched into place (needs at least some equally distributed checkpoints)
				BOOST_FOREACH(vertex_descr const & v2, vertices(p->boostGraph)) {
					// we only move none checkpoints, cause the checkpoints will be moved to exact
					// locations
					if (vertexDataMap[v2].checkpoint <= 0) {
						// we use linear falloff maybe exponential is better???
						math::flt falloff = 1.0f / (vertexDataMap[v].pos - vertexDataMap[v2].pos).norm();
						vertexDataMap[v2].pos = vertexDataMap[v2].pos + offsetDir * falloff;
					}
				}
			}
		}
	}
}
#endif

void RNDFGraph::recalculateData()
{
	resplinify();
	p->buildSKDTree();
	p->initConfidences();
	p->fillReverseMap();
	signalGraphChanged(*this);
}

void RNDFGraph::variousGraphFixes()
{
	p->variousGraphFixes();
}

void RNDFGraph::optimiseGraph()
{
#if defined (STATISTICS)
	sleep(2);
	unsigned long vertexCountBefore = num_vertices(p->boostGraph);
	unsigned long edgeCountBefore = num_edges(p->boostGraph);
	unsigned long long pruneVerticesTime;
	unsigned long long buildMirrorGraphTime;
	unsigned long long addLaneChangingEdgesTime;

	std::cout << "\nrndfGraphStats ->----------------------------------------------------------";
	std::cout << "\nrndfGraphStats -> Before Optimise: VertexCount: " << vertexCountBefore;
	std::cout << "\nrndfGraphStats -> Before Optimise: EdgeCount: " << edgeCountBefore;

	TimeStamp startTime = TimeStamp();
	TimeStamp endTime = TimeStamp();
	startTime.stamp();
#endif

	p->pruneVertices();

#if defined (STATISTICS)
	endTime.stamp();
	pruneVerticesTime = endTime - startTime;
	startTime.stamp();
#endif

	p->buildMirrorGraph();

#if defined (STATISTICS)
	endTime.stamp();
	buildMirrorGraphTime = endTime - startTime;
	startTime.stamp();
#endif
	p->addLaneChangingEdges();

#if defined (STATISTICS)
	endTime.stamp();
	addLaneChangingEdgesTime = endTime - startTime;
#endif

	isOptimised = true;
	signalGraphChanged(*this);

#if defined (STATISTICS)
	unsigned long long totalTime = pruneVerticesTime + buildMirrorGraphTime + addLaneChangingEdgesTime;
	unsigned long vertexCountAfter = num_vertices(p->boostGraph);
	unsigned long edgeCountAfter = num_edges(p->boostGraph);

	std::map<std::string, math::flt> laneLength;
	BOOST_FOREACH(edge_descr const & e, edges(p->boostGraph)) {
		EdgeData & edgeData = p->edgeDataMap[e];

		if (!edgeData.isMirrored) {
			math::flt targetParam  = edgeData.targetParam;
			stringstream key;
			key << edgeData.segment << "_" << edgeData.lane;

			if (laneLength[key.str()] < targetParam) {
				laneLength[key.str()] = targetParam;
			}
		}
	}

	float totalLength = 0;
	std::map<std::string, math::flt>::iterator it = laneLength.begin();

	for (; it != laneLength.end(); it++) {
//		cout << "\n id:"<< it->first << " length:	" << it->second;
		totalLength += it->second;
	}

	std::cout
			<< "\nrndfGraphStats -> After Optimise: VertexCount: " << vertexCountAfter << " reduced by: " << (1 - (vertexCountAfter / (float)vertexCountBefore)) * 100 << "%"
			<< "\nrndfGraphStats -> After Optimise: EdgeCount: " << edgeCountAfter << " reduced by: " << (1 - (edgeCountAfter / (float)edgeCountBefore)) * 100 << "%"
			<< "\nrndfGraphStats -> combinedLaneLength: " << totalLength << "\n"
			<< "\nrndfGraphStats -> OptimiseTimes:"
			<< "\nrndfGraphStats -> pruneVertices: " << pruneVerticesTime << " nsecs, ratio: " << pruneVerticesTime * 100 / (float)totalTime << "% of totalTime"
			<< "\nrndfGraphStats -> buildMirrorGraph: " << buildMirrorGraphTime << " nsecs, ratio: " << buildMirrorGraphTime * 100 / (float)totalTime << "% of totalTime"
			<< "\nrndfGraphStats -> addLaneChangingEdges: " << addLaneChangingEdgesTime << " nsecs, ratio: " << addLaneChangingEdgesTime * 100 / (float)totalTime << "% of totalTime"
			<< "\nrndfGraphStats -> totalTime: " << totalTime << " nsecs"
			<< "\nrndfGraphStats -> " << vertexCountBefore << "," << vertexCountAfter << "," << edgeCountBefore << "," << edgeCountAfter << "," << pruneVerticesTime << "," << buildMirrorGraphTime << "," << addLaneChangingEdgesTime
			<< "\nrndfGraphStats -> ----------------------------------------------------------\n";
	// quit programm ... the dirty way
	assert(false);
#endif

//	std::cout << " Optimized Graph:" << std::endl;
//	boost::property_map<aGraph, edge_weight_t>::type edgeWeightMap = get(edge_weight_t(), p->boostGraph);
//	boost::property_map<aGraph, vertex_index_t>::type vertexIndexMap = get(vertex_index_t(), p->boostGraph);

//	BOOST_FOREACH(edge_descr e, edges(p->boostGraph)) {
//		EdgeData  & edgeData(p->edgeDataMap[e]);
//		VertexData & src(p->vertexDataMap[e.m_source]);
//		VertexData & dst(p->vertexDataMap[e.m_target]);


//		std::cout << src.name << " to " << dst.name << " Weight: " << edgeWeightMap[e]  << std::endl;
//		std::cout << " m_source: " << vertexIndexMap[source(e, p->boostGraph)]
//				  << " m_target: " << vertexIndexMap[target(e, p->boostGraph)] << std::endl;
//	}


//	BOOST_FOREACH(vertex_descr v, vertices(p->boostGraph)) {
//		VertexData  & vertexData(p->vertexDataMap[v]);

//		std::cout << vertexData.name << std::endl;
//		std::cout << "Check all out edges:" << std::endl;
//		GraphTraits::out_edge_iterator ei, ei_end;

//		for (tie(ei, ei_end) = out_edges(v, p->boostGraph); ei != ei_end; ++ei) {
//			GraphTraits::vertex_descriptor v_target = target(*ei, p->boostGraph);
//			VertexData  & vertexData2(p->vertexDataMap[v_target]);
//			std::cout << "-> " << vertexData2.name << " : " << std::endl;
//// << get(id1, *v)

//		}

//	}

	// this has to be done
	boost::property_map<aGraph, boost::vertex_index_t>::type vertexIndexMap2 = get(boost::vertex_index_t(), p->boostGraph);
	unsigned int i = 0;
	BOOST_FOREACH(vertex_descr v, vertices(p->boostGraph)) {
		vertexIndexMap2[v] = i++;
	}

	signalGraphChanged(*this);

//	std::cout << "Go Mr Johnson" << std::endl;

//	//init weight Matrix with infinity
//	typedef std::vector< std::vector<math::flt> > WeightMatrix;

//	WeightMatrix mDistMatrix;

//	aGraph const & rGraph = getBoostGraph();
//	unsigned long V = num_vertices(rGraph);

//	mDistMatrix.resize(V);

//	for (unsigned long i = 0; i < V; ++i) {
//		mDistMatrix[i].resize(V, std::numeric_limits<math::flt>::infinity());
//	}


//	// Johnsons apsp algo runs in O(VE logV) time
//	bool ret = johnson_all_pairs_shortest_paths(
//				   const_cast<aGraph &>(rGraph),
//				   mDistMatrix
//			   );
}

std::pair<edge_descr, bool> RNDFGraph::findNonMirroredEdge(math::flt param, boost::shared_ptr< ::math::LaneSpline const> laneSpline) const
{

	return p->findNonMirroredEdge(param,  laneSpline);
}

std::pair<edge_descr, bool> RNDFGraph::findMirroredEdge(math::flt param, boost::shared_ptr< ::math::LaneSpline const> laneSpline) const
{
	return p->findMirroredEdge(param,  laneSpline);
}

}
}
}
}

namespace boost
{
namespace serialization
{

template<class Archive, class Base>
void serialize(Archive & ar, ::boost::property< ::boost::vertex_predecessor_t, aa::modules::models::rndf::aGraphTraits::vertex_descriptor, Base>& prop, const unsigned int version)
{
    //this does not work with boost >=1.51 and we dont need it is we are not loading rndfs
    //	ar & ::boost::serialization::make_nvp("property_base" , boost::serialization::base_object<Base>(prop));
}

}
}


