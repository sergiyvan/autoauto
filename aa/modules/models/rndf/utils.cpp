#include "utils.h"

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{


edge_descr findLanes(aa::modules::models::rndf::RNDFGraph const & rndfGraph, math::Vec2 const & vehiclePos2d, std::list<boost::tuple<edge_descr, math::flt, math::Vec2, EdgeData> > & lanes)
{
	boost::property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rndfGraph.getBoostGraph());

	std::vector<std::pair<edge_descr, math::flt> > closeEdges;
	rndfGraph.getNearestEdges(closeEdges, vehiclePos2d, 10.0);

	lanes.clear();
	math::flt paramLeft, paramRight;
	math::flt closestSquaredDist;
	edge_descr startEdge, leftEdge, rightEdge;
	math::Vec2 posLeft, posRight;
	math::flt currParam;
	edge_descr currEdge;

	if (!closeEdges.empty()) {
		EdgeData const & edgeData = edgeDataMap[closeEdges[0].first];
		boost::shared_ptr<math::LaneSpline const> laneSpline =  edgeData.laneSpline();

		if (laneSpline) {
			currEdge = closeEdges[0].first;
			currParam = closeEdges[0].second;
			startEdge = currEdge;
			lanes.push_back(boost::tuple<edge_descr, math::flt, math::Vec2, EdgeData>(currEdge, currParam, vehiclePos2d, edgeData));
			while (true) {
				bool foundLeft = rndfGraph.getDirectNeighbouredEdge(currParam, currEdge, true, leftEdge, posLeft, paramLeft, closestSquaredDist);
				if (!foundLeft) {
					break;
				}
				currEdge = leftEdge;
				currParam = paramLeft;
				if (currEdge != lanes.front().get<0>()) {
					lanes.push_front(boost::tuple<edge_descr, math::flt, math::Vec2, EdgeData>(currEdge, currParam, posLeft, edgeDataMap[currEdge]));
				} else {
					break;
				}
			}
			currEdge = startEdge;
			while (true) {
				bool foundRight = rndfGraph.getDirectNeighbouredEdge(currParam, currEdge, false, rightEdge, posRight, paramRight, closestSquaredDist);
				if (!foundRight) {
					break;
				}
				currEdge = rightEdge;
				currParam = paramRight;
				if (currEdge != lanes.back().get<0>()) {
					lanes.push_front(boost::tuple<edge_descr, math::flt, math::Vec2, EdgeData>(currEdge, currParam, posRight, edgeDataMap[currEdge]));
				} else {
					break;
				}
			}
		}
	}
	return startEdge;
}

}


}


}


}


