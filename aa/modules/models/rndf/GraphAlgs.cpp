#include "GraphAlgs.h"
#include <util/Templates.h>
#include <math/PathSpline.h>
#include <patterns/Singleton.h>
#include <iostream>
#include <cmath>

#include "RndfGraph.h"

using namespace ::math;
using namespace ::aa::modules::models::rndf;

typedef patterns::Singleton<RNDFGraph> theRNDFGraph;

// boost::tuple<flt, flt> findClosestPoint(LaneSpline const & spline, flt from, flt to, Vec2 const & point)
// {
// 	return math::findClosestPoint(spline, from, to, 0.5f*(from+to), point);
// }
boost::tuple<flt, flt> findClosestPoint(LaneSpline const & spline, flt from, flt to, Vec2 const & point)
{
	flt minParm = from;
	flt minDist2 = ssd(spline(from), point);

	for (from += 0.1f; from < to; from += 0.1f) {
		flt dist2 = ssd(spline(from), point);

		if (dist2 < minDist2) {
			minParm = from;
			minDist2 = dist2;
		}
	}

	return boost::make_tuple(minDist2, minParm);
}

void laneMarkings(std::vector<Vec2> & markings,  Vec2 const & pos, Vec2 const & on)
{
	RNDFGraph const & rndfGraph = theRNDFGraph::instance();
	aGraph & rGraph = const_cast<aGraph &>(rndfGraph.getBoostGraph());

	markings.clear();
	// TODO: Remove hard-coded value, when getNearestEdges is fixed (does not return all the edges in the given area
	std::vector<std::pair<edge_descr, flt> > edges;
	flt const minDist2 = theRNDFGraph::instance().getNearestEdges(edges, pos, 5.0f);

	if (edges.empty()) {
		return;
	}

	boost::property_map<aGraph, edge_data_t>::type
	edgeDataMap = get(edge_data_t(), rGraph);

	edge_descr minEdge;
	flt minParam;

	for (std::vector<std::pair<edge_descr, flt> >::const_iterator it = edges.begin();
			it != edges.end(); ++it) {
		EdgeData const & edgeData = edgeDataMap[it->first];

		if (edgeData.isConnection) {	// Skip connecting edges
			continue;
		}

		minEdge = it->first;
		minParam = it->second;

		break;
	}

	if (minEdge == edge_descr()) {
		return;
	}

	markings.resize(2);
	EdgeData const & edgeData = edgeDataMap[minEdge];

	flt const halfLaneWidth = 0.5f * rndfGraph.laneWidth(minEdge, minParam);
	Vec2 laneCentrePos = (*edgeData.laneSpline())(minParam);
	Vec2 orthDir = normalized(edgeData.laneSpline()->firstDerivative(minParam));
	std::swap(orthDir(0), orthDir(1));
	orthDir(0) = -orthDir(0);
	markings[0] = laneCentrePos - halfLaneWidth * orthDir;
	markings[1] = laneCentrePos + halfLaneWidth * orthDir;
}


flt maxCurvature(LaneSpline const & spline, flt from, flt to)
{
	flt _maxCurvature = 0.f;

	size_t fromIdx = spline.findIndex(from);
	size_t toIdx = spline.findIndex(to);
	LaneSpline::DomVec const & abscissae = spline.abscissae();
	LaneSpline::CoeffVec const  & coeffs = spline.coeffs();

	assert(abscissae.size());

	size_t maxIdx = std::min(toIdx, abscissae.size() - 2);

	for (size_t idx = fromIdx; idx <= maxIdx; ++idx) {
		LaneSpline::Coeff const & coeff = coeffs[idx];
		flt fromVal = abscissae[idx];
		flt toVal = abscissae[idx + 1];
		flt fromVal2 = *(&(*abscissae.begin()) + idx);
		assert(fromVal == fromVal2);
		const flt ax = coeff[0](0);
		const flt bx = coeff[1](0);
		const flt cx = coeff[2](0);

		const flt ay = coeff[0](1);
		const flt by = coeff[1](1);
		const flt cy = coeff[2](1);


		// curv(t) = (6bA - 6Ba) t² +
		//		(6Ac - 6aC) t +
		//		2cB - 2Cb
		/*
		#define CURVEVAL(t) std::abs( (6*bx*ay - 6*by*ax) * t * t + (6*ay*cx - 6*ax*cy) * t + 2*cx*by - 2*cy*bx )

				flt delta = toVal - fromVal;
				flt curveValfrom = std::abs( 2*cx*by - 2*cy*bx);
				flt curveValTo = 	CURVEVAL(delta);

		//		maxCurvature = std::max(maxCurvature, curveValfrom);
		//		maxCurvature = std::max(maxCurvature, curveValTo);
		*/
		{
			Vec2 first = spline.firstDerivative(fromVal);
			Vec2 second = spline.secondDerivative(fromVal);
			flt curvature = std::abs(first(0) * second(1) + first(1) * second(0));
			/*
						flt xi =  cx;
						flt yi =  cy;
						if(	std::abs(xi - first(0)) > 0.0001f || std::abs(yi - first(1)) > 0.0001f)
							std::cout << "first= "<< first << "!= "<<  Vec2(xi,yi) <<std::endl;

						flt xii =  2*bx;
						flt yii =  2*by;
						if(	std::abs(xii - second(0)) > 0.0001f || std::abs(yii - second(1)) > 0.0001f)
							std::cout << "AAAAA second= "<< second << "!= "<<  Vec2(xii,yii) <<std::endl;
						else
							std::cout << "second ok" << std::endl;


						if(curvature > 100.f)
							std::cout << "aaaaaaaaaaaaaaaaaa idx = "<< idx << "size= "<< abscissae.size() <<std::endl;
						if(	std::abs(curveValfrom - curvature) > 0.0001f )
							std::cout << "curvature = "<< curvature << "!= curveValfrom = "<< curveValfrom <<std::endl;
						std::cout << "idx = "<< idx << "size= "<< abscissae.size() <<std::endl;
			*/
			_maxCurvature = std::max(_maxCurvature, curvature);
		}

		// compute extremum of curvature (see wikipedia)
		// curv(t) = x'y'' - x''y' / (grob konstant, da )
		//t = (aC - Ac) / (bA - Ba)
// 		flt param = ( coeff[0](0) * coeff[2](1) - coeff[0](1) *coeff[2](0) ) /
// 					  ( coeff[1](0) * coeff[0](1) -  coeff[1](1) * coeff[0](0));
//
// 		if(param > 0 && param < delta)
// 		{
// 			flt curveValEtremum = CURVEVAL(param);
// 			maxCurvature = std::max(maxCurvature, curveValEtremum);
// 		}
	}

	return _maxCurvature;
}

flt minRadius(LaneSpline const & spline, flt from, flt to)
{
	flt _maxCurvature = maxCurvature(spline, from, to);

	if (_maxCurvature < 1e-6f) {
		return std::numeric_limits<flt>::infinity();
	}

	return 1.f / _maxCurvature;
}
