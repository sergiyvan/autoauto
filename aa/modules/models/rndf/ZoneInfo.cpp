#include "ZoneInfo.h"
#include <rtt/Logger.hpp>
#include <boost/foreach.hpp>
#include <vgl/vgl_polygon.h>
#include <vcl_vector.h>

#include <patterns/Singleton.h>

#include "RndfGraph.h"
#include <util/Templates.h>
#include <math/AutoMath.h>


using namespace ::math;
using namespace ::aa::modules::models::rndf;

typedef patterns::Singleton<RNDFGraph> theRNDFGraph;

ZoneInfo::ZoneInfo()
{}

ZoneInfo::~ZoneInfo()
{}



void ZoneInfo::computeZones()
{
	RNDFGraph & rndfGraph = theRNDFGraph::instance();
	aGraph const & boostGraph = const_cast<aGraph &>(rndfGraph.getBoostGraph());
	boost::property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), boostGraph);

	//recompute zones
	clear();

	BOOST_FOREACH(zone_vec_type const & zoneVec, rndfGraph.getGraphVector().second) {
		Zone zone(zoneVec);

		//create perimeter polygon, center and radius
		zone.mPerimeterPolygon.new_sheet();
		::math::Vec2 center(0.f, 0.f);
		BOOST_FOREACH(vertex_descr perimeterPt, zoneVec.first) {
			::math::Vec2 pos = vertexDataMap[perimeterPt].pos;
			center += pos;
			zone.mPerimeterPolygon.push_back(vgl_point_2d<flt>(pos(0), pos(1)));
		}
		center /= ::math::flt(zoneVec.first.size());
		zone.mCenter = center;

		::math::flt radius2(0);
		//for(uint i=0; i<zo.mPerimeterPts.size();i++)
		BOOST_FOREACH(vertex_descr perimeterPt, zoneVec.first) {
			radius2 = std::max(radius2, ssd(vertexDataMap[perimeterPt].pos, center));
		}

		zone.mRadius2 = radius2;

		//create spots
		zone.mParkingSpots.clear();
		BOOST_FOREACH(edge_descr spotEdge, zoneVec.second) {
			vertex_descr src = source(spotEdge, boostGraph);
			vertex_descr dst = target(spotEdge, boostGraph);

			assert(vertexDataMap[src].vertexType.isSet(VertexData::PARKINGSPOT));
			assert(vertexDataMap[dst].vertexType.isSet(VertexData::PARKINGSPOT));

			zone.mParkingSpots.push_back(Zone::ParkingSpot(src, dst, spotEdge));
		}

		mZones.push_back(zone);
	}


// 	property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rGraph);
// 	property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);
//
// 	mZones.clear();
//
// 	BOOST_FOREACH(vertex_descr v1, vertices(rGraph)) {
// 		VertexData const & vData = vertexDataMap[v1];
//
// 		if (!vData.vertexType.isSet(VertexData::IN_ZONE)
// 			&& !vData.vertexType.isSet(VertexData::PERIMETER_POINT)
// 			&& !vData.vertexType.isSet(VertexData::PARKINGSPOT) )
// 			continue;
//
// 		map_type::iterator it = mZones.find(vData.l0);
// 		if(it == mZones.end())
// 			mZones[vData.l0] = computeZone(rGraph, v1, vData.l0);
// 	}
//
// 	return true;
}


bool ZoneInfo::Zone::isPointInZone(::math::Vec2 const & pos) const
{
	if (ssd(mCenter, pos) > mRadius2) {
		return false;
	}

	return mPerimeterPolygon.contains(pos(0), pos(1));

//	vgl_polygon<flt> isect;
//	isect.new_sheet();
//	flt dir = 1;
//	isect.push_back(vgl_point_2d<flt>(pos(0) - dir,	pos(1) - dir));
//	isect.push_back(vgl_point_2d<flt>(pos(0) - dir,	pos(1) + dir));
//	isect.push_back(vgl_point_2d<flt>(pos(0) + dir,	pos(1) + dir));
//	isect.push_back(vgl_point_2d<flt>(pos(0) + dir,	pos(1) - dir));

//	vgl_polygon<flt> clipZone = vgl_clip(isect, mPerimeterPolygon);

//	return clipZone.num_vertices() > 0;
}

bool ZoneInfo::Zone::isQuadInZone(math::Quad2d const & quad) const
{
	//check if a corner point is in zone
	if (isPointInZone(quad.corner(0))) {
		return true;
	}

	if (isPointInZone(quad.corner(1))) {
		return true;
	}

	if (isPointInZone(quad.corner(2))) {
		return true;
	}

	if (isPointInZone(quad.corner(3))) {
		return true;
	}

	//else do a edge check
	for (uint i = 0; i < 4; i++) {
		Vec2 a = quad.corner(i);
		Vec2 b = quad.corner((i + 1) % 4);

		for (uint j = 0; j < mPerimeterPolygon.num_sheets(); j++) {
			vcl_vector< vgl_point_2d<flt> > sheet = mPerimeterPolygon[j];

			Vec2 p = Vec2(sheet[0].x(), sheet[0].y());
			Vec2 q = Vec2(sheet[1].x(), sheet[1].y());

			Vec2 cross;

			if (geradeStreckenSchnitt(a, b, p, q, cross)) {
				return true;
			}
		}
	}

	return false;
}

bool ZoneInfo::Zone::isPolygonInZone(vgl_polygon<math::flt> const & poly) const
{
	//check if a corner point is in zone
	for (uint j = 0; j < poly.num_sheets(); j++) {
		vcl_vector< vgl_point_2d<flt> > sheet = poly[j];

		Vec2 p = Vec2(sheet[0].x(), sheet[0].y());

		if (isPointInZone(p)) {
			return true;
		}
	}

	//else do a edge check
	for (uint i = 0; i < poly.num_sheets(); i++) {
		vcl_vector< vgl_point_2d<flt> > poly_sheet = poly[i];

		Vec2 a = Vec2(poly_sheet[0].x(), poly_sheet[0].y());
		Vec2 b = Vec2(poly_sheet[1].x(), poly_sheet[1].y());

		for (uint j = 0; j < mPerimeterPolygon.num_sheets(); j++) {
			vcl_vector< vgl_point_2d<flt> > sheet = mPerimeterPolygon[j];

			Vec2 p = Vec2(sheet[0].x(), sheet[0].y());
			Vec2 q = Vec2(sheet[1].x(), sheet[1].y());

			Vec2 cross;

			if (geradeStreckenSchnitt(a, b, p, q, cross)) {
				return true;
			}
		}
	}

	return false;
}


boost::optional<ZoneInfo::Zone> ZoneInfo::findZone(::math::Vec2 const & pos) const
{
	vgl_polygon<flt> isect;
	isect.new_sheet();
	flt dir = 1;
	isect.push_back(vgl_point_2d<flt>(pos(0) - dir,	pos(1) - dir));
	isect.push_back(vgl_point_2d<flt>(pos(0) - dir,	pos(1) + dir));
	isect.push_back(vgl_point_2d<flt>(pos(0) + dir,	pos(1) + dir));
	isect.push_back(vgl_point_2d<flt>(pos(0) + dir,	pos(1) - dir));

	BOOST_FOREACH(Zone const & zone, mZones) {
		if (zone.isPolygonInZone(isect)) {
			return zone;
		}
	}
	return boost::optional<Zone>();
}

uint ZoneInfo::Zone::getZoneID() const
{
	RNDFGraph & rndfGraph = theRNDFGraph::instance();
	aGraph & boostGraph = const_cast<aGraph &>(rndfGraph.getBoostGraph());
	boost::property_map<aGraph, vertex_data_t>::type vertexDataMap = get(vertex_data_t(), boostGraph);

	return vertexDataMap[zoneVec.first[0]].l0;
}

void ZoneInfo::clear()
{
	mZones.clear();
}
