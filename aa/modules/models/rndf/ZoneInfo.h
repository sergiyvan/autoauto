#pragma once
#include <boost/noncopyable.hpp>
#include <vgl/vgl_polygon.h>
#include <math/Quad2d.h>

#include "RndfGraph.h"

class ZoneInfo
	: boost::noncopyable
{
public:
	ZoneInfo();
	~ZoneInfo();

	void computeZones();

	class Zone
	{
	public:
		Zone()
		{}

		explicit Zone(::aa::modules::models::rndf::zone_vec_type const & z)
			: zoneVec(z) {
		}


		class ParkingSpot
		{
		public:
			ParkingSpot(::aa::modules::models::rndf::vertex_descr src, ::aa::modules::models::rndf::vertex_descr dst, ::aa::modules::models::rndf::edge_descr edge)
				: mSrcWaypoint(src)
				, mDstWaypoint(dst)
				, mSpotEdge(edge)
			{ }

			::aa::modules::models::rndf::vertex_descr getSrcWaypoint() const {
				return mSrcWaypoint;
			}
			::aa::modules::models::rndf::vertex_descr getDstWaypoint() const {
				return mDstWaypoint;
			}
			::aa::modules::models::rndf::edge_descr getSpotEdge() const {
				return mSpotEdge;
			}

		protected:
			::aa::modules::models::rndf::vertex_descr mSrcWaypoint;
			::aa::modules::models::rndf::vertex_descr mDstWaypoint;
			::aa::modules::models::rndf::edge_descr mSpotEdge;
		};


		//check for point in zone
		bool isPointInZone(::math::Vec2 const & pos) const;

		//check for quad in zone
		bool isQuadInZone(math::Quad2d const & quad) const;

		//check for polygon in zone
		bool isPolygonInZone(vgl_polygon<math::flt> const & poly) const;

		uint getZoneID() const;

		math::flt mRadius2;
		math::Vec2 mCenter;
		vgl_polygon<math::flt> mPerimeterPolygon;

		std::vector<ParkingSpot> mParkingSpots;

		//source type
		::aa::modules::models::rndf::zone_vec_type zoneVec;

	};


	typedef boost::shared_ptr<Zone> zone_ptr;
	typedef std::map< uint, zone_ptr > map_type;

	boost::optional<Zone> findZone(math::Vec2 const & posInZone) const;

	std::vector<Zone> const & getZones() const {
		return mZones;
	}

	void clear();

protected:
	//zone_ptr computeZone(vertex_descr v1, uint id);
	std::vector<Zone> mZones;
};


