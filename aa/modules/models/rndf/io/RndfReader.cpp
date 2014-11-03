#include "RndfReader.h"
// #define BOOST_SPIRIT_DEBUG
#include <rtt/Logger.hpp>
#include <boost/spirit/include/classic.hpp>
#include <boost/bind.hpp>
#include "BoundaryType.h"

//#define VERBOSE

#if defined(VERBOSE)
#define DLOG(X) Logger::log() << Logger::Debug << X << Logger::endl;
#else
#define DLOG(X)
#endif

using namespace boost;
using namespace boost::spirit;
using namespace boost::spirit::classic;
using RTT::Logger;
using namespace ::math;

typedef char char_t;
typedef file_iterator <char_t>  iterator_t;

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

namespace io
{


template<typename ostream>
ostream & operator<<(ostream & o, WaypointID const w)
{
	o << w.segment << '.' << w.lane << '.' << w.waypoint;
}

struct rndf_grammar
		: public grammar<rndf_grammar> {
	typedef rndf_grammar self_t;
	friend struct definition;

	rndf_grammar(RNDFData * _rndfData)
		: rndfData(_rndfData)
		, laneWidth(0.f)
	{	}

	template <typename ScannerT>
	struct definition {
		definition(rndf_grammar const & _self) {
			rndf_grammar & self = const_cast<rndf_grammar &>(_self);

			comment = comment_p("/*", "*/");
			opt = *(comment | blank_p);
			optnl = *(comment | space_p);
			str = *graph_p;
			level1_id = uint_p[bind(&self_t::setLevel1, ref(self), _1)];
			level2_id = level1_id >> ch_p('.') >> uint_p[bind(&self_t::setLevel2, ref(self), _1)];
			level3_id = level2_id >> ch_p('.') >> uint_p[bind(&self_t::setLevel3, ref(self), _1)];

			optional_file_header =
				*(strlit<>("format_version") >> *blank_p >> str[bind(&self_t::setFormatVersion, ref(self), _1, _2)] >> optnl
				  | strlit<>("creation_date") >> *blank_p >> str[bind(&self_t::setCreationDate, ref(self), _1, _2)] >> optnl
				 );

			optional_lane_header = *(
									   strlit<>("lane_width") >> *blank_p >> ureal_p[bind(&self_t::setLaneWidth, ref(self), _1)] >> optnl
									   | strlit<>("left_boundary") >> *blank_p >> str[bind(&self_t::setLeftBoundary, ref(self), _1, _2)] >> optnl
									   | strlit<>("right_boundary") >> *blank_p >> str[bind(&self_t::setRightBoundary, ref(self), _1, _2)] >> optnl
									   | strlit<>("zone_lane")[bind(&self_t::setZoneLane, ref(self), _1, _2)] >> optnl
									   | strlit<>("checkpoint") >> *blank_p >> level3_id >> *blank_p >> uint_p[bind(&self_t::setLaneCheckpoint, ref(self), _1)] >> optnl
									   | strlit<>("stop") >> *blank_p >> level3_id >> opt >> eol_p[bind(&self_t::addLaneStop, ref(self))] >> optnl
									   | strlit<>("#traffic_light") >> *blank_p >> level3_id >> opt >> eol_p[bind(&self_t::addTrafficLight, ref(self))] >> optnl
									   | strlit<>("#traffic_light_two_phases") >> *blank_p >> level3_id >> opt >> eol_p[bind(&self_t::addTrafficLightTwoPhases, ref(self))] >> optnl
									   | strlit<>("#decision_point") >> *blank_p >> level3_id >> opt >> eol_p[bind(&self_t::addDecisionPoint, ref(self))] >> optnl
									   | strlit<>("#give_way") >> *blank_p >> level3_id >> opt >> eol_p[bind(&self_t::addGiveWay, ref(self))] >> optnl
									   | strlit<>("#hidden") >> *blank_p >> eol_p[bind(&self_t::hideLane, ref(self))]
									   | strlit<>("exit") >> *blank_p >> level3_id >> blank_p[bind(&self_t::saveWaypoint, ref(self))] >> *blank_p >> level3_id >> opt >> eol_p[bind(&self_t::addLaneExit, ref(self))]
								   );

			waypoint =
				level3_id >> *blank_p >> real_p[bind(&self_t::setLatitude, ref(self), _1)] >> *blank_p >> real_p[bind(&self_t::setLongitude, ref(self), _1)] >> opt;

			perimeter =	   strlit<>("perimeter") >> *blank_p >> level2_id >> optnl
						   >> strlit<>("num_perimeterpoints") >> *blank_p >> uint_p[bind(&self_t::setNumPerimiterpoints, ref(self), _1)] >> optnl
						   >> *(strlit<>("exit") >> *blank_p >> level3_id >> blank_p[bind(&self_t::saveWaypoint, ref(self))] >> *blank_p >> level3_id >> opt >> eol_p[bind(&self_t::addPerimeterExit, ref(self))] >> optnl)
						   >> +(waypoint >> eol_p[bind(&self_t::setPerimeterpoint, ref(self))] >> optnl)
						   >> strlit<>("end_perimeter");

			optional_spot_header = *(
									   strlit<>("spot_width") >> *blank_p >> ureal_p[bind(&self_t::setSpotWidth, ref(self), _1)] >> optnl
									   | strlit<>("checkpoint") >> *blank_p >> level3_id >> *blank_p >> uint_p[bind(&self_t::setSpotCheckpoint, ref(self), _1)] >> optnl
								   );

			spot =	   strlit<>("spot") >> *blank_p >> level2_id >> optnl
					   >> optional_spot_header
					   >> (waypoint >> eol_p[bind(&self_t::setSpotpoint, ref(self))] >> optnl)
					   >> (waypoint >> eol_p[bind(&self_t::setSpotpoint, ref(self))] >> optnl)
					   >> strlit<>("end_spot")[bind(&self_t::endSpot, ref(self))];

			zone =	   strlit<>("zone") >> *blank_p >> level1_id >> optnl
					   >> strlit<>("num_spots") >> *blank_p >> uint_p[bind(&self_t::setNumSpots, ref(self), _1)] >> optnl
					   >> !(strlit<>("zone_name") >> *blank_p >> str[bind(&self_t::setZoneName, ref(self), _1, _2)] >> optnl)
					   >> perimeter >> optnl
					   >> *(spot >> optnl)
					   >> strlit<>("end_zone")[bind(&self_t::endZone, ref(self))];;

			lane =	   strlit<>("lane") >> *blank_p >> level2_id >> optnl
					   >> strlit<>("num_waypoints") >> *blank_p >> uint_p[bind(&self_t::setNumWaypoints, ref(self), _1)] >> optnl
					   >> optional_lane_header
					   >> +(
						   *(strlit<>("#lane_width") >> *blank_p >> ureal_p[bind(&self_t::setLaneWidth, ref(self), _1)] >> optnl)
						   >> waypoint >> eol_p[bind(&self_t::setLaneWaypoint, ref(self))] >> optnl
					   )
					   >> strlit<>("end_lane")[bind(&self_t::checkLane, ref(self))];

			segment =  strlit<>("segment") >> *blank_p >> level1_id >> optnl
					   >> strlit<>("num_lanes") >> *blank_p >> uint_p[bind(&self_t::setNumLanes, ref(self), _1)] >> optnl
					   >> !(strlit<>("segment_name") >> *blank_p >> str[bind(&self_t::setSegmentName, ref(self), _1, _2)] >> optnl)
					   >> +(lane >> optnl)
					   >> strlit<>("end_segment");

			baseExpression = optnl
							 >> strlit<>("RNDF_name") >> *blank_p >> str[bind(&self_t::setRNDFName, ref(self), _1, _2)] >> optnl
							 >> strlit<>("num_segments") >> *blank_p >> uint_p[bind(&self_t::setNumSegments, ref(self), _1)] >> optnl
							 >> strlit<>("num_zones") >> *blank_p >> uint_p[bind(&self_t::setNumZones, ref(self), _1)] >> optnl
							 >> optional_file_header
							 >> +(segment >> optnl)
							 >> *(zone >> optnl)
							 >> strlit<>("end_file") >> optnl >> end_p;

			BOOST_SPIRIT_DEBUG_RULE(opt);
			BOOST_SPIRIT_DEBUG_RULE(optnl);
			BOOST_SPIRIT_DEBUG_RULE(str);
			BOOST_SPIRIT_DEBUG_RULE(comment);
			BOOST_SPIRIT_DEBUG_RULE(level1_id);
			BOOST_SPIRIT_DEBUG_RULE(level2_id);
			BOOST_SPIRIT_DEBUG_RULE(level3_id);
			BOOST_SPIRIT_DEBUG_RULE(waypoint);
			BOOST_SPIRIT_DEBUG_RULE(optional_spot_header);
			BOOST_SPIRIT_DEBUG_RULE(spot);
			BOOST_SPIRIT_DEBUG_RULE(optional_lane_header);
			BOOST_SPIRIT_DEBUG_RULE(lane);
			BOOST_SPIRIT_DEBUG_RULE(segment);
			BOOST_SPIRIT_DEBUG_RULE(optional_perimeter_header);
			BOOST_SPIRIT_DEBUG_RULE(perimeter);
			BOOST_SPIRIT_DEBUG_RULE(zone);
			BOOST_SPIRIT_DEBUG_RULE(optional_file_header);
			BOOST_SPIRIT_DEBUG_RULE(baseExpression);
		}

		rule<ScannerT>
		opt,
		optnl,
		str,
		comment,
		level1_id,
		level2_id,
		level3_id,
		waypoint,
		optional_spot_header,
		spot,
		optional_lane_header,
		lane,
		segment,
		optional_perimeter_header,
		perimeter,
		zone,
		optional_file_header,
		baseExpression;

		rule<ScannerT> const & start() const {
			return baseExpression;
		}
	};

	void setRNDFName(iterator_t first, iterator_t last) {
		rndfData->filename = std::string(first, last);
	}

	void setFormatVersion(iterator_t first, iterator_t last) {
		rndfData->format_version = std::string(first, last);
	}

	void setCreationDate(iterator_t first, iterator_t last) {
		rndfData->creation_date = std::string(first, last);
	}

	void setNumSegments(uint numSegments) {
		rndfData->segments.resize(numSegments);
		DLOG(numSegments << " segments");
	}

	void setNumZones(uint numZones) {
		rndfData->zones.resize(numZones);
		DLOG(numZones << " zones");
	}

	void setNumSpots(uint numSpots) {
		Zone & zone = currZone();

		zone.id = waypoint.segment;
		zone.spots.resize(numSpots);

		DLOG("Zone " <<  zone.id << ": " << numSpots << " spots");
	}

	void setNumLanes(uint numLanes) {
		Segment & segment = currSegment();
		assert(segment.id == 0);	// The first to set it
		segment.id = waypoint.segment;
		segment.lanes.resize(numLanes);

		DLOG("Segment " << segment.id << ": " << numLanes << " lanes");
	}

	void setNumWaypoints(uint numWaypoints) {
		Lane & lane = currLane();

		assert(lane.id == 0);	// The first to set it
		lane.segment = waypoint.segment;
		lane.id = waypoint.lane;
		lane.waypoints.resize(numWaypoints);

		DLOG("Lane " << lane.segment << '.' << lane.id << ": " << numWaypoints << " waypoints ");
	}

	void setSegmentName(iterator_t first, iterator_t last) {
		Segment & segment = currSegment();
		segment.name = std::string(first, last);
	}

	void setZoneName(iterator_t first, iterator_t last) {
		Zone & zone = currZone();
		zone.name = std::string(first, last);
	}

	void setSpotWidth(flt spotWidth) {
		Spot & spot = currSpot();
		spot.width = spotWidth;
	}

	void hideLane() {
		Lane & lane = currLane();
		lane.isHidden = true;
	}

	void endSpot() {
		Spot & spot = currSpot();
		spot.id = waypoint.lane;
		spot.zone = waypoint.segment;

		DLOG("Finished spot" <<  spot.id);
	}

	void endZone() {
		DLOG("Finished Zone");
	}


	void setLeftBoundary(iterator_t first, iterator_t last) {
		Lane & lane = currLane();
		lane.left_boundary = string2boundaryType(std::string(first, last));
	}

	void setRightBoundary(iterator_t first, iterator_t last) {
		Lane & lane = currLane();
		lane.right_boundary = string2boundaryType(std::string(first, last));
	}

	void setZoneLane(iterator_t first, iterator_t last) {
		Lane & lane = currLane();
		lane.zone_lane = true;
	}

	void setLaneCheckpoint(uint checkpointID) {
		Waypoint & wp = currWaypoint();
		wp.checkpoint_id = checkpointID;
	}

	void setSpotCheckpoint(uint checkpointID) {
		Waypoint & wp = currSpotpoint();
		wp.checkpoint_id = checkpointID;
	}

	void addLaneStop() {
		Waypoint & wp = currWaypoint();
		wp.stopsign = true;
	}

	void addGiveWay() {
		Waypoint & wp = currWaypoint();
		wp.giveWay = true;
	}

	void addTrafficLight() {
		Waypoint & wp = currWaypoint();
		wp.trafficLight = true;
	}

	void addTrafficLightTwoPhases() {
		Waypoint & wp = currWaypoint();
		wp.trafficLightTwoPhases = true;
	}

	void addDecisionPoint() {
		Waypoint & wp = currWaypoint();
		wp.decisionPoint = true;
	}

	void saveWaypoint() {
		waypoint2 = waypoint;
	}

	void setNumPerimiterpoints(uint numPerimeterpoints) {
		Zone & zone = currZone();
		zone.perimeterpoints.resize(numPerimeterpoints);
		DLOG("Zone " <<  zone.id << ": " << numPerimeterpoints << " perimeterpoints");
	}

	void addLaneExit() {
		std::swap(waypoint2, waypoint);
		currWaypoint().exit_to.push_back(waypoint2);
	}

	void addPerimeterExit() {
		std::swap(waypoint2, waypoint);
		currPerimeterpoint().exit_to.push_back(waypoint2);
	}

	void setLaneWaypoint() {
		Waypoint & wp = currWaypoint();
// 		Logger::log() << Logger::Debug << waypoint << ": " << latitude << " " << longitude << Logger::endl;
		wp.id = waypoint.waypoint;
		wp.latitude = latitude;
		wp.longitude = longitude;
		wp.laneWidth = laneWidth;
	}

	void setSpotpoint() {
		Waypoint & wp = currSpotpoint();

		DLOG("Spot " <<  waypoint);

		wp.isSpot = true;
		wp.id = waypoint.waypoint;
		wp.latitude = latitude;
		wp.longitude = longitude;
	}

	void setPerimeterpoint() {
		Waypoint & wp = currPerimeterpoint();

		DLOG("Perimeter " <<  waypoint);

		wp.isPerimeterPoint = true;
		wp.id = waypoint.waypoint;
		wp.latitude = latitude;
		wp.longitude = longitude;
	}

	Segment & currSegment() {
		assert(waypoint.segment <= rndfData->segments.size());
		return rndfData->segments[waypoint.segment - 1];
	}

	Zone & currZone() {
		uint zoneidx = waypoint.segment - rndfData->segments.size() - 1;
		assert(zoneidx < rndfData->zones.size());
		return rndfData->zones[zoneidx];
	}

	Spot & currSpot() {
		Zone & zone = currZone();
		assert(waypoint.lane <= zone.spots.size());
		return zone.spots[waypoint.lane - 1];
	}

	Lane & currLane() {
		Segment & segment = currSegment();
		assert(waypoint.lane <= segment.lanes.size());
		return segment.lanes[waypoint.lane - 1];
	}

	Waypoint & currWaypoint() {
		Lane & lane = currLane();
		assert(waypoint.waypoint <= lane.waypoints.size());
		return lane.waypoints[waypoint.waypoint - 1];
	}

	Waypoint & currPerimeterpoint() {
		Zone & zone = currZone();
		assert(waypoint.waypoint <= zone.perimeterpoints.size());
		return zone.perimeterpoints[waypoint.waypoint - 1];
	}

	Waypoint & currSpotpoint() {
		Spot & spot = currSpot();
		assert(waypoint.waypoint <= spot.waypoints.size());
		return spot.waypoints[waypoint.waypoint - 1];
	}

	void setLatitude(flt l) {
		latitude = l;
	}

	void setLongitude(flt l) {
		longitude = l;
	}

	void setLevel1(uint id) {
		waypoint.segment = id;
	}

	void setLevel2(uint id) {
		waypoint.lane = id;
	}

	void setLevel3(uint id) {
		waypoint.waypoint = id;
	}

	void setLaneWidth(flt lw) {
		laneWidth = lw;
	}

	void checkLane() {
		Lane const & lane = currLane();

		for (uint i = 0; i < lane.waypoints.size(); ++i) {
			Waypoint const & wp = lane.waypoints[i];

			if (wp.id != i + 1) {
				Logger::log(Logger::Error) << "Waypoint " << lane.segment << '.' << lane.id << '.' << i + 1 << " is incorrectly set to ID " << wp.id << Logger::endl;
			}
		}

		// reset lane width
		laneWidth = 0;
	}

	RNDFData * rndfData;
	WaypointID waypoint, waypoint2;
	flt latitude;
	flt longitude;
	flt laneWidth;
};

RNDFReader::RNDFReader()
{
}

RNDFReader::~RNDFReader()
{
}

boost::shared_ptr<RNDFData> RNDFReader::loadRNDF(std::string const & fname)
{
	Logger::In in("RNDFReader");
	iterator_t first(fname);

	// check, if file could be opened
	if (!first) {
		Logger::log(Logger::Error) << "failed opening file \"" << fname << '"' << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	// parse file
	iterator_t last = first.make_end();
	boost::shared_ptr<RNDFData> ptr(new RNDFData);
	rndf_grammar g(ptr.get());
	parse_info<iterator_t> info = parse(first, last, g);

	// check, if file successfully parsed
	if (!info.hit) {
		Logger::log(Logger::Error) << "RNDF-Parse-Error at " << info.stop << Logger::endl;
		ptr.reset();
	}

	return ptr;
}

}

}

}

}

}

