#pragma once
#include <math/Types.h>
#include <util/BitMask.h>
#include <boost/any.hpp>
#include <boost/serialization/shared_ptr.hpp>
namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

/////////////////////////////////////// struct VertexData //////////////////////////////

class VertexData
{
	typedef ::math::flt flt;

	/**
	* enumeration for the vertex type
	*/
public:

	enum vertex_type {	NONE                                                    = 0,
						WAY_POINT 				= 1,
						PARKINGSPOT				= 2,
						PERIMETER_POINT 			= 4,
						IN_ZONE 				= 8,
						STOP_SIGN 				= 16,
						TRAFFIC_LIGHT 				= 32,
						TRAFFIC_LIGHT_TWO_PHASES                = 64,
						GIVE_WAY 				= 128,
						DECISION_POINT 				= 256,
						EXIT 					= 512,
						INSERTED 				= 1024,
						MIRROR_VERTEX 				= 2048,
						CHECK_POINT 				= 4096,
						ENTRY 					= 8192
					 };

	enum TrafficSignalState {
		UNKNOWN		= 0,
		RED		= 1,
		RED_YELLOW	= 2,
		YELLOW		= 3,
		GREEN		= 4,
		FORCE_RED       = 5,
		FORCE_RED_YELLOW = 6,
		FORCE_YELLOW    = 7,
		FORCE_GREEN     = 8,
		FORCE_RED_CV_RED        = 9,
		FORCE_RED_CV_RED_YELLOW = 10,
		FORCE_RED_CV_YELLOW     = 11,
		FORCE_RED_CV_GREEN      = 12,
		FORCE_RED_YELLOW_CV_RED        = 13,
		FORCE_RED_YELLOW_CV_RED_YELLOW = 14,
		FORCE_RED_YELLOW_CV_YELLOW     = 15,
		FORCE_RED_YELLOW_CV_GREEN      = 16,
		FORCE_YELLOW_CV_RED        = 17,
		FORCE_YELLOW_CV_RED_YELLOW = 18,
		FORCE_YELLOW_CV_YELLOW     = 19,
		FORCE_YELLOW_CV_GREEN      = 20,
		FORCE_GREEN_CV_RED        = 21,
		FORCE_GREEN_CV_RED_YELLOW = 22,
		FORCE_GREEN_CV_YELLOW     = 23,
		FORCE_GREEN_CV_GREEN      = 24
	};

	VertexData()
		: name("")
		, pos(0.f, 0.f)
		, laneWidth(0.f)
		, checkpoint(0)
		, vertexType() // All cleared
		, l0(0)
		, l1(0)
		, id(0)
		, metaData()
	{}

	~VertexData()
	{}

	VertexData(VertexData const & other)
		: name(other.name)
		, pos(other.pos)
		, laneWidth(other.laneWidth)
		, checkpoint(other.checkpoint)
		, vertexType(other.vertexType)
		, l0(other.l0)
		, l1(other.l1)
		, id(other.id)
		, metaData(other.metaData)
	{}

	VertexData & operator=(VertexData const & other) {
		if (&other == this) {
			return *this;
		}

		name = other.name;
		pos = other.pos;
		laneWidth = other.laneWidth;
		checkpoint = other.checkpoint;
		vertexType = other.vertexType;
		l0 = other.l0;
		l1 = other.l1;
		id = other.id;
		metaData = other.metaData;

		return *this;
	}

	/**
	* the name of this vertex
	*/
	std::string name;

	/**
	* the position of this vertex in world coordinates
	*/
	::math::Vec2 pos;

	/**
	 * Width of the lane at the point
	 */
	flt laneWidth;

	/**
	* the checkpoint ID of this vertex.
	* If ID == 0, then this point is not a checkpoint
	*/
	uint checkpoint;

	/**
	* type of the vertex
	*/
	util::BitMask vertexType;

	/**
	* meta data
	*/
	boost::any metaData;

	/**
	* TODO: refactor ID as a triple
	*/
	uint l0;	// Level0: Segment/Zone
	uint l1;	// Level1: Lane/Spot
	uint id;	// ID:	   Waypoint Id

	// not really needed, everything is public anyway
	friend class boost::serialization::access;

	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {

		ar & name;
		ar & pos;
		ar & laneWidth;
		ar & checkpoint;
		ar & vertexType;
		ar & l0;
		ar & l1;
		ar & id;
//		std::cout << "Vertex Name:" << name << " id: " << l0 << "." << l1 << "." << id << " Width: " << laneWidth << std::endl;
//		ar & metaData;

	}

}; // end of struct VertexData

}


}


}


}

 //namespace aa::modules::models::rndf
