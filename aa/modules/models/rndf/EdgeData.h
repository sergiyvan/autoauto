#pragma once
#include "io/BoundaryType.h"
#include <math/LaneSpline.h>
#include <math/Types.h>
namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

//////////////////////////////////////// class EdgeData ////////////////////////////////

class EdgeData
{
public:
	typedef ::math::flt flt;
	/**
	* enumeration for the edge type
	*/
	enum edge_type {
		NONE = 0,
		LANE_EDGE = 1,
		LANE_CONNECTION = 2,
		U_TURN = 4,
		SPOT_EDGE = 8,
		PERIMETER_EDGE = 16,
		ZONE_CONNECTION = 32,
		MIRRORED_EDGE = 64,
		LANE_CHANGING = 128
	};

	enum street_type {
		STREET = 0,
		HIGHWAY,
		ACCELERATIONLANE,
		DECELERATIONLANE,
		ROUNDABOUT,
		EMERGENCYLANE,
		STREET_TYPE_MAX
	};

	EdgeData()
		: name("")
		, boundaryLeft(aa::modules::models::rndf::io::LANE_BOUNDARY_BROKEN_WHITE)
		, boundaryRight(aa::modules::models::rndf::io::LANE_BOUNDARY_BROKEN_WHITE)
		, inZone(false)
		, isConnection(false)
		, isUTurn(false)
		, isMirrored(false)
		, isHidden(false)
		, streetType(STREET)
		, segment(0)
		, lane(0)
		, sourceParam(0.0)
		, targetParam(0.0)
		, mLaneSpline()
		, mLeftLaneSpline()
		, mRightLaneSpline()
		, leftLaneParamRange(std::make_pair(0.0, 0.0))
		, rightLaneParamRange(std::make_pair(0.0, 0.0))
		, mParamDir(1.0)
	{}

	EdgeData(EdgeData const & other)
		: name(other.name)
		, boundaryLeft(other.boundaryLeft)
		, boundaryRight(other.boundaryRight)
		, inZone(other.inZone)
		, isConnection(other.isConnection)
		, isUTurn(other.isUTurn)
		, isMirrored(other.isMirrored)
		, isHidden(other.isHidden)
		, streetType(other.streetType)
		, segment(other.segment)
		, lane(other.lane)
		, sourceParam(other.sourceParam)
		, targetParam(other.targetParam)
		, mLaneSpline(other.mLaneSpline)
		, mLeftLaneSpline(other.mLeftLaneSpline)
		, mRightLaneSpline(other.mRightLaneSpline)
		, leftLaneParamRange(other.leftLaneParamRange)
		, rightLaneParamRange(other.rightLaneParamRange)
		, mParamDir(other.mParamDir) {
		assert(mParamDir == 1.0f || mParamDir == -1.0f);
	}

	~EdgeData()
	{}

	EdgeData & operator=(EdgeData const & other) {
		if (&other == this) {
			return *this;
		}

		name = other.name;
		boundaryLeft = other.boundaryLeft;
		boundaryRight = other.boundaryRight;
		inZone = other.inZone;
		isConnection = other.isConnection;
		isUTurn = other.isUTurn;
		isMirrored = other.isMirrored;
		isHidden = other.isHidden;
		streetType = other.streetType;
		mParamDir = other.mParamDir;
		sourceParam = other.sourceParam;
		targetParam = other.targetParam;
		mLaneSpline = other.mLaneSpline;
		mLeftLaneSpline = other.mLeftLaneSpline;
		mRightLaneSpline = other.mRightLaneSpline;
		leftLaneParamRange = other.leftLaneParamRange;
		rightLaneParamRange = other.rightLaneParamRange;
		segment = other.segment;
		lane = other.lane;

		assert(mParamDir == 1.0f || mParamDir == -1.0f);

		return *this;
	}

	void setParamDir(flt paramDir) {
		assert(paramDir == 1.0f || paramDir == -1.0f);
		mParamDir = paramDir;
	}

	flt paramDir() const {
		return mParamDir;
	}

	std::string getStreetTypeName() const {
		switch (streetType) {
		case STREET:
			return "Street";

		case HIGHWAY:
			return "Highway";

		case ACCELERATIONLANE:
			return "Accelerationlane";

		case DECELERATIONLANE:
			return "Decelerationlane";

		case ROUNDABOUT:
			return "Roundabout";

		case EMERGENCYLANE:
			return "Emergencylane";

		default:
			return "unknown";
		}

	}

	std::string name;

	aa::modules::models::rndf::io::BoundaryType boundaryLeft;
	aa::modules::models::rndf::io::BoundaryType boundaryRight;

	bool inZone;
	bool isConnection;
	bool isUTurn;
	bool isMirrored;
	bool isHidden;
	street_type streetType;


    boost::shared_ptr< ::math::LaneSpline const > laneSpline() const {
		return mLaneSpline;
	}

    boost::shared_ptr< ::math::LaneSpline > & laneSpline() {
		return mLaneSpline;
	}

    boost::shared_ptr< ::math::LaneSpline const > leftLaneSpline() const {
		return mLeftLaneSpline;
	}

    boost::shared_ptr< ::math::LaneSpline > & leftLaneSpline() {
		return mLeftLaneSpline;
	}

    boost::shared_ptr< ::math::LaneSpline const > rightLaneSpline() const {
		return mRightLaneSpline;
	}

    boost::shared_ptr< ::math::LaneSpline > & rightLaneSpline() {
		return mRightLaneSpline;
	}


	unsigned int segment;
	unsigned int lane;

	flt sourceParam;
	flt targetParam;

	std::pair<flt, flt> leftLaneParamRange;
	std::pair<flt, flt> rightLaneParamRange;

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & name;
		ar & boundaryLeft;
		ar & boundaryRight;
		ar & inZone;
		ar & isConnection;
		ar & isUTurn;
		ar & isMirrored;
		ar & isHidden;
		ar & streetType;
		ar & segment;
		ar & lane;
		ar & sourceParam;
		ar & targetParam;
		ar & mLaneSpline;
		ar & mLeftLaneSpline;
		ar & mRightLaneSpline;
		ar & leftLaneParamRange;
		ar & rightLaneParamRange;
		ar & mParamDir;
	}

protected:
    boost::shared_ptr< ::math::LaneSpline> mLaneSpline;
    boost::shared_ptr< ::math::LaneSpline> mLeftLaneSpline;
    boost::shared_ptr< ::math::LaneSpline> mRightLaneSpline;
	flt mParamDir;
}; // end of class EdgeData

}


}


}


}

//namespace aa::modules::models::rndf
