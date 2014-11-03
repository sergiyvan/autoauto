#pragma once

#include <util/RtTaskContext.h>
#include "RndfData.h"

#include <boost/shared_ptr.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/OperationCaller.hpp>
#include <util/Ports.h>

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

class RNDFGenerator
	: public util::RtTaskContext
{
public:
	typedef double flt;
	typedef ::math::Vec2d Vec2;
	explicit RNDFGenerator(std::string const & name);
	virtual ~RNDFGenerator();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	/**
	* Convenience method to create a default straight RNDF with its default parameters in Tempelhof, Berlin
	* \return true if succesful
	*/
	bool generateAndLoadDefaultStraightRNDF(std::string const & fname);

	/**
	* Convenience method to create a default circle RNDF with its default parameters in Tempelhof, Berlin
	* \return true if succesful
	*/
	bool generateAndLoadDefaultCircleRNDF(std::string const & fname);

	/**
	* Convenience method to create a default eight RNDF with its default parameters in Tempelhof, Berlin
	* \return true if succesful
	*/
	bool generateAndLoadDefaultEightRNDF(std::string const & fname);

	/**
	* Convenience method to create a default multilane RNDF with its default parameters in Tempelhof, Berlin
	* \return true if succesful
	*/
	bool generateAndLoadDefaultMultilaneRNDF(std::string const & fname);

	/**
	* Generates and loads an RDNF file with a circle segment
	* \param startPointLat latitude of start point of straight segment
	* \param startPointLong longitude of start point of straight segment
	* \param endPointLat latitude of end point of straight segment
	* \param endPointLong longitude of end point of straight segment
	* \param fname filename under which to save RNDF
	* \return true if succesful
	*/
	bool generateAndLoadStraightRNDF(flt startPointLat, flt startPointLong, flt endPointLat, flt endPointLong, std::string const & fname);

	/**
	* Generates and loads an RDNF file with a circle segment
	* \param midPointLat latitude of center of circle
	* \param midPointLong longitude of center of circle
	* \param radius radius of circle in meter
	* \param fname filename under which to save RNDF
	* \return true if succesful
	*/
	bool generateAndLoadCircleRNDF(flt midPointLat, flt midPointLong, flt radius, std::string const & fname);

	/**
	* Generates and loads an RDNF file with a eight segment
	* \param midPointLat latitude of center of eight
	* \param midPointLong longitude of center of eight
	* \param radius radius of eight circles in meter
	* \param swivel angle in radian at which a eight circle changes to the other circle
	* \param rotAngle angle in radian after which the eight is rotated
	* \param fname filename under which to save RNDF
	* \return true if succesful
	*/
	bool generateAndLoadEightRNDF(flt midPointLat, flt midPointLong, flt radius, flt swivel, flt rotAngle, std::string const & fname);

	/**
	* Generates and loads an RDNF file with a multiple lanes
	* \param midPointLat latitude of center of crossing
	* \param midPointLong longitude of center of crossing
	* \param segmentLength length of segment in meter (12 segments in total)
	* \param lanes number of lanes for each segment
	* \param fname filename under which to save RNDF
	* \return true if succesful
	*/
	bool generateAndLoadMultilaneRNDF(flt midPointLat, flt midPointLong, flt segmentLength, int lanes, std::string const & fname);

protected:
	RTT::os::Mutex mMutex;
	boost::shared_ptr<RNDFData> mRNDFData;

	/** \name Properties: */
	//\{
	RTT::Property<flt>
	mStraightStartLat,
	mStraightStartLong,
	mStraightEndLat,
	mStraightEndLong;

	RTT::Property<flt>
	mCircleLat,
	mCircleLong,
	mCircleRadius;

	RTT::Property<flt>
	mEightLat,
	mEightLong,
	mEightRadius,
	mEightSwivel,
	mEightRotAngle;

	RTT::Property<flt>
	mMultilaneLat,
	mMultilaneLong,
	mMultilaneSegmentLength;
	RTT::Property<int>
	mMultilaneLanes;

	RTT::Property<flt>
	mSampleSizeDegree,
	mSampleSizeUnit;
	//\}

private:
	boost::shared_ptr<RNDFData> generateStraightRNDF(Vec2 startPoint, Vec2 endPoint, std::string const & fname);
	boost::shared_ptr<RNDFData> generateCircleRNDF(Vec2 midPoint, flt radius, std::string const & fname);
	boost::shared_ptr<RNDFData> generateEightRNDF(Vec2 crossingPoint, flt radius, flt swivel, flt rotAngle, std::string const & fname);
	boost::shared_ptr<RNDFData> generateMultilaneRNDF(Vec2 midPoint, flt segmentLength, unsigned int lanes, std::string const & fname);
};

}
}
}
}
}
