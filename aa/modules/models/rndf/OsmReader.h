#pragma once
#include <xercesc/sax2/DefaultHandler.hpp>
#include <tr1/unordered_map>
#include <boost/smart_ptr.hpp>
#include "io/RndfData.h"

namespace aa
{
namespace modules
{
namespace models
{
namespace rndf
{

class OSMReader
	: public xercesc::DefaultHandler
{
public:
	OSMReader();
	~OSMReader();

	bool loadMap(RNDFGraph & graph, std::string const & fname);

	virtual void startElement(XMLCh const * const uri, XMLCh const * const localname, XMLCh const * const qname, xercesc::Attributes const & attrs);
	virtual void endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname);

private:
	enum State {
		UNDEF,
		NODE,
		WAY,
		HIGHWAY
	};

	State mState;
	typedef std::tr1::unordered_map<unsigned int, io::Waypoint> waypoint_map_type;
	waypoint_map_type mWaypoints;
	bool mOneWay;
	uint mLanes;
	uint mSegmentId;
	uint mWaypointId;
	io::Lane mLane;
	io::Segment mSegment;
	boost::shared_ptr<io::RNDFData> pRNDFData;
	RNDFGraph * pGraph;
};

}
}
}
}

