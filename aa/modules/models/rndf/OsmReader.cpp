#include "OsmReader.h"
#include <rtt/Logger.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#include <xercesc/sax2/Attributes.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/XMLDouble.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include "io/Waypoint.h"

using namespace xercesc;
using namespace ::math;

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

using io::RNDFData;
using io::Waypoint;

OSMReader::OSMReader()
	: mState(UNDEF)
{}

OSMReader::~OSMReader()
{}

bool OSMReader::loadMap(RNDFGraph & graph, std::string const & fname)
{
	bool success = false;

	try {
		XMLPlatformUtils::Initialize();
	}
	catch (XMLException const & toCatch) {
		char * message = XMLString::transcode(toCatch.getMessage());
		RTT::Logger::log(RTT::Logger::Error) << "Error during initialization! :\n";
		RTT::Logger::log(RTT::Logger::Error) << "Exception message is: \n"
											 << message << RTT::Logger::endl;
		XMLString::release(&message);
		return false;
	}

	pRNDFData = boost::shared_ptr<RNDFData>(new RNDFData());
	mWaypoints.clear();
	mSegmentId = 0;
	std::auto_ptr<SAX2XMLReader> parser(XMLReaderFactory::createXMLReader());
	parser->setFeature(XMLUni::fgSAX2CoreValidation, false);
	parser->setFeature(XMLUni::fgSAX2CoreNameSpaces, false);   // optional

	parser->setContentHandler(this);
	parser->setErrorHandler(this);

	try {
		parser->parse(fname.c_str());
		graph.buildGraph(pRNDFData);
		success = true;
	}
	catch (const XMLException & toCatch) {
		char * message = XMLString::transcode(toCatch.getMessage());
		RTT::Logger::log(RTT::Logger::Error) << "Exception message is: \n"
											 << message << RTT::Logger::endl;
		XMLString::release(&message);
	}
	catch (const SAXParseException & toCatch) {
		char * message = XMLString::transcode(toCatch.getMessage());
		RTT::Logger::log(RTT::Logger::Error) << "Exception message is: \n"
											 << message << RTT::Logger::endl;
		XMLString::release(&message);
	}
	catch (...) {
		RTT::Logger::log(RTT::Logger::Error) << "Unexpected Exception" << RTT::Logger::endl;
	}

	RTT::Logger::log(RTT::Logger::Debug) << "Segments: " << pRNDFData->segments.size() << RTT::Logger::endl;
	pRNDFData.reset();
	mWaypoints.clear();

	return success;
}

typedef std::basic_string<XMLCh> XercesString;

XercesString toString(char const * const cstr)
{
	XMLCh * pStr = XMLString::transcode(cstr);
	XercesString str(pStr);
	XMLString::release(&pStr);
	return str;
}

std::string toNativeString(XMLCh const * const cstr)
{
	char * pStr = XMLString::transcode(cstr);
	std::string str(pStr);
	XMLString::release(&pStr);
	return str;
}

void OSMReader::startElement(XMLCh const * const uri, XMLCh const * const localname, XMLCh const * const qname, Attributes const & attrs)
{
	static XercesString const sNode = toString("node");
	static XercesString const sWay = toString("way");
	static XercesString const sTag = toString("tag");
	static XercesString const sNd = toString("nd");
	static XercesString const sId = toString("id");
	static XercesString const sRef = toString("ref");
	static XercesString const sLat = toString("lat");
	static XercesString const sLon = toString("lon");
	static XercesString const sK = toString("k");
	static XercesString const sV = toString("v");

	if (localname == sNode) {
		mState = NODE;
		Waypoint wp;
// 		unsigned int const nAttrs = attrs.getLength();
		bool success = XMLString::textToBin(attrs.getValue(sId.c_str()), wp.id);
		XMLDouble lat(attrs.getValue(sLat.c_str()));
		XMLDouble lon(attrs.getValue(sLon.c_str()));
		wp.latitude = lat.getValue();
		wp.longitude = lon.getValue();
		mWaypoints[wp.id] = wp;
// 		std::cout << "Node: " << wp.id << " " << wp.latitude << ", " << wp.longitude << std::endl;
	}
	else if (localname == sWay) {
		mState = WAY;
		mSegment.id = ++mSegmentId;
// 		bool success = XMLString::textToBin(attrs.getValue(sId.c_str()), wp.id);
		mLane.waypoints.clear();
		mLane.segment = mSegment.id;
		mLane.id = 1;
		mSegment.name = "";
		mOneWay = false;
		mLanes = 0;
		mWaypointId = 0;
		mSegment.lanes.clear();
	}
	else if (localname == sTag) {
		if (WAY == mState || HIGHWAY == mState) {
			std::string const key(toNativeString(attrs.getValue(sK.c_str())));
			std::string const value(toNativeString(attrs.getValue(sV.c_str())));

			if (key == "highway") {
				mState = HIGHWAY;
			}
			else if (key == "name") {
				mSegment.name = value;
			}
			else if (key == "maxspeed") {
			}
			else if (key == "oneway") {
				mOneWay = true;
			}
			else if (key == "lanes") {
				if (!XMLString::textToBin(attrs.getValue(sV.c_str()), mLanes)) {
					mLanes = 0;
				}
			}
		}
	}
	else if (localname == sNd) {
		unsigned int id = 0;

		if (XMLString::textToBin(attrs.getValue(sRef.c_str()), id)) {
			waypoint_map_type::const_iterator pos = mWaypoints.find(id);

			if (mWaypoints.end() != pos) {
// 				std::cout << "Node: " << mWaypointId << " " << pos->second.latitude << ", " << pos->second.longitude << std::endl;
				if (mLane.waypoints.empty()) {
					mLane.addWaypoint(++mWaypointId, pos->second.latitude, pos->second.longitude);
				}
				else {
					Waypoint const & prev = mLane.waypoints.back();

					if (prev.longitude != pos->second.longitude || prev.latitude != pos->second.latitude) {
						mLane.addWaypoint(++mWaypointId, pos->second.latitude, pos->second.longitude);
					}
					else {
						std::cout << "Duplicate waypoint: " << id << std::endl;
					}
				}
			}
		}
	}
}

void OSMReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{
	static XercesString const sWay = toString("way");

	if (localname == sWay) {
		if (HIGHWAY == mState) {
// 			bool success = XMLString::textToBin(attrs.getValue(sId.c_str()), wp.id);
			mSegment.lanes.push_back(mLane);
			pRNDFData->segments.push_back(mSegment);
		}
		else {	// Not a street
			--mSegmentId;
		}

		mSegment.lanes.clear();
		mLane.waypoints.clear();
	}
}

}

}

}

}

