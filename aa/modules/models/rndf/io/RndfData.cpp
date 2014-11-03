#include "RndfData.h"

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

RNDFData::RNDFData()
{}

RNDFData::~RNDFData()
{}

RNDFData::RNDFData(const RNDFData & r)
	: segments(r.segments)
	, zones(r.zones)
	, filename(r.filename)
	, format_version(r.format_version)
	, creation_date(r.creation_date)
{}

RNDFData & RNDFData::operator=(const RNDFData & r)
{
	if (this == &r) {
		return *this;
	}

	segments           = r.segments;
	zones              = r.zones;
	filename           = r.filename;
	format_version     = r.format_version;
	creation_date      = r.creation_date;

	return *this;
}

void RNDFData::addSegment(uint id)
{
	if (segments.size() < id) {
		segments.resize(id);
	}

	segments[id - 1].id = id;
}

void RNDFData::addZone(uint id)
{
	uint zid = id - segments.size();

	if (zones.size() < zid) {
		zones.resize(zid);
	}

	zones[zid - 1].id = id;
}

}


}


}


}


}


