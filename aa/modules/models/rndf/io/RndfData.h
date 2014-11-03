#pragma once

#include <string>
#include <vector>
#include "Segment.h"
#include "aa/modules/models/rndf/io/Zone.h"
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

class RNDFData
{
public:
	RNDFData();
	RNDFData(const RNDFData & r);
	~RNDFData();

	RNDFData & operator=(const RNDFData & r);

	void addSegment(uint segID);
	void addZone(uint zonID);

	std::vector<Segment>	segments;
	std::vector<Zone>	zones;
	std::string			filename;
	std::string			format_version;
	std::string			creation_date;
};

}

}

}

}

}

