#pragma once

#include <string>
#include <vector>

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


struct SpeedLimit {
	unsigned int min_speed;	//in mph
	unsigned int max_speed;	//in mph
	bool isDefault;
};

class MdfData
{
public:
	MdfData();
	MdfData(MdfData const & m);
	~MdfData();

	std::string		filename;
	std::string		rndf_name;
	std::string		format_version;
	std::string		creation_date;
	std::vector<unsigned int>	checkpoints;
	std::vector<SpeedLimit>	speedlimits;
};

}


}


}


}


}


