#include "MdfData.h"

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

MdfData::MdfData()
{}

MdfData::~MdfData()
{
	checkpoints.clear();
	speedlimits.clear();
}

MdfData::MdfData(MdfData const & m)
{
	filename              = m.filename;
	rndf_name             = m.rndf_name;
	format_version        = m.format_version;
	creation_date         = m.creation_date;
	checkpoints           = m.checkpoints;
	speedlimits           = m.speedlimits;
}

}

}

}

}

}

