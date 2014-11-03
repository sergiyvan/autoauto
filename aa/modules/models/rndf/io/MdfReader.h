#pragma once
#include "MdfData.h"
#include <boost/shared_ptr.hpp>

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

class MdfReader
{
public:
	MdfReader();
	~MdfReader();

	boost::shared_ptr<MdfData> loadMdf(std::string const & fname);

private:
	boost::shared_ptr<MdfData> mdfData;
};

}

}

}

}

}

