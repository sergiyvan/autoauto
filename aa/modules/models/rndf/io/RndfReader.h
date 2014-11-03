#pragma once
#include "RndfData.h"
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

class RNDFReader
{
public:
	RNDFReader();
	~RNDFReader();

	boost::shared_ptr<RNDFData> loadRNDF(std::string const & fname);
};

}

}

}

}

}

