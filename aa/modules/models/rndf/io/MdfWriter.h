#pragma once
#include <fstream>
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

class MdfData;

class MdfWriter
{
public:
	MdfWriter();
	~MdfWriter();
	void saveMdf(boost::shared_ptr<MdfData> const & mdfData, std::string const & fname);

private:
	void writedownSpeedlimits();
	void writedownCheckpoints();
	void writedownMdf();

	boost::shared_ptr<MdfData> mMdfData;
	std::ofstream mOut;
};

}

}

}

}

}

