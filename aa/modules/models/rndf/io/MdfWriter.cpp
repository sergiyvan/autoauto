#include "MdfWriter.h"
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

MdfWriter::MdfWriter()
{
}

MdfWriter::~MdfWriter()
{
}

//---------------------------------------------------------------------------
void MdfWriter::writedownSpeedlimits()
{
	mOut << "speed_limits" << std::endl;
	mOut << "num_speed_limits\t" << mMdfData->speedlimits.size() << std::endl;

	for (unsigned int s = 0; s < mMdfData->speedlimits.size(); ++s) {
		mOut << (s + 1) << '\t' << mMdfData->speedlimits[s].min_speed << '\t' << mMdfData->speedlimits[s].max_speed << std::endl;
	}

	mOut << "end_speed_limits" << std::endl;
}

void MdfWriter::writedownCheckpoints()
{
	mOut << "checkpoints" << std::endl;
	mOut << "num_checkpoints\t" << mMdfData->checkpoints.size() << std::endl;

	for (unsigned int c = 0; c < mMdfData->checkpoints.size(); ++c) {
		mOut <<  mMdfData->checkpoints[c] << std::endl;
	}

	mOut << "end_checkpoints" << std::endl;
}

void MdfWriter::writedownMdf()
{
	mOut << "MDF_name\t"	<< mMdfData->filename << std::endl;
	mOut << "RNDF\t"	<< mMdfData->rndf_name << std::endl;
	mOut << "format_version\t" << mMdfData->format_version  << std::endl;
	mOut << "creation_date\t" << mMdfData->creation_date << std::endl;
	writedownCheckpoints();
	writedownSpeedlimits();
	mOut << "end_file" << std::endl;
}

//---------------------------------------------------------------------------
void MdfWriter::saveMdf(boost::shared_ptr<MdfData> const & mdfData, std::string const & fname)
{
	mMdfData = mdfData;
	mOut.open(fname.c_str());
	writedownMdf();
	mMdfData.reset();
}

}


}


}


}


}



