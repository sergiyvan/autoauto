#include "MdfReader.h"
#include <boost/spirit/include/classic.hpp>
#include <boost/bind.hpp>

using namespace boost::spirit::classic;
using namespace boost::spirit;
using namespace boost;

typedef char char_t;
typedef file_iterator <char_t>  iterator_t;

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

struct mdf_grammar
		: public grammar<mdf_grammar> {
	typedef mdf_grammar self_t;
	friend struct definition;

	mdf_grammar(MdfData * _mdfData)
		: mdfData(_mdfData)
	{	}

	template <typename ScannerT>
	struct definition {
		definition(mdf_grammar const & self) {
			opt = (comment | blank_p);
			speedlimit = uint_p[bind(&self_t::setSpeedSegment, cref(self), _1)] >> *blank_p >> uint_p[bind(&self_t::setSegmentLower, cref(self), _1)] >> *blank_p >> uint_p[bind(&self_t::setSegmentUpper, cref(self), _1)] >> *opt >> eol_p;
			checkpoint = uint_p[bind(&self_t::addCheckpoint, cref(self), _1)];
			comment = comment_p("/*", "*/");
			str = *graph_p;
			MDFtoken =
				(strlit<>("MDF_name") >> *blank_p >> str[bind(&self_t::setMDFName, cref(self), _1, _2)])
				| (strlit<>("RNDF")  >> *blank_p >> str[bind(&self_t::setRNDFName, cref(self), _1, _2)])
				| (strlit<>("format_version")  >> *blank_p >> str[bind(&self_t::setFormatVersion, cref(self), _1, _2)])
				| (strlit<>("creation_date")  >> *blank_p >> str[bind(&self_t::setCreationDate, cref(self), _1, _2)])
				| (strlit<>("checkpoints")  >> *space_p >> strlit<>("num_checkpoints") >> *blank_p >> uint_p >> *(checkpoint | space_p | opt) >> strlit<>("end_checkpoints"))
				| (strlit<>("speed_limits")  >> *space_p >> strlit<>("num_speed_limits") >> *blank_p >> uint_p >> *space_p >> *(speedlimit | space_p) >> strlit<>("end_speed_limits"));

			baseExpression = *space_p >> *(comment | MDFtoken | space_p) >> end_p;
		}

		rule<ScannerT>
		opt,
		str,
		MDFtoken,
		comment,
		speedlimit,
		checkpoint,
		baseExpression;

		rule<ScannerT> const & start() const {
			return baseExpression;
		}
	};

	void setMDFName(iterator_t first, iterator_t last) const {
		mdfData->filename = std::string(first, last);
	}

	void setRNDFName(iterator_t first, iterator_t last) const {
		mdfData->rndf_name = std::string(first, last);
	}

	void setFormatVersion(iterator_t first, iterator_t last) const {
		mdfData->format_version = std::string(first, last);
	}

	void setCreationDate(iterator_t first, iterator_t last) const {
		mdfData->creation_date = std::string(first, last);
	}

	void addCheckpoint(unsigned int checkpoint) const {
		mdfData->checkpoints.push_back(checkpoint);
	}

	void setSpeedSegment(unsigned int _segment) const {
		segment = _segment - 1;

// 		std::cout << " Speed-limit for segment: " << _segment << std::endl;
		if (_segment > mdfData->speedlimits.size()) {
			mdfData->speedlimits.resize(_segment);
		}
	}

	void setSegmentLower(unsigned int lower) const {
		mdfData->speedlimits[segment].min_speed = lower;
		mdfData->speedlimits[segment].isDefault = false;
	}

	void setSegmentUpper(unsigned int upper) const {
		mdfData->speedlimits[segment].max_speed = upper;
		mdfData->speedlimits[segment].isDefault = false;
	}

	mutable uint segment;
	MdfData * mdfData;
};

MdfReader::MdfReader()
{}

MdfReader::~MdfReader()
{}

//---------------------------------------------------------------------------

boost::shared_ptr<MdfData> MdfReader::loadMdf(std::string const & fname)
{
// 	ifs.open(fname.c_str());
	iterator_t first(fname);

	if (!first) {
		return boost::shared_ptr<MdfData>();
	}

	iterator_t last = first.make_end();

	boost::shared_ptr<MdfData> ptr(new  MdfData);
	mdfData = ptr;

	mdf_grammar g(ptr.get());

	parse_info<iterator_t> info = parse(first, last, g);

	return mdfData;
}

}


}


}


}


}


