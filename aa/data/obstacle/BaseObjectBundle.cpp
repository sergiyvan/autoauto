/*!
 * \file "BaseObjectBundle.h"
 * \brief definition of a Bundel of BaseObjects
give only one bundle for each sensortype
 * \author Manuel Schwabe
 */

#include <aa/data/obstacle/BaseObjectBundle.h>
#include "util/ScannerSystem.h"


std::ostream & operator <<(std::ostream & lhs, const BaseObjectBundle & ob)
{
	lhs << ob.toString();
	return lhs;
}
RTT::Logger & operator <<(RTT::Logger & lhs, const BaseObjectBundle & ob)
{

	lhs << ob.toString();
	return lhs;
}


namespace aa
{
namespace data
{
namespace obstacle
{


BaseObjectBundle::BaseObjectBundle()

{
}
BaseObjectBundle::BaseObjectBundle(ScannerSystemPtr scanner)
{
	mScannerSystem = scanner;
}

BaseObjectBundle::~BaseObjectBundle()
{
}
void BaseObjectBundle::addObjectBundle(BaseObjectBundle & bundle)
{
	for (int i = 0; i < bundle.size(); i++) {
		this->addObject(bundle.object(i));
	}

	for (int i = 0; i < bundle.scannerSystem()->size(); i++) {
		mScannerSystem->push_back(bundle.scannerSystem()->at(i));
	}

}

void BaseObjectBundle::clearBundle()
{
	mObjects.clear();
	mScannerSystem.reset();
}

std::string BaseObjectBundle::toString() const
{
	std::stringstream s;
	s << "BaseObjectBundle: numberOfObjects: " << size() << "\n"
	  << "Scannersystem: ";

	for (int i = 0; i < mScannerSystem->size(); i++) {
		s << mScannerSystem->at(i).model() << " ";
	}

	s << "\n";

	return s.str();
}


}
}
}

namespace RTT
{
template class InputPort<TimedBaseObjectBundle_ptr>;
template class OutputPort<TimedBaseObjectBundle_ptr>;
}
