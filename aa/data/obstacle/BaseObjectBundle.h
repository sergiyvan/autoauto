#pragma once
/**
 * \file BaseObjectBundle.h
 * \brief Definition of a bundle of BaseObjects
 *		It gives only one bundle for each sensortype.
 * \author Manuel Schwabe
 */

#include "BaseObject.h"

#include <vector>
#include <core/TimedData.h>
#include <rtt/Port.hpp>
#include <util/PooledObjectTemplate.h>


namespace aa
{
namespace data
{
namespace obstacle
{

class BaseObjectBundle
{
public:
	typedef boost::shared_ptr<util::ScannerSystem> ScannerSystemPtr;

	BaseObjectBundle();
	BaseObjectBundle(ScannerSystemPtr const scanner);
	~BaseObjectBundle();

	// Selectors (getters):
	ScannerSystemPtr const scannerSystem() const {
		return mScannerSystem;
	}
	BaseObject & object(uint i) {
		assert(i < mObjects.size());
		return mObjects.at(i);
	}
	// Modifiers (setters):
	void setScannerSystem(ScannerSystemPtr scanner) {
		mScannerSystem = scanner;
	}
	void addObject(BaseObject const & o) {
		mObjects.push_back(o);
	}
	void addObjectBundle(BaseObjectBundle & bundle);

	// Functions:
	void clearBundle();

	uint size() const {
		return mObjects.size();
	}
	std::string toString() const;

protected:
	// Functions:

	// Attributes:

private:
	std::vector<BaseObject> mObjects;
	ScannerSystemPtr mScannerSystem;

};


}
}
}


typedef aa::data::obstacle::BaseObjectBundle BaseObjectBundle;
TIMEDPOOLEDOBJECT(BaseObjectBundle, 8)

std::ostream & operator<< (std::ostream & lhs, const aa::data::obstacle::BaseObjectBundle & ob);
RTT::Logger & operator<<(RTT::Logger & lhs , const aa::data::obstacle::BaseObjectBundle & ob);

namespace RTT
{
extern template class InputPort<TimedBaseObjectBundle_ptr>;
extern template class OutputPort<TimedBaseObjectBundle_ptr>;
}
