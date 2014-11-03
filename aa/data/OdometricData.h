#ifndef ODOMETRICDATA_H
#define ODOMETRICDATA_H

#include <core/TimedData.h>
namespace aa
{

namespace data
{

class OdometricData
{
public:
	OdometricData() : leftTicksAccumulated(0), rightTicksAccumulated(0), leftTicksAccumulated2(0), rightTicksAccumulated2(0) {}
	OdometricData(int left, int right) : leftTicksAccumulated2(0), rightTicksAccumulated2(0) {
		leftTicksAccumulated = left;
		rightTicksAccumulated = right;
	}

	OdometricData(int left, int right, int left2, int right2) {
		leftTicksAccumulated = left;
		rightTicksAccumulated = right;
		leftTicksAccumulated2 = left2;
		rightTicksAccumulated2 = right2;

	}

	int leftTicksAccumulated;
	int rightTicksAccumulated;
	int leftTicksAccumulated2;
	int rightTicksAccumulated2;

	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & leftTicksAccumulated;
		ar & rightTicksAccumulated;
		ar & leftTicksAccumulated2;
		ar & rightTicksAccumulated2;
	}
};


typedef TimedData<aa::data::OdometricData> TimedOdometricData;

}
}

extern template class TimedData<aa::data::OdometricData>;


#include <rtt/Port.hpp>
namespace RTT
{
extern template class InputPort< aa::data::TimedOdometricData >;
extern template class OutputPort< aa::data::TimedOdometricData >;

}


#endif // ODOMETRICDATA_H
