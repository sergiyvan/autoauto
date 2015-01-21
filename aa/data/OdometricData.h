#ifndef ODOMETRICDATA_H
#define ODOMETRICDATA_H

#include <core/TimedData.h>
#include <math/Types.h>

namespace aa
{

namespace data
{

class OdometricData
{
public:
    OdometricData() : mTravelledDistance(0), mAngularDisplacement(0) {}

    OdometricData(math::flt dist, math::flt angle) {
        mTravelledDistance = dist;
        mAngularDisplacement = angle;
	}

    math::flt mTravelledDistance;
    math::flt mAngularDisplacement;

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
