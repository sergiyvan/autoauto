#include "OdometricData.h"


template class TimedData<aa::data::OdometricData>;

namespace RTT
{
template class InputPort< aa::data::TimedOdometricData >;
template class OutputPort< aa::data::TimedOdometricData >;
}


