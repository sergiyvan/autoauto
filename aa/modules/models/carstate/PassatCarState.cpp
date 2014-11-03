#include "PassatCarState.h"

#include <util/MqPortType.h>

namespace aa
{
namespace modules
{
namespace models
{

namespace carstate
{

//REGISTER_MQ_PORT_TYPE(TimedPassatCarState);
}


}


}


}



namespace RTT
{
template class InputPort<aa::modules::models::carstate::TimedPassatCarState>;
template class OutputPort<aa::modules::models::carstate::TimedPassatCarState>;
}
