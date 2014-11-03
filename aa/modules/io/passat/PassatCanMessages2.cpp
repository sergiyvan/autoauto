#include "PassatCanMessages.h"
#include <boost/preprocessor/list.hpp>

#define SUBSET BOOST_PP_LIST_REST_N(20, PASSAT_CAN_MESSAGES_TYPES)

BOOST_PP_LIST_FOR_EACH(PASSAT_CAN_MESSAGE_INSTANTIATE, _, SUBSET)
