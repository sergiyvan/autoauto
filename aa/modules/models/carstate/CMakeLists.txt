add_data(CarState
	PassatCarState.cpp
LOG
LINK_LIBRARIES
	BaseCarState
)

add_module(CarStateTasks
	CarStateGenerator.cpp
LINK_LIBRARIES
	CarState
	Passat
)

