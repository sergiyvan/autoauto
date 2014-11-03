add_data(Controller
	Plan.cpp
	ComfortSettings.cpp
	data/ControllerData.cpp
LINK_LIBRARIES
	StateMachine
	EgoState
)

add_module(ControllerTasks
	AbstractController.cpp
    ControllerPassat.cpp
	SteerController.cpp
LINK_LIBRARIES
        CarState
        Controller
	Data
	DataObstacle
        EgoStateFilter
        Math
	Passat
        Rndf
)

