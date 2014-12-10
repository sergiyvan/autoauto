add_data(ShortestPath
	ShortestPath.cpp
LINK_LIBRARIES
	Rndf
)

add_module(CarFollowingBehaviour
	CarFollowingBehaviour.cpp
LINK_LIBRARIES
	AAData
	Controller
	EgoState
)

add_module(SimplePlanGenerator
        SimplePlanGenerator.cpp
LINK_LIBRARIES
        AAData
        Controller
)

add_module(AStarPlanGenerator
        AStarPlanGenerator.cpp
LINK_LIBRARIES
        AAData
        Controller
        DataObstacle
)

add_module(DisplayAStarWaypoints
        DisplayAStarWaypoints.cpp
LINK_LIBRARIES
        Gui
)
