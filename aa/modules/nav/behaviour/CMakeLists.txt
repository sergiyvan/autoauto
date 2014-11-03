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
