add_module(DisplayObstacles
        DisplayBaseObstacles.cpp
	DisplayObjects.cpp
#        DisplayObstacles.cpp
        DisplayObstaclesBase.cpp
	ObstacleColorScheme.cpp
LINK_LIBRARIES
	Gui
	glut
	Math
	Data
	DataObstacle
#	DataObstacleRenderer
	EgoStateFilter
	EgoState
	DisplayUtilities
)
