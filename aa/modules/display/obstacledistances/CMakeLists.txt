add_module(DisplayObstacleDistances
	DisplayObstacleDistances.cpp
    Object.cpp
LINK_LIBRARIES
	Data
	DataObstacle
	Gui
	EgoState
	Math
	PlotData
	vgl_algo
	glut
)
