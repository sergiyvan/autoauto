add_module(DisplayUtilities
	Util.cpp
LINK_LIBRARIES
	Gui
	glut
	DataGeometry
)


add_module(DisplayRangeCircles.cpp
	DisplayRangeCircles.cpp
LINK_LIBRARIES
	DisplayUtilities
	EgoState
	Gui
)
