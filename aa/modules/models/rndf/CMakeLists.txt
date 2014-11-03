add_data(Rndf
	GraphAlgs.cpp
	OsmReader.cpp
	RndfGraph.cpp
	SplineKdTree.cpp
	XingInfo.cpp
	ZoneInfo.cpp
	io/BoundaryType.cpp
	io/Lane.cpp
	io/MdfData.cpp
	io/MdfReader.cpp
	io/MdfWriter.cpp
	io/RndfData.cpp
	io/RndfReader.cpp
	io/RndfWriter.cpp
	io/Segment.cpp
	io/Spot.cpp
	io/Waypoint.cpp
	io/Zone.cpp
LINK_LIBRARIES
	Math
	xerces-c
	vgl
	AAData
	 ${QT_LIBRARIES}
)

add_module(RndfTasks
	io/RndfGenerator.cpp
LINK_LIBRARIES
	Rndf
)

add_data(RndfUtils
	utils.cpp
LINK_LIBRARIES
	Rndf
)


