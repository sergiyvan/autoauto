SET(SRC
	ObstacleData.cpp
	OdometricData.cpp
	RawObstacleBoundingBoxes.cpp
)

ADD_LIBRARY(AAData SHARED ${SRC})
TARGET_LINK_LIBRARIES(AAData Math Util ${OROCOS_LIBRARIES})

INSTALL(TARGETS	AAData
	LIBRARY DESTINATION lib
	)

ADD_TO_XML(AAData ${SRC})


add_subdirectory(obstacle)

