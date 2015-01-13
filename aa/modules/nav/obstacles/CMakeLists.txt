add_data(NavObstacles
        InterferingObstacle.cpp
        ObstaclesOnSpline.cpp
LINK_LIBRARIES
)

add_module(ObstacleTacker
        KalmanObstacleTracker.cpp
LINK_LIBRARIES
        DataObstacle
)

