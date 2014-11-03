SET(PROJECT Simulator)

SET(QT_REQUIRED TRUE)
SET(QT_USE_QTOPENGL TRUE)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

add_module(Simulator
	AutoSimulant.cpp
	DummyController.cpp
	DisplaySimulator.cpp
	DisplaySimulatorControl.cpp
	CarModel.cpp
	ScannerCamera.cpp
	SimObject.cpp
	Simulator.cpp
	Simulant.cpp
	SimpleBackend.cpp
	SimulatorEngine.cpp
LINK_LIBRARIES Data Rndf EgoState CarState vnl DataObstacle ShortestPath
)
	

