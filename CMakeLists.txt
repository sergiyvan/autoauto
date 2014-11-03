CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(aa)

STRING(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

#Bootstrapping to Core, Afterwards we can use prepared cmake makros from CORE to bind dependencies
IF(EXISTS "${CORE_INCLUDE_DIR}/cmake/DependencyRepositories.cmake")
	if (NOT IS_ABSOLUTE ${CORE_INCLUDE_DIR})
		SET(CORE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/${CORE_INCLUDE_DIR}")
		GET_FILENAME_COMPONENT( CORE_INCLUDE_DIR "${CORE_INCLUDE_DIR}" ABSOLUTE)
	endif()
	INCLUDE("${CORE_INCLUDE_DIR}/cmake/DependencyRepositories.cmake") 
ELSE()
	MESSAGE( FATAL_ERROR "Can not find DependencyRepositories.cmake Have you set the 'CORE_INCLUDE_DIR' correcly?")
ENDIF()

ADD_REPO_DEPENDENCY(CORE)

SET(AA_GUI ON)

INCLUDE(InitCompiler)
INCLUDE(InitLibraries)
INCLUDE(SymbolicLinks)

SET(aa_TARGET_RUNTIME_DIR "${CMAKE_BINARY_DIR}/bin")

SYMBOLIC_LINK_DIR(${PROJECT_NAME} xml)
SYMBOLIC_LINK_DIR(${PROJECT_NAME} resources)
SYMBOLIC_LINKS_FRAMEWORK_BINARYS(remotecontrolcentre)
SYMBOLIC_LINKS_FRAMEWORK_BINARYS(runaaxml)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/PluginPath.xml.in ${CMAKE_BINARY_DIR}//PluginPath.xml)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/ResourcePath.xml.in ${CMAKE_BINARY_DIR}//ResourcePath.xml)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/path.sh.in ${CMAKE_BINARY_DIR}/path.sh)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/aaconfig.h.in ${CMAKE_BINARY_DIR}/aaconfig.h)

# needed with boost1.54 (and greater?) to compile in C++11 mode
ADD_DEFINITIONS(-DBOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS)

INCLUDE(AutoModule)

add_subdirectory(${PROJECT_NAME})
