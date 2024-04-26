CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

PROJECT(
	point-cloud-display-sample
	VERSION 0.1.0
	LANGUAGES C CXX)

SET(CMAKE_CXX_STANDARD 20)

IF(NOT TARGET dv::processing)
	FIND_PACKAGE(dv-processing 1.7.0 REQUIRED)
ENDIF()
IF(NOT TARGET CLI11::CLI11)
	FIND_PACKAGE(CLI11 CONFIG REQUIRED)
ENDIF()

FIND_PACKAGE(PCL 1.8)

IF(PCL_FOUND)
	ADD_EXECUTABLE(point-cloud-display point-cloud-display.cpp)

	TARGET_INCLUDE_DIRECTORIES(point-cloud-display PRIVATE ${PCL_INCLUDE_DIRS})
	TARGET_LINK_DIRECTORIES(point-cloud-display PRIVATE ${PCL_LIBRARY_DIRS})

	TARGET_LINK_LIBRARIES(point-cloud-display PRIVATE dv::processing CLI11::CLI11 ${PCL_LIBRARIES})
ELSE()
	MESSAGE(WARNING "Trying to build point cloud sample, but libpcl is not found, disabling the target.")
ENDIF()
