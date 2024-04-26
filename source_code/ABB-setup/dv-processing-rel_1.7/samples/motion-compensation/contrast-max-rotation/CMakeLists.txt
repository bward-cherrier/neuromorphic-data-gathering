CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

PROJECT(
	contrast-maximization-rotation-sample
	VERSION 0.1.0
	LANGUAGES C CXX)

SET(CMAKE_CXX_STANDARD 20)

IF(NOT TARGET dv::processing)
	FIND_PACKAGE(dv-processing 1.7.0 REQUIRED)
ENDIF()
IF(NOT TARGET CLI11::CLI11)
	FIND_PACKAGE(CLI11 CONFIG REQUIRED)
ENDIF()

ADD_EXECUTABLE(contrast-maximization-rotation contrast-maximization-rotation.cpp)

TARGET_LINK_LIBRARIES(contrast-maximization-rotation PRIVATE dv::processing CLI11::CLI11)
