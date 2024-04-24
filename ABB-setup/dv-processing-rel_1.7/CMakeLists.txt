CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

# Project name and version
PROJECT(
	dv-processing
	VERSION 1.7.9
	LANGUAGES C CXX)

# List of available options
OPTION(ENABLE_TESTS "Build unit tests" ON)
OPTION(ENABLE_SAMPLES "Build sample code" OFF)
OPTION(ENABLE_DOC_SAMPLES_BUILD "Build documentation samples" OFF)
OPTION(ENABLE_UTILITIES "Build CLI utilities" ON)
OPTION(ENABLE_BENCHMARKS "Build benchmarking executables" OFF)
OPTION(ENABLE_PYTHON "Build Python bindings" OFF)

# Very specific build option
OPTION(BUILD_CONFIG_VCPKG "Set build environment compatible with VCPKG" OFF)

# C++ standard settings.
SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

# Git submodule automatic update support.
FIND_PACKAGE(Git QUIET)

IF(Git_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
	# Update submodules as needed
	OPTION(GIT_SUBMODULE "Check-out submodules during build" ON)

	IF(GIT_SUBMODULE)
		MESSAGE(STATUS "Git submodule update")

		EXECUTE_PROCESS(
			COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			RESULT_VARIABLE GIT_SUBMOD_RESULT)
		IF(NOT
		   GIT_SUBMOD_RESULT
		   EQUAL
		   "0")
			MESSAGE(
				FATAL_ERROR
					"${GIT_EXECUTABLE} submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please check-out submodules manually."
			)
		ENDIF()
	ENDIF()
ENDIF()

# Basic setup, useful variables, see docs.
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

INCLUDE(inivation-setup)

# Skip compiler compatibility checks when doing the VCPKG build
IF(NOT BUILD_CONFIG_VCPKG)
	# Compiler compatibility testing starts GCC has to be at least version 10
	IF(CC_GCC AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "10.0.0")
		MESSAGE(
			FATAL_ERROR
				"The gcc compiler version is not supported (${CMAKE_CXX_COMPILER_VERSION}), please use gcc compiler of at least version 10.0"
		)
	ENDIF()

	# CLang needs at least version 13
	IF(CC_CLANG AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "13.0.0")
		MESSAGE(
			FATAL_ERROR
				"The clang compiler version is not supported (${CMAKE_CXX_COMPILER_VERSION}), please use clang compiler of at least version 13.0"
		)
	ENDIF()

	# MSVC needs at least version 19.29
	IF(CC_MSVC AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "19.29")
		MESSAGE(
			FATAL_ERROR
				"The MSVC compiler version is not supported (${CMAKE_CXX_COMPILER_VERSION}), please use MSVC compiler of at least version 19.29"
		)
	ENDIF()

	# Others are unknown, so it can lead
	IF(NOT CC_GCC
	   AND NOT CC_CLANG
	   AND NOT CC_MSVC)
		MESSAGE(
			WARNING
				"The compiler you are using was not verified with dv-processing, make sure it supports full C++20 feature-set, otherwise missing C++20 features can prevent the project from compilation."
		)
	ENDIF()
	# Compiler compatibility testing ends
ENDIF()

# Boost support. Search in extra directory for custom version.
IF(EXISTS ${CMAKE_SYSROOT}/opt/inivation/boost/)
	SET(BOOST_ROOT ${CMAKE_SYSROOT}/opt/inivation/boost/)
	SET(Boost_NO_SYSTEM_PATHS ON)
	SET(Boost_NO_BOOST_CMAKE ON)
ENDIF()

FIND_PACKAGE(Boost 1.76 REQUIRED COMPONENTS nowide)

# SSL encryption and certificates support for boost::ASIO.
FIND_PACKAGE(OpenSSL REQUIRED)
SET(NETWORKING_DEPENDENCIES OpenSSL::SSL OpenSSL::Crypto)

# Windows boost::ASIO needs extra libraries for networking support.
IF(OS_WINDOWS)
	LIST(
		APPEND
		NETWORKING_DEPENDENCIES
		wsock32
		ws2_32)
ENDIF()

# Eigen support.
FIND_PACKAGE(Eigen3 3.4.0 REQUIRED)

# OpenCV support.
FIND_PACKAGE(OpenCV REQUIRED)
IF(OpenCV_VERSION VERSION_LESS "4.2.0")
	MESSAGE(FATAL_ERROR "Cannot find OpenCV 4.2.0 or newer.")
ENDIF()

# libfmt support for C++ string formatting.
FIND_PACKAGE(fmt 8.1.1 REQUIRED)

# Direct device access support.
FIND_PACKAGE(libcaer 3.3.14 REQUIRED)

# CLI11 command-line parsing support, used in multiple places.
IF(BUILD_CONFIG_VCPKG)
	# Find CLI11 as an installed package
	FIND_PACKAGE(CLI11 CONFIG REQUIRED)
ELSE()
	# External CLI11 checked-out as a submodule
	ADD_SUBDIRECTORY(tests/external/cli11 EXCLUDE_FROM_ALL)
ENDIF()

IF(BUILD_CONFIG_VCPKG AND CC_MSVC)
	# This VCPKG port's tools use C++20, but Qt6 (via OpenCV4) uses C++17. Assuming that no coroutines are passed
	# between the two. For details: https://devblogs.microsoft.com/oldnewthing/20230111-00/?p=107694
	ADD_DEFINITIONS(-D_ALLOW_COROUTINE_ABI_MISMATCH)
ENDIF()

# Boost::UT (UnitTest) unit testing support. We don't use ADD_SUBDIRECTORY() to include their CMakeLists.txt as it's
# broken due to using CMake CPM, and we don't really need it, so no Boost::ut target is present.
ADD_LIBRARY(boost-ut INTERFACE)

TARGET_INCLUDE_DIRECTORIES(boost-ut INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/tests/external/boost-ut/include>)
TARGET_COMPILE_FEATURES(boost-ut INTERFACE cxx_std_20)
IF(CC_MSVC)
	# Boost::UT module import doesn't compile using MSVC.
	TARGET_COMPILE_DEFINITIONS(boost-ut INTERFACE BOOST_UT_DISABLE_MODULE=ON)
ENDIF()

# Install header files.
ADD_SUBDIRECTORY(include)

IF(ENABLE_TESTS)
	MESSAGE(STATUS "Tests enabled")
	INCLUDE(CTest)
	ENABLE_TESTING()
	ADD_SUBDIRECTORY(tests)
ENDIF()

# Samples build support.
IF(ENABLE_SAMPLES)
	MESSAGE(STATUS "Samples enabled")
	ADD_SUBDIRECTORY(samples)
ENDIF()

# Extra command-line utilities.
IF(ENABLE_UTILITIES)
	MESSAGE(STATUS "CLI utilities enabled")
	ADD_SUBDIRECTORY(utilities)
ENDIF()

IF(ENABLE_BENCHMARKS)
	FIND_PACKAGE(benchmark QUIET)

	IF(benchmark_FOUND)
		MESSAGE(STATUS "Benchmarks enabled")
		ADD_SUBDIRECTORY(benchmarks)
	ELSE()
		MESSAGE(WARNING "Google benchmark library is not available, benchmarking is not going to be built")
	ENDIF()
ENDIF()

IF(ENABLE_PYTHON)
	MESSAGE(STATUS "Python bindings enabled")
	ADD_SUBDIRECTORY(python)

	IF(ENABLE_DOC_SAMPLES_BUILD)
		MESSAGE(STATUS "Python documentation samples are going to be built")
		ADD_SUBDIRECTORY(docs/source/assets/code_samples/python)
	ENDIF()
ENDIF()

IF(ENABLE_DOC_SAMPLES_BUILD)
	MESSAGE(STATUS "C++ documentation samples are going to be built")
	ADD_SUBDIRECTORY(docs/source/assets/code_samples/c++)
ENDIF()

# Support automatic RPM/DEB generation via CPack
SET(CPACK_PACKAGE_NAME ${PROJECT_NAME})
SET(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
SET(CPACK_PACKAGE_RELEASE 1)
SET(CPACK_PACKAGE_CONTACT "support@inivation.com")
SET(CPACK_PACKAGE_VENDOR "iniVation AG")
SET(CPACK_PACKAGE_DESCRIPTION "Generic algorithms for event cameras.")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${CPACK_PACKAGE_DESCRIPTION})
SET(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}.${CMAKE_SYSTEM_PROCESSOR}")
SET(CPACK_GENERATOR "RPM" "DEB")
SET(CPACK_RPM_PACKAGE_AUTOREQ 1)
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS 1)

INCLUDE(CPack)
