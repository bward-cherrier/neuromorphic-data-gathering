CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

# Find Python installation and its NumPy support. Execute before pybind11 to prime CMake caches.
FIND_PACKAGE(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

# External pybind11 checked-out as a submodule
ADD_SUBDIRECTORY(external/pybind11 EXCLUDE_FROM_ALL)

PYBIND11_ADD_MODULE(
	dv_processing
	MODULE
	src/dv_processing.cpp
	external/pybind11_opencv_numpy/ndarray_converter.cpp)

TARGET_COMPILE_FEATURES(dv_processing PRIVATE cxx_std_20)

TARGET_LINK_LIBRARIES(dv_processing PRIVATE dv::processing Python3::Module Python3::NumPy)

INSTALL(TARGETS dv_processing DESTINATION ${Python3_SITEARCH})

MESSAGE(STATUS "Python executable: ${Python3_EXECUTABLE}")
MESSAGE(STATUS "Python installation directory: ${Python3_SITEARCH}")

IF(ENABLE_TESTS)
	FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests)
	FILE(COPY tests/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tests)

	ADD_TEST(
		NAME test-python-bindings
		COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_all_tests.py
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests)

	SET_TESTS_PROPERTIES(test-python-bindings PROPERTIES ENVIRONMENT PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR})
ENDIF()

OPTION(ENABLE_PYTHON_STUBGEN "Generate stubs for python bindings" ON)

IF(ENABLE_PYTHON_STUBGEN)
	# If stubgen is available, generate stubs.
	FIND_PROGRAM(PYBIND_STUBGEN pybind11-stubgen)

	IF(EXISTS ${PYBIND_STUBGEN})
		MESSAGE("-- Found stubgen: ${PYBIND_STUBGEN}")
		MESSAGE("-- Stubs will be generated")

		ADD_CUSTOM_COMMAND(
			TARGET dv_processing
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR} ${PYBIND_STUBGEN} ARGS
					--ignore-invalid=all -o . dv_processing
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
			VERBATIM)

		INSTALL(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dv_processing-stubs DESTINATION ${Python3_SITEARCH})
	ELSE()
		MESSAGE(WARNING "-- Python stubgen not found, stubs will not be generated")
	ENDIF()
ENDIF()
