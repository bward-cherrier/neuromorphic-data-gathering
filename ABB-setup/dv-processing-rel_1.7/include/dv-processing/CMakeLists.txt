# Version header, contains version macros.
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/version.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/version.hpp @ONLY)

ADD_LIBRARY(processing INTERFACE)
ADD_LIBRARY(dv::processing ALIAS processing)

TARGET_INCLUDE_DIRECTORIES(processing INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
												$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

TARGET_LINK_LIBRARIES(
	processing
	INTERFACE ${SYSTEM_THREAD_LIBS}
			  ${OpenCV_LIBS}
			  Boost::boost
			  Boost::nowide
			  Eigen3::Eigen
			  libcaer::caer
			  fmt::fmt
			  ${NETWORKING_DEPENDENCIES})

TARGET_COMPILE_FEATURES(processing INTERFACE cxx_std_20)

IF(OS_MACOS)
	# Eigen requires alloca, dl_info also used.
	TARGET_COMPILE_DEFINITIONS(processing INTERFACE -D_DARWIN_C_SOURCE=1)
ENDIF()

IF(CC_MSVC)
	# Disable "min" and  "max" macros on systems compiling with MSVC
	TARGET_COMPILE_DEFINITIONS(processing INTERFACE -DNOMINMAX=ON)
ENDIF()

# Compression support: LZ4 and ZSTD
FIND_PACKAGE(lz4 QUIET)
IF(lz4_FOUND)
	# Found CMake configuration.
	IF(TARGET LZ4::lz4_shared)
		TARGET_LINK_LIBRARIES(processing INTERFACE LZ4::lz4_shared)
	ELSEIF(TARGET LZ4::lz4_static)
		TARGET_LINK_LIBRARIES(processing INTERFACE LZ4::lz4_static)
	ELSE()
		# Fallback for VCPKG custom target name.
		TARGET_LINK_LIBRARIES(processing INTERFACE lz4::lz4)
	ENDIF()
ELSE()
	# Search for external libraries with pkg-config.
	INCLUDE(FindPkgConfig)

	PKG_CHECK_MODULES(liblz4 IMPORTED_TARGET liblz4)

	IF(liblz4_FOUND)
		# Found pkg-config file.
		TARGET_LINK_LIBRARIES(processing INTERFACE PkgConfig::liblz4)
	ELSE()
		# Fall back to library path.
		FIND_LIBRARY(LIBLZ4 lz4)
		IF(${LIBLZ4} STREQUAL "LIBLZ4-NOTFOUND")
			MESSAGE(FATAL_ERROR "lz4 not found, required for I/O compression.")
		ENDIF()

		MESSAGE(STATUS "lz4 library found at: ${LIBLZ4}")
		TARGET_LINK_LIBRARIES(processing INTERFACE lz4)
	ENDIF()
ENDIF()

FIND_PACKAGE(zstd QUIET)
IF(zstd_FOUND)
	# Found CMake configuration.
	IF(TARGET zstd::libzstd_shared)
		TARGET_LINK_LIBRARIES(processing INTERFACE zstd::libzstd_shared)
	ELSE()
		TARGET_LINK_LIBRARIES(processing INTERFACE zstd::libzstd_static)
	ENDIF()
ELSE()
	# Search for external libraries with pkg-config.
	INCLUDE(FindPkgConfig)

	PKG_CHECK_MODULES(libzstd IMPORTED_TARGET libzstd)

	IF(libzstd_FOUND)
		# Found pkg-config file.
		TARGET_LINK_LIBRARIES(processing INTERFACE PkgConfig::libzstd)
	ELSE()
		# Fall back to library path.
		FIND_LIBRARY(LIBZSTD zstd)
		IF(${LIBZSTD} STREQUAL "LIBZSTD-NOTFOUND")
			MESSAGE(FATAL_ERROR "zstd not found, required for I/O compression.")
		ENDIF()

		MESSAGE(STATUS "zstd library found at: ${LIBZSTD}")
		TARGET_LINK_LIBRARIES(processing INTERFACE zstd)
	ENDIF()
ENDIF()

IF(OS_UNIX
   OR OS_LINUX
   OR OS_MACOS)
	# libdl needed for Boost::stacktrace on some platforms like Linux and MacOS X.
	TARGET_LINK_LIBRARIES(processing INTERFACE dl)
ENDIF()

# Boost::stacktrace can use libbacktrace for better debug output. The addr2line stacktrace does not work:
# https://github.com/boostorg/stacktrace/issues/97
FIND_LIBRARY(LIBBACKTRACE backtrace)
IF(NOT
   ${LIBBACKTRACE}
   STREQUAL
   "LIBBACKTRACE-NOTFOUND")
	MESSAGE(STATUS "Boost::stacktrace will use libbacktrace.")

	TARGET_LINK_LIBRARIES(processing INTERFACE backtrace)
	TARGET_COMPILE_DEFINITIONS(processing INTERFACE BOOST_STACKTRACE_USE_BACKTRACE)
ELSE()
	MESSAGE(STATUS "Boost::stacktrace will provide basic info.")

	TARGET_COMPILE_DEFINITIONS(processing INTERFACE BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED)
ENDIF()

INSTALL(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	FILES_MATCHING
	PATTERN "*.hpp"
	PATTERN "*.h")

INSTALL(TARGETS processing EXPORT "${PROJECT_NAME}-targets")

# CMake targets and files to support FIND_PACKAGE().
IF(BUILD_CONFIG_VCPKG)
	SET(CMAKE_EXPORT_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
ELSE()
	SET(CMAKE_EXPORT_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
ENDIF()

INSTALL(
	EXPORT "${PROJECT_NAME}-targets"
	DESTINATION ${CMAKE_EXPORT_DESTINATION}
	NAMESPACE "dv::")

INCLUDE(CMakePackageConfigHelpers)

# Generate a config file so other CMake project can include it.
CONFIGURE_PACKAGE_CONFIG_FILE(
	${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
	INSTALL_DESTINATION ${CMAKE_EXPORT_DESTINATION}
	PATH_VARS CMAKE_EXPORT_DESTINATION)

# Generate a version file; IMPORTANT: Compatibility setting is SameMajorVersion, so CMake will assume that any version
# requirement with same major version value will be compatible.
WRITE_BASIC_PACKAGE_VERSION_FILE(
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion)

# Install CMake config files.
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
			  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
			  ${PROJECT_SOURCE_DIR}/cmake/modules/inivation-setup.cmake DESTINATION ${CMAKE_EXPORT_DESTINATION})

# Generate pkg-config file
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}.pc.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
