ADD_EXECUTABLE(dv-list-devices list-devices.cpp)

TARGET_LINK_LIBRARIES(dv-list-devices PRIVATE dv::processing CLI11::CLI11)

INSTALL(TARGETS dv-list-devices DESTINATION ${CMAKE_INSTALL_BINDIR})
