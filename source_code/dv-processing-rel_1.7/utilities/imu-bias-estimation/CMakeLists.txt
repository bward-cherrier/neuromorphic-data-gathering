ADD_EXECUTABLE(dv-imu-bias-estimation imu-bias-estimation.cpp)

TARGET_LINK_LIBRARIES(dv-imu-bias-estimation PRIVATE dv::processing CLI11::CLI11)

INSTALL(TARGETS dv-imu-bias-estimation DESTINATION ${CMAKE_INSTALL_BINDIR})
