FILE(COPY calibration_files DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

DV_ADD_TEST(calibration-set)
DV_ADD_TEST(camera-geometry CLI11::CLI11)
DV_ADD_TEST(camera-distortion)
DV_ADD_TEST(stereo-geometry CLI11::CLI11)
