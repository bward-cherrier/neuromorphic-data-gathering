ADD_EXECUTABLE(benchmark-mean-shift mean_shift.cpp)

TARGET_LINK_LIBRARIES(benchmark-mean-shift PRIVATE dv::processing benchmark::benchmark)
