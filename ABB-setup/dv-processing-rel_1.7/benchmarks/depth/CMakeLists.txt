ADD_EXECUTABLE(benchmark-disparity disparity.cpp)

TARGET_LINK_LIBRARIES(benchmark-disparity PRIVATE dv::processing benchmark::benchmark)

ADD_EXECUTABLE(benchmark-sparse-block-matcher sparse-block-matcher.cpp)

TARGET_LINK_LIBRARIES(benchmark-sparse-block-matcher PRIVATE dv::processing benchmark::benchmark)
