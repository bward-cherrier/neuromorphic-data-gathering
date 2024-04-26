ADD_EXECUTABLE(benchmark-feature-sort feature-sort.cpp)

TARGET_LINK_LIBRARIES(benchmark-feature-sort PRIVATE dv::processing benchmark::benchmark)
