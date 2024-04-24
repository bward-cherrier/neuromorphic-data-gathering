ADD_EXECUTABLE(benchmark-kd-tree kd_tree.cpp)

TARGET_LINK_LIBRARIES(benchmark-kd-tree PRIVATE dv::processing benchmark::benchmark)
