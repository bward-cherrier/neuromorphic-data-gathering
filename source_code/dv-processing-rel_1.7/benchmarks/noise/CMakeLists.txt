ADD_EXECUTABLE(benchmark-noise-filtering noise-filtering.cpp)

TARGET_LINK_LIBRARIES(benchmark-noise-filtering PRIVATE dv::processing benchmark::benchmark)
