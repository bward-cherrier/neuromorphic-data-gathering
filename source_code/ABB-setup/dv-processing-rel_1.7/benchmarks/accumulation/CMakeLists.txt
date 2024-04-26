ADD_EXECUTABLE(benchmark-accumulation accumulation.cpp)

TARGET_LINK_LIBRARIES(benchmark-accumulation PRIVATE dv::processing benchmark::benchmark)
