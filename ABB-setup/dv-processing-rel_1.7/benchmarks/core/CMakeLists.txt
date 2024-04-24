ADD_EXECUTABLE(benchmark-event-store event_store.cpp)

TARGET_LINK_LIBRARIES(benchmark-event-store PRIVATE dv::processing benchmark::benchmark)
