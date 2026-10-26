# Kinect SDK v1.8 configuration
set(KINECT_SDK_DIR "$ENV{KINECTSDK10_DIR}")
if(NOT KINECT_SDK_DIR)
    set(KINECT_SDK_DIR "C:/Program Files/Microsoft SDKs/Kinect/v1.8")
endif()

set(HOST_VISION_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/la.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vision_pipeline.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kalman_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/display.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kinect_capture.cpp
)

add_library(host_vision STATIC
    ${HOST_VISION_SOURCES}
)

target_include_directories(host_vision PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    "${KINECT_SDK_DIR}/inc"
)

target_link_directories(host_vision PUBLIC
    "${KINECT_SDK_DIR}/lib/amd64"
)

target_link_libraries(host_vision PUBLIC
    Kinect10
    SDL2main
    SDL2
    m
)

# Main executable
add_executable(intercept_vision
    src/main.c
)

target_link_libraries(intercept_vision PRIVATE
    host_vision
    Kinect10
    SDL2main
    SDL2
    ole32
    m
)

# Test target
add_executable(test_la_kalman
    tests/test_la_kalman.c
)

target_link_libraries(test_la_kalman PRIVATE
    host_vision
    m
)
