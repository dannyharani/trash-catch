cmake_minimum_required(VERSION 3.20)
project(TrashCatch VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic -O3 -mstackrealign -mno-avx -mno-avx2)

include_directories(${CMAKE_SOURCE_DIR}/common/include)

add_subdirectory(host_vision)
