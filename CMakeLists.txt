cmake_minimum_required(VERSION 3.16)

project(VKM VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(Catch2 CONFIG REQUIRED)

include_directories(include)

add_compile_definitions(_USE_MATH_DEFINES)

enable_testing()

include(CTest)
include(Catch)

add_library(vkm src/geom.cpp src/math.cpp)

add_executable(math_tests tests/math.cpp)
target_link_libraries(math_tests PRIVATE Catch2::Catch2WithMain vkm)

catch_discover_tests(math_tests)

add_executable(geom_tests tests/geom.cpp)
target_link_libraries(geom_tests PRIVATE Catch2::Catch2WithMain vkm)

catch_discover_tests(geom_tests)
