#include_directories(lib/googletest/googletest lib/googletest/googletest/include)

add_executable(runTest test_curvature_calculation.cpp ../../src/utils.cpp)

target_link_libraries(runTest gtest gtest_main)