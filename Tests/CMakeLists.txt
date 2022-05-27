project(building-and-tracking-a-road-model)

include(FetchContent)
FetchContent_Declare(
    googletest
    # Specify the commit you depend on and update it regularly.
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(tests test_curvature_calculation.cpp test_remove_small_contours.cpp test_remove_small_contours ../src/ContourBuilder.cpp ../src/ContourBuilder.h ../src/Utils.h ../src/Utils.cpp ../src/CurvatureCalculator.cpp ../src/CurvatureCalculator.h test_perpendicular_line_calculation.cpp test_tangent_line_calculation.cpp test_system_solutions_with_respect_to_x.cpp)

target_link_libraries(tests gtest_main)

target_include_directories(
        tests
        PUBLIC
        ../src
)
