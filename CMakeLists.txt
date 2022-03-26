cmake_minimum_required(VERSION 3.20)
project(building-and-tracking-a-road-model)

set(CMAKE_CXX_STANDARD 14)

add_executable(building-and-tracking-a-road-model
		src/main.cpp
		src/Utils.cpp src/ExperimentWithCurvatureCalculation.cpp src/ExperimentWithCurvatureCalculation.h src/RoadModel.cpp src/RoadModel.h src/contourBuilder.cpp src/contourBuilder.h)

# include_directories(src)

target_include_directories(
	building-and-tracking-a-road-model
	PUBLIC
	src)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(building-and-tracking-a-road-model ${OpenCV_LIBS} )

enable_testing()

add_subdirectory(Tests)