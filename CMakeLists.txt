cmake_minimum_required (VERSION 2.8.3)
project (particle_filter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package (OpenCV REQUIRED)

# add executable
add_executable (
	SimulatorTest 
	src/Test.cpp
	src/Simulator.cpp
	src/ParticleFilter.cpp
	src/MotionModel.cpp
	src/MeasurementModel.cpp
	src/Probability.cpp
)
# include directories
include_directories (
	include
)
# link OpenCV libraries
target_link_libraries (
	SimulatorTest
	${OpenCV_LIBS}
)
