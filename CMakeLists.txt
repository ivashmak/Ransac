cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project( Ransac	)
find_package( OpenCV REQUIRED )

add_executable(ransac main.cpp 
				Generator/generator.cpp Generator/generator.h 
				Detector/detector.cpp Detector/detector.h
				Ransac/NaiveRansac.cpp Ransac/NaiveRansac.h
				Ransac/Quality.h Ransac/Model.h Ransac/Estimator.h Ransac/Sampler.h Ransac/TerminationCriteria.h
				Ransac/Line.h)

# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -fopenmp -Wextra" )

target_link_libraries( ransac ${OpenCV_LIBS} )
