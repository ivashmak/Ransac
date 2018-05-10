cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project( Ransac	)
find_package( OpenCV REQUIRED )

add_executable(ransac main.cpp 
				Generator/generator.cpp Generator/generator.h 
				Detector/detector.cpp Detector/detector.h
				Ransac/NaiveRansac.cpp Ransac/AbstractRansac.h Ransac/NaiveRansac.h)

target_link_libraries( ransac ${OpenCV_LIBS} )
