cmake_minimum_required(VERSION 2.8)
project( Ransac	)
find_package( OpenCV REQUIRED )

add_executable(ransac ransac.cpp Generator/generator.cpp Generator/generator.h Detector/detector.cpp Detector/detector.h)
target_link_libraries( ransac ${OpenCV_LIBS} )
