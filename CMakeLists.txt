cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project( Ransac	)
find_package( OpenCV REQUIRED )

add_executable(
        ransac main.cpp
        Ransac/Generator/generator.cpp Ransac/Generator/generator.h
        Ransac/Detector/detector.cpp Ransac/Detector/detector.h
        Ransac/Estimator.h
        Ransac/Ransac.cpp Ransac/Ransac.h
        Ransac/Line2DEstimator.h
        Ransac/Quality.h
        Ransac/Model.h
        Ransac/Sampler.h Ransac/UniformSampler.h
        Ransac/TerminationCriteria.h
        Ransac/Drawing.h

        Homographies/ReadPoints/ReadPoints.cpp Homographies/ReadPoints/ReadPoints.h
        Homographies/Estimator.h
        Homographies/HomographyEstimator.h
        Homographies/DLT.cpp
        Homographies/NormalizedDLT.cpp
        Homographies/GetNormalizingTransformation.cpp

        tests/Tests.h
        tests/testHomographies.cpp
        tests/testRansac.cpp
)

# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -fopenmp -Wextra" )

target_link_libraries( ransac ${OpenCV_LIBS} )