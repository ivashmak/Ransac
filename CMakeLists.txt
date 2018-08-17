cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project(Usac)
find_package( OpenCV REQUIRED )

add_executable(
        ransac main.cpp
        Generator/generator.cpp Generator/generator.h
        Detector/detector.cpp Detector/detector.h
        Usac/Estimator.h
        Usac/Line2DEstimator.h
        Usac/Quality.h
        Usac/Model.h
        Usac/Sampler.h Usac/UniformSampler.h
        Usac/TerminationCriteria.h
        Usac/Drawing.h

        Detector/ReadPoints.cpp Detector/ReadPoints.h
        Usac/HomographyEstimator.h
        Usac/DLT.cpp
        Usac/NormalizedDLT.cpp
        Usac/GetNormalizingTransformation.cpp
        Usac/HomographyMethods.h

        Usac/Prosac.cpp Usac/Prosac.h
        Usac/Ransac.cpp Usac/Ransac.h
        Usac/Napsac.cpp Usac/Napsac.h

        tests/Tests.h
        tests/testHomographyFitting.cpp
        tests/testLineFitting.cpp
)

# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -fopenmp -Wextra" )

target_link_libraries( ransac ${OpenCV_LIBS} )