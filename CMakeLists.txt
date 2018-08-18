cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project(Usac)
find_package( OpenCV REQUIRED )

add_executable(
        ransac main.cpp
        Generator/generator.cpp Generator/generator.h

        Detector/ReadPoints.cpp Detector/ReadPoints.h
        Detector/detector.cpp Detector/detector.h

        Usac/Quality.h
        Usac/Model.h
        Usac/TerminationCriteria.h
        Usac/Helper/Drawing.h

        Usac/Estimator/Estimator.h
        Usac/Estimator/Line2DEstimator.h
        Usac/Estimator/HomographyEstimator.h

        Usac/Sampler/Sampler.h
        Usac/Sampler/UniformSampler.h
        Usac/Sampler/NapsacSampler.h

        Usac/Homographies/DLT.cpp
        Usac/Homographies/NormalizedDLT.cpp
        Usac/Homographies/GetNormalizingTransformation.cpp
        Usac/Homographies/HomographyMethods.h

        Usac/Ransac/Ransac.cpp Usac/Ransac/Ransac.h
        Usac/Prosac.cpp Usac/Prosac.h

        Tests/Tests.h
        Tests/testHomographyFitting.cpp
        Tests/testLineFitting.cpp
)

# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -fopenmp -Wextra" )

target_link_libraries( ransac ${OpenCV_LIBS} )