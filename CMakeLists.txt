cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project(Usac)
find_package( OpenCV REQUIRED )

# theia
find_package(Theia REQUIRED)
include_directories(${THEIA_INCLUDE_DIRS})

add_executable(
        ransac main.cpp
        Generator/generator.cpp Generator/generator.h

        Detector/ReadPoints.cpp Detector/ReadPoints.h
        Detector/detector.cpp Detector/detector.h

        Usac/Quality.h
        Usac/Model.h
        Usac/TerminationCriteria.h
        Usac/ProsacTerminationCriteria.h

        Usac/Helper/Drawing.h
        Usac/Helper/Logging.h
        Usac/Helper/DrawPanorama.cpp

        Usac/Estimator/Estimator.h
        Usac/Estimator/Line2DEstimator.h
        Usac/Estimator/HomographyEstimator.h
        Usac/Estimator/EssentialEstimator.h
        Usac/Estimator/FundamentalEstimator.h

        Usac/Sampler/Sampler.h
        Usac/Sampler/UniformSampler.h
        Usac/Sampler/NapsacSampler.h
        Usac/Sampler/EvsacSampler.h
        Usac/Sampler/ProsacSampler.h
        Usac/Sampler/ProsacSampler2.h

        Usac/DLT/DLT.h
        Usac/DLT/NormalizedDLT.h
        Usac/DLT/GetNormalizingTransformation.h

        Usac/Ransac/Ransac.cpp Usac/Ransac/Ransac.h

        Usac/FundamentalMatrixEstimation.h
        Usac/EssentialMatrixEstimation.h

        Tests/Tests.h
        Tests/testHomographyFitting.cpp
        Tests/testLineFitting.cpp
        Tests/testEssentialFitting.cpp
        Tests/testFundamentalFitting.cpp


        RandomGenerator/RandomGenerator.h
        RandomGenerator/ArrayRandomGenerator.h
        RandomGenerator/PrimeNumberRandomGenerator.h
        RandomGenerator/UniformRandomGenerator.h
        RandomGenerator/SimpleRandomGenerator.h
        RandomGenerator/ShuffleRandomGenerator.h

        RandomGenerator/EvaluateRandomGenerators.h

        Usac/Verbose.h
)
# parallel
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -pthread -ggdb" )
# warnings
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -Wextra" )
# optimalization
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3" )

target_link_libraries (ransac ${OpenCV_LIBS} )
target_link_libraries (ransac ${THEIA_LIBRARIES})