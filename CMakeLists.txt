cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project (Usac)
find_package (OpenCV REQUIRED)

add_executable(
        ransac main.cpp

        Generator/generator.cpp
        Generator/generator.h

        Detector/Reader.cpp
        Detector/Reader.h
        Detector/detector.cpp
        Detector/detector.h

        Usac/Quality/Quality.h
        Usac/Quality/LineQuality.h

        Usac/Model.h

        Usac/TerminationCriteria/StandardTerminationCriteria.h
        Usac/TerminationCriteria/ProsacTerminationCriteria.h
        Usac/TerminationCriteria/TerminationCriteria.h

        Usac/Sampler/Sampler.h
        Usac/Sampler/UniformSampler.h
        Usac/Sampler/NapsacSampler.h
        Usac/Sampler/ProgressiveNapsac.h
        Usac/Sampler/EvsacSampler.h
        Usac/Sampler/ProsacSimpleSampler.h
        Usac/Sampler/ProsacSampler.h
        Usac/Sampler/ProsacNapsacSampler.h

        Usac/Ransac/Ransac.cpp
        Usac/Ransac/Ransac.h
        Usac/Ransac/RansacOutput.h

        Usac/Estimator/Estimator.h
        Usac/Estimator/Line2DEstimator.h
        Usac/Estimator/HomographyEstimator.h
        Usac/Estimator/EssentialEstimator.h
        Usac/Estimator/FundamentalEstimator.h

        Usac/Estimator/DLT/DLT.h Usac/Estimator/DLT/DLT.cpp
        Usac/Estimator/DLT/NormalizedDLT.cpp
        Usac/Estimator/DLT/NormalizingTransformation.cpp

        Usac/Estimator/Essential/FivePoints.h
        Usac/Estimator/Essential/Rpoly.cpp Usac/Estimator/Essential/Rpoly.h
        Usac/Estimator/Essential/Polynomial.h
        Usac/Estimator/Essential/FivePointsOpenCV.cpp
        Usac/Estimator/Essential/FivePoints.cpp

        Usac/Estimator/Fundamental/SevenPoints.cpp
        Usac/Estimator/Fundamental/EightPoints.cpp
        Usac/Estimator/Fundamental/FundemantalSolver.h

        Usac/LocalOptimization/LocalOptimization.h
        Usac/LocalOptimization/RansacLocalOptimization.h
        Usac/LocalOptimization/GraphCut.cpp Usac/LocalOptimization/GraphCut.h
        Usac/LocalOptimization/GreedyLocalOptimization.h
        Usac/LocalOptimization/IRLS.h

        Usac/SPRT.h

        Tests/Tests.h
        Tests/testHomographyFitting.cpp
        Tests/testLineFitting.cpp
        Tests/testEssentialFitting.cpp
        Tests/testFundamentalFitting.cpp
        Tests/test.cpp
        Tests/StatisticalResults.h
        Tests/samplerInitialization.cpp
        Tests/estimatorInitialization.cpp

        Usac/Helper/Logging.h
        Usac/Helper/Drawing/Drawing.h
        Usac/Helper/Drawing/DrawPanorama.cpp
        Usac/Helper/Drawing/DrawEpipolarLines.cpp

        RandomGenerator/RandomGenerator.h
        RandomGenerator/ArrayRandomGenerator.h
        RandomGenerator/UniformRandomGenerator.h
        RandomGenerator/EvaluateRandomGenerators.h

        dataset/Dataset.h dataset/Dataset.cpp
        dataset/SaveGTModel.h dataset/SaveGTModel.cpp
        dataset/GetImage.h

        Usac/Utils/Math.cpp Usac/Utils/Math.h
        Usac/Utils/NearestNeighbors.cpp Usac/Utils/NearestNeighbors.h
        Usac/Utils/Utils.h
        Usac/Utils/Utils.cpp

        Usac/Verbose.h
)

# parallel
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -pthread -ggdb" )
# warnings
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -Wextra" )
# optimalization
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3" )

target_link_libraries (ransac ${OpenCV_LIBS} )

# theia
#find_package(Theia REQUIRED)
#include_directories(${THEIA_INCLUDE_DIRS})
#target_link_libraries (ransac ${THEIA_LIBRARIES})

# Nanoflann
find_package(nanoflann)
target_link_libraries(ransac nanoflann::nanoflann)

# Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (ransac Eigen3::Eigen)