cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project (usac)
find_package (OpenCV REQUIRED)

add_executable(
        ransac main.cpp

        Generator/generator.cpp
        Generator/generator.h

        Detector/Reader.cpp
        Detector/Reader.h
        Detector/detector.cpp
        Detector/detector.h

        usac/quality/quality.hpp
        usac/quality/line_quality.hpp

        usac/termination_criteria/standard_termination_criteria.hpp
        usac/termination_criteria/prosac_termination_criteria.hpp
        usac/termination_criteria/termination_criteria.hpp

        usac/sampler/sampler.hpp
        usac/sampler/uniform_sampler.hpp
        usac/sampler/napsac_sampler.hpp
        usac/sampler/progressive_sampler.hpp
        usac/sampler/evsac_sampler.hpp
        usac/sampler/prosac_simple_sampler.hpp
        usac/sampler/prosac_sampler.hpp
        usac/sampler/prosac_napsac_sampler.hpp

        usac/ransac/ransac.cpp
        usac/ransac/ransac.hpp
        usac/ransac/ransac_output.hpp
        usac/ransac/init.hpp
        usac/ransac/init.cpp
        
        usac/estimator/estimator.hpp
        usac/estimator/line2d_estimator.hpp
        usac/estimator/homography_estimator.hpp
        usac/estimator/essential_estimator.hpp
        usac/estimator/fundamental_estimator.hpp

        usac/estimator/dlt/dlt.hpp usac/estimator/dlt/dlt.cpp
        usac/estimator/dlt/normalized_dlt.cpp
        usac/estimator/dlt/normalizing_transformation.cpp

        usac/estimator/essential/five_points.hpp
        usac/estimator/essential/five_points.cpp
        usac/estimator/essential/rpoly.cpp usac/estimator/essential/rpoly.hpp
        usac/estimator/essential/polynomial.hpp
        usac/estimator/essential/five_points_opencv.cpp

        usac/estimator/fundamental/seven_points.cpp
        usac/estimator/fundamental/eight_points.cpp
        usac/estimator/fundamental/fundamental_solver.hpp

        usac/local_optimization/local_optimization.hpp
        usac/local_optimization/inner_local_optimization.hpp
        usac/local_optimization/iterative_local_optimization.hpp
        usac/local_optimization/graphcut.cpp usac/local_optimization/graphcut.hpp
        usac/local_optimization/irls.hpp
        usac/local_optimization/greedy_local_optimization.hpp
        usac/local_optimization/sorted_lo.hpp
        
        usac/model.hpp
        usac/sprt.hpp
        usac/precomp.hpp

        Tests/tests.h
        Tests/test_homography_fitting.cpp
        Tests/test_line2d_fitting.cpp
        Tests/test_essential_fitting.cpp
        Tests/test_fundamental_fitting.cpp
        Tests/test.cpp
        Tests/statistical_results.h
        Tests/test_neighbors_search.cpp

        helper/Logging.h
        helper/drawing/Drawing.h
        helper/drawing/DrawPanorama.cpp
        helper/drawing/DrawEpipolarLines.cpp

        usac/random_generator/random_generator.hpp
        usac/random_generator/array_random_generator.hpp
        usac/random_generator/uniform_random_generator.hpp
        Tests/test_random_generator.h

        dataset/Dataset.h dataset/Dataset.cpp
        dataset/SaveGTModel.h dataset/SaveGTModel.cpp
        dataset/GetImage.h

        usac/utils/math.cpp usac/utils/math.hpp
        usac/utils/nearest_neighbors.cpp usac/utils/nearest_neighbors.hpp
        usac/utils/utils.hpp
        usac/utils/utils.cpp
)

# parallel
# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -pthread -ggdb" )
# warnings
# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -Wextra" )
# optimalization
# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3" )

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