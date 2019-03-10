cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project (usac)
find_package (OpenCV REQUIRED)

add_executable(
        ransac test/test_main.cpp

        src/reader.cpp
        include/opencv2/usac/reader.hpp

        include/opencv2/usac/quality.hpp

        include/opencv2/usac/standard_termination_criteria.hpp
        include/opencv2/usac/prosac_termination_criteria.hpp
        include/opencv2/usac/termination_criteria.hpp

        include/opencv2/usac/sampler.hpp
        include/opencv2/usac/uniform_sampler.hpp
        include/opencv2/usac/napsac_sampler.hpp
        include/opencv2/usac/prosac_sampler.hpp

        src/ransac.cpp
        include/opencv2/usac/ransac.hpp
        include/opencv2/usac/ransac_output.hpp
        include/opencv2/usac/init.hpp
        src/init.cpp

        include/opencv2/usac/estimator.hpp
        include/opencv2/usac/line2d_estimator.hpp
        include/opencv2/usac/homography_estimator.hpp
        include/opencv2/usac/essential_estimator.hpp
        include/opencv2/usac/fundamental_estimator.hpp
        include/opencv2/usac/affine_estimator.hpp
        
        include/opencv2/usac/dlt.hpp
        src/dlt.cpp
        src/normalized_dlt.cpp
        src/normalizing_transformation.cpp

        include/opencv2/usac/five_points.hpp
        src/five_points.cpp
        src/rpoly.cpp
        include/opencv2/usac/rpoly.hpp
        include/opencv2/usac/polynomial.hpp

        src/seven_points.cpp
        src/eight_points.cpp
        include/opencv2/usac/fundamental_solver.hpp

        include/opencv2/usac/local_optimization.hpp
        include/opencv2/usac/inner_local_optimization.hpp
        include/opencv2/usac/iterative_local_optimization.hpp
        src/graphcut.cpp
        include/opencv2/usac/graphcut.hpp

        include/opencv2/usac/model.hpp
        include/opencv2/usac/sprt.hpp
        src/precomp.hpp

        test/tests.hpp
        test/test_homography_fitting.cpp
        test/test_line2d_fitting.cpp
        test/test_essential_fitting.cpp
        test/test_fundamental_fitting.cpp
        test/test.cpp
        test/test_precomp.hpp
        test/test_utils.cpp
        test/test_random_generator.cpp

        include/opencv2/usac/drawing.hpp
        src/drawing.cpp
        src/draw_epipolar_lines.cpp
        src/draw_panorama.cpp

        include/opencv2/usac/random_generator.hpp
        include/opencv2/usac/array_random_generator.hpp
        include/opencv2/usac/uniform_random_generator.hpp

        src/math.cpp
        include/opencv2/usac/math.hpp
        src/nearest_neighbors.cpp
        include/opencv2/usac/nearest_neighbors.hpp
        include/opencv2/usac/utils.hpp
        src/utils.cpp
)

# parallel
# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -pthread -ggdb" )
# warnings
# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -pthread -Wextra" )
# optimalization
# set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3" )

# Google test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

target_link_libraries (ransac ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
