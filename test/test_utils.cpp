// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_TEST_UTILS_CPP_H
#define USAC_TEST_UTILS_CPP_H

#include "test_precomp.hpp"
#include "tests.hpp"
#include "../include/opencv2/usac/utils.hpp"
#include "../include/opencv2/usac/reader.hpp"

void opencv_test::testFindMedian () {

    long time1 = 0, time2 = 0;
    int num_tests = 10000;
    for (int test = 0; test < num_tests; test++) {
//        std::cout << "test " << test << "\n";
        int arr_size = 5000;

        if (test > num_tests / 2) {
            // for odd number testing
            arr_size = arr_size + 1;
        }

        int max_range = 5000;
        int arr[arr_size];
        for (int i = 0; i < arr_size; i++) {
            arr[i] = random() % max_range;
        }

        auto begin_time = std::chrono::steady_clock::now();
        int med = cv::usac::findMedian(arr, arr_size);
        time1 += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin_time).count();

        // check
        begin_time = std::chrono::steady_clock::now();
        std::sort(arr, arr + arr_size);
        int med_gt;
        if (arr_size % 2 == 0) {
            med_gt = (arr[arr_size / 2 - 1] + arr[arr_size / 2]) / 2;
        } else {
            med_gt = arr[arr_size / 2];
        }
        time2 += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin_time).count();

        if (med != med_gt) {
            std::cout << "fail est med " << med << " gt med " << med_gt << "\n";
            exit(1);
        }
    }

    std::cout << "proposal average time " << time1 / num_tests << "\n";
    std::cout << "easy-calculation average time " << time2 / num_tests << "\n";

}

void opencv_test::testFindNearestNeighbors (int knn) {
    knn = 7;

    std::clock_t start;
    double duration;

    cv::Mat points;
    cv::usac::Reader::LoadPointsFromFile (points, "../test/dataset/homography/graf_pts.txt");
    std::vector<std::vector<int>> neighbors;
    start = std::clock();
    cv::usac::NearestNeighbors::getGridNearestNeighbors(points, 50, neighbors);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout << "duration grid: "<< duration <<'\n';

    //  debug
//    for (int i = 0; i < neighbors.size(); i++) {
//        std::cout << "point " << i << ", neighbors: ";
//        for (int j = 0; j < neighbors[i].size(); j++) {
//            std::cout << neighbors[i][j] << " ";
//        }
//        std::cout << "\n";
//    }

//    for (int i = 0; i < points.rows; i++) {
//        if (neighbors[i].size() < 2) continue;
////        cv::Scalar color (random() % 255, random() % 255, random() % 255);
//        cv::Scalar color (255, 255, 255);
//        cv::Mat img1 = cv::imread ("../dataset/homography/"+img_name+"A.png");
//        cv::Mat img2 = cv::imread ("../dataset/homography/"+img_name+"B.png");
//        for (int n = 0; n < neighbors[i].size(); n++) {
//            cv::circle (img1, cv::Point_<float>(points1.at<float>(neighbors[i][n], 0), points1.at<float>(neighbors[i][n], 1)), 3, color, -1);
//            cv::circle (img2, cv::Point_<float>(points2.at<float>(neighbors[i][n], 0), points2.at<float>(neighbors[i][n], 1)), 3, color, -1);
//        }
//        cv::hconcat(img1, img2, img1);
//        cv::imshow("neighbors", img1);
//        cv::waitKey(0);
//    }

//    std::vector<cv::Point_<float>> points_v;
//    generate(points_v, false);
    cv::Mat_<int> nearest_neighbors_flann, nearest_neighbors_nanoflann;

//    cv::Mat_<float> points_mat = cv::Mat (points);
//
//
//    start = std::clock();
//    getNearestNeighbors_flann(points_mat, knn, nearest_neighbors_flann);
//    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
//    std::cout << "duration flann: "<< duration <<'\n';
//
    start = std::clock();
    cv::Mat neighbors_distances;
    cv::usac::NearestNeighbors::getNearestNeighbors_nanoflann(points, knn, nearest_neighbors_nanoflann, false, neighbors_distances);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout << "duration nanoflann: "<< duration <<'\n';
//    std::cout << nearest_neighbors_nanoflann << "\n";
    // points size 3300
    // flann 0.33 mcs
    // nanoflann 0.014

//    cv::hconcat(nearest_neighbors_flann, cv::Mat_<int>::zeros(points.size(), 3), nearest_neighbors_flann);
//    cv::hconcat(nearest_neighbors_flann, nearest_neighbors_nanoflann, nearest_neighbors_flann);
//
//    std::cout << nearest_neighbors_flann << "\n";

//    for (int p = 0; p < points.size(); p++) {
//        std::cout << "for point " << points_mat.row(p) << "\n";
//        for (int nn = 0; nn < knn; nn++) {
//            std::cout << "\tnorm of flann " <<
//                      cv::norm (points_mat.row(p), points_mat.row(nearest_neighbors_flann.at<int>(p,nn))) <<
//                      " ("<< nearest_neighbors_flann.at<int>(p,nn) << ")  VS  ";
//
//            std::cout << "nanoflann " <<
//                      cv::norm (points_mat.row(p), points_mat.row(nearest_neighbors_nanoflann.at<int>(p,nn))) <<
//                      " ("<< nearest_neighbors_nanoflann.at<int>(p,nn) << ")\n";
//        }
//    }
}

void opencv_test::testInv () {
    cv::Mat A;
    A = (cv::Mat_<float>(3,3) << 12, 2, 6, 12, 88, 0, 17, 90, 1);
    cv::Mat A_inv;
    int test_size = 1000000;

    std::clock_t start;
    double duration;

    start = std::clock();
    for (int i = 0; i < test_size; i++) {
        A_inv = A.inv();
    }
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout << "duration: "<< duration <<'\n';

    std::cout << A_inv << "\n";

    start = std::clock();
    for (int i = 0; i < test_size; i++) {
        cv::usac::inverse3x3(A, A_inv);
    }
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<< "duration: "<< duration <<'\n';

    std::cout << A_inv << "\n";
}


#endif //USAC_TEST_UTILS_CPP_H
