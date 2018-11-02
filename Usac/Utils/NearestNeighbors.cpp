#include "NearestNeighbors.h"

#include <Eigen/Dense>
#include <opencv/cxeigen.hpp>
#include <opencv2/core/eigen.hpp>
/*
 * @points N x 2
 * x1 y1
 * ...
 * xN yN
 *
 * @k_nearest_neighbors is number of nearest neighbors for each point.
 *
 * @nearest_neighbors is matrix N x k of indexes of nearest points
 * x1_nn1 x1_nn2 ... x1_nnk
 * ...
 * xN_nn1 xN_nn2 ... xN_nnk
 */
void NearestNeighbors::getNearestNeighbors_nanoflann (cv::InputArray input_points, int k_nearest_neighbors,
                                                      cv::Mat &nearest_neighbors) {
    unsigned int points_size = input_points.getMat().rows;
    unsigned int dim = input_points.getMat().cols;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat;
    cv::cv2eigen(cv::Mat_<float>(input_points.getMat()), mat);

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> my_kd_tree_t;

    my_kd_tree_t mat_index(dim, std::cref(mat), 10 /* max leaf */);
    mat_index.index->buildIndex();

    // do a knn search
    unsigned long * ret_indexes = new unsigned long [k_nearest_neighbors];
    float * out_dists_sqr = new float [k_nearest_neighbors];
    nanoflann::KNNResultSet<float> resultSet(k_nearest_neighbors);

    nearest_neighbors = cv::Mat_<int> (points_size, k_nearest_neighbors);
    int * nearest_neighbors_ptr = (int *) nearest_neighbors.data;
    int pp;
    for (int p = 0; p < points_size; p++) {
        resultSet.init(ret_indexes, out_dists_sqr);
        mat_index.index->findNeighbors(resultSet, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(mat.row(p)).data(), nanoflann::SearchParams(10));

//        std::cout << "query point idx " << p << " value is " << mat.row(p) << "\n";
        pp = k_nearest_neighbors * p;
        for (int nn = 0; nn < k_nearest_neighbors; nn++) {
//            std::cout << "\tret_index[" << nn << "]=" << ret_indexes[nn]
//                      << "\tout_dist_sqr=" << out_dists_sqr[nn] << '\n';
            nearest_neighbors_ptr[pp + nn] = ret_indexes[nn];
        }
    }
    nearest_neighbors = nearest_neighbors.colRange(1, k_nearest_neighbors);

    delete ret_indexes, out_dists_sqr;
}

/*
 * @points N x 2
 * x1 y1
 * ...
 * xN yN
 *
 * @k_nearest_neighbors is number of nearest neighbors for each point.
 *
 * @nearest_neighbors is matrix N x k of indexes of nearest points
 * x1_nn1 x1_nn2 ... x1_nnk
 * ...
 * xN_nn1 xN_nn2 ... xN_nnk
 *
 */
void NearestNeighbors::getNearestNeighbors_flann (cv::InputArray input_points, int k_nearest_neighbors, cv::Mat &nearest_neighbors) {
    unsigned int points_size = input_points.getMat().rows;
    cv::Mat_<float> points = input_points.getMat();
    cv::flann::LinearIndexParams flannIndexParams;
    cv::flann::Index flannIndex (points.reshape(1), flannIndexParams);
    cv::Mat dists;

    flannIndex.knnSearch(points, nearest_neighbors, dists, k_nearest_neighbors);

    // first nearest neighbor of point is this point itself.
    // remove this first column
    nearest_neighbors = nearest_neighbors.colRange(1, k_nearest_neighbors);
}

void NearestNeighbors::test (int knn) {
    std::vector<cv::Point_<float>> points;
    generate(points, false);
    cv::Mat_<int> nearest_neighbors_flann, nearest_neighbors_nanoflann;

    cv::Mat_<float> points_mat = cv::Mat (points);

    std::clock_t start;
    double duration;

    start = std::clock();
    getNearestNeighbors_flann(points_mat, knn, nearest_neighbors_flann);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout << "duration flann: "<< duration <<'\n';

    start = std::clock();
    getNearestNeighbors_nanoflann(points_mat, knn, nearest_neighbors_nanoflann);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout << "duration nanoflann: "<< duration <<'\n';

    // points size 3300
    // flann 0.33 mcs
    // nanoflann 0.014

//    knn -= 1;
//    cv::hconcat(nearest_neighbors_flann, cv::Mat_<int>::zeros(points.size(), 3), nearest_neighbors_flann);
//    cv::hconcat(nearest_neighbors_flann, nearest_neighbors_nanoflann, nearest_neighbors_flann);

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