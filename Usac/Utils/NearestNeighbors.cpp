#include "NearestNeighbors.h"

#include <Eigen/Dense>
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
void NearestNeighbors::getNearestNeighbors_nanoflann (const cv::Mat& points, int k_nearest_neighbors,
                                                      cv::Mat &nearest_neighbors, bool get_distances,
                                                      cv::Mat &nearest_neighbors_distances) {
    unsigned int points_size = points.rows;
    unsigned int dim = points.cols;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat;
    cv::cv2eigen(points, mat);

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> my_kd_tree_t;

    my_kd_tree_t mat_index(dim, std::cref(mat), 10 /* max leaf */);
    mat_index.index->buildIndex();

    // do a knn search
    unsigned long *ret_indexes = new unsigned long[k_nearest_neighbors + 1];
    float *out_dists_sqr = new float[k_nearest_neighbors + 1];
    nanoflann::KNNResultSet<float> resultSet(k_nearest_neighbors + 1);

    nearest_neighbors = cv::Mat_<int>(points_size, k_nearest_neighbors);
    int *nearest_neighbors_ptr = (int *) nearest_neighbors.data;

    int p_idx;
    if (get_distances) {
        nearest_neighbors_distances = cv::Mat_<float>(points_size, k_nearest_neighbors);
        float *nearest_neighbors_distances_ptr = (float *) nearest_neighbors_distances.data;

        for (int p = 0; p < points_size; p++) {
            resultSet.init(ret_indexes, out_dists_sqr);
            mat_index.index->findNeighbors(resultSet,
                                           Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(mat.row(p)).data(),
                                           nanoflann::SearchParams(10));
            p_idx = k_nearest_neighbors * p;

            for (int nn = 0; nn < k_nearest_neighbors; nn++) {
                nearest_neighbors_ptr[p_idx + nn] = (int) ret_indexes[nn + 1];
                nearest_neighbors_distances_ptr[p_idx + nn] = out_dists_sqr[nn + 1];
            }
        }
    } else {
        for (int p = 0; p < points_size; p++) {
            resultSet.init(ret_indexes, out_dists_sqr);
            mat_index.index->findNeighbors(resultSet,
                                           Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(mat.row(p)).data(),
                                           nanoflann::SearchParams(10));
            p_idx = k_nearest_neighbors * p;
            for (int nn = 0; nn < k_nearest_neighbors; nn++) {
                nearest_neighbors_ptr[p_idx + nn] = ret_indexes[nn + 1];
            }
        }
    }

//    std::cout << nearest_neighbors << "\n\n";
//    std::cout << nearest_neighbors_distances << "\n\n";

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
void NearestNeighbors::getNearestNeighbors_flann (const cv::Mat& points, int k_nearest_neighbors, cv::Mat &nearest_neighbors) {
    unsigned int points_size = points.rows;
    cv::flann::LinearIndexParams flannIndexParams;
    cv::flann::Index flannIndex (points.reshape(1), flannIndexParams);
    cv::Mat dists;

    flannIndex.knnSearch(points, nearest_neighbors, dists, k_nearest_neighbors+1);

    // first nearest neighbor of point is this point itself.
    // remove this first column
    nearest_neighbors.colRange(1, k_nearest_neighbors+1).copyTo (nearest_neighbors);

//    std::cout << nearest_neighbors << "\n\n";
//    std::cout << dists << "\n\n";
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
    cv::Mat neighbors_distances;
    getNearestNeighbors_nanoflann(points_mat, knn, nearest_neighbors_nanoflann, false, neighbors_distances);
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout << "duration nanoflann: "<< duration <<'\n';

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