#include "NearestNeighbors.h"

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
void NearestNeighbors::getNearestNeighbors_nanoflann (cv::InputArray input_points, int k_nearest_neighbors, cv::Mat &nearest_neighbors) {
    unsigned int points_size = input_points.getMat().rows;

//     NUM query_pt[1] = { 0.5};
//
//     // construct a kd-tree index:
//     typedef nanoflann::KDTreeSingleIndexAdaptor<size_t, cv::Mat_<float>> my_kd_tree_t;
//     cv::Mat points = input_points.getMat();
//     my_kd_tree_t index(1 /*dim*/, points, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
//     index.buildIndex();

//     // do a knn search
//     const size_t num_results = 1;
//     std::vector<size_t>   ret_indexes(num_results);
//     std::vector<NUM> out_dists_sqr(num_results);
//
//     nanoflann::KNNResultSet<NUM> resultSet(num_results);
//
//     resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
//     index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10) );

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
    cv::Mat dists, k_nearest_neighbors_indices;

    flannIndex.knnSearch(points, k_nearest_neighbors_indices, dists, 7);

    // first nearest neighbor of point is this point itself.
    // remove this first column
    k_nearest_neighbors_indices = k_nearest_neighbors_indices.colRange(1, k_nearest_neighbors);
}
void NearestNeighbors::test () {
    std::vector<cv::Point_<float>> points;
    generate(points, false);
    cv::Mat nearest_neighbors;
    getNearestNeighbors(cv::Mat(points), 7, nearest_neighbors);
}