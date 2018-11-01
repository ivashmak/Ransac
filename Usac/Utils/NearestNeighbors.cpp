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
    unsigned int dim = 2;

//    Eigen::Matrix<float> mat(nSamples, dim);
    cv::Mat_<float> mat = input_points.getMat();
    const float max_range = 20;

    //	cout << mat << endl;

    // Query point:
    std::vector<float> query_pt(dim);

    query_pt[0] = mat.at<float>(0,0);
    query_pt[1] = mat.at<float>(0,1);

    // ------------------------------------------------------------
    // construct a kd-tree index:
    //    Some of the different possibilities (uncomment just one)
    // ------------------------------------------------------------
    // Dimensionality set at run-time (default: L2)
//    typedef nanoflann::KDTreeEigenMatrixAdaptor<cv::Mat_<float>>
//            my_kd_tree_t;
//
//
//    my_kd_tree_t mat_index(dim, std::cref(mat), 10 /* max leaf */);
//    mat_index.index->buildIndex();
//
//    // do a knn search
//    const size_t num_results = 3;
//    std::vector<size_t> ret_indexes(num_results);
//    std::vector<float> out_dists_sqr(num_results);
//
//    nanoflann::KNNResultSet<float> resultSet(num_results);
//
//    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
//    mat_index.index->findNeighbors(resultSet, &query_pt[0],
//                                   nanoflann::SearchParams(10));
//
//    std::cout << "knnSearch(nn=" << num_results << "): \n";
//    for (size_t i = 0; i < num_results; i++)
//        std::cout << "ret_index[" << i << "]=" << ret_indexes[i]
//                  << " out_dist_sqr=" << out_dists_sqr[i] << '\n';
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

void NearestNeighbors::test () {
    std::vector<cv::Point_<float>> points;
    generate(points, false);
    cv::Mat nearest_neighbors;
    getNearestNeighbors_flann(cv::Mat(points), 7, nearest_neighbors);

//    getNearestNeighbors_nanoflann(cv::Mat(points), 7, nearest_neighbors);

}