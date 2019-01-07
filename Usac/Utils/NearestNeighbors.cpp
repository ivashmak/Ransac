#include "NearestNeighbors.h"
#include "../../Detector/Reader.h"
#include "Math.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

/*
 * Problem with repeated points in flann and nanoflann:
 *

0 38 37 39 42 20 21
1 32 2 30 34 33 47
2 30 32 1 34 33 31
3 9 43 11 15 14 29
4 20 21 38 37 39 16
5 6 51 29 18 19 9
6 5 51 29 18 19 9
7 29 47 5 6 3 9
8 23 49 48 35 52 22
9 43 3 11 15 14 29
10 13 15 14 50 11 9
11 15 14 9 43 3 13
12 44 45 25 24 27 0
13 10 15 14 50 11 9
15 14 11 13 9 43 3 // here must be 14 15
15 14 11 13 9 43 3
 */

/*
[0, 37, 38, 39, 42, 20, 21, 4;
 1, 32, 2, 30, 33, 34, 47, 31;
 2, 30, 32, 1, 33, 34, 31, 46;
 3, 9, 43, 11, 14, 15, 29, 13;
 4, 20, 21, 37, 38, 39, 16, 17;
 5, 6, 51, 29, 18, 19, 9, 43;
 6, 5, 51, 29, 18, 19, 9, 43;
 7, 29, 47, 5, 6, 3, 9, 43;
 8, 23, 49, 48, 35, 52, 22, 28;
 9, 43, 3, 11, 14, 15, 29, 13;
 10, 13, 14, 15, 50, 11, 9, 43;
 11, 14, 15, 9, 43, 3, 13, 29;
 12, 44, 45, 24, 25, 27, 0, 20;
 13, 10, 14, 15, 50, 11, 9, 43;
 14, 15, 11, 13, 9, 43, 3, 10;
 14, 15, 11, 13, 9, 43, 3, 10; // must be 15 14
 16, 17, 4, 20, 21, 18, 19, 37;
 */




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

        for (unsigned int p = 0; p < points_size; p++) {
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
        for (unsigned int p = 0; p < points_size; p++) {
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

   // std::cout << nearest_neighbors << "\n\n";
   // std::cout << nearest_neighbors_distances << "\n\n";

    delete[] ret_indexes;
    delete[] out_dists_sqr;
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

struct CellCoord {
    int c1x, c1y, c2x, c2y;
    CellCoord() {};
    void init (int c1x_, int c1y_, int c2x_, int c2y_) {
        c1x = c1x_;
        c1y = c1y_;
        c2x = c2x_;
        c2y = c2y_;
    }
    bool operator==(const CellCoord &o) const {
        return c1x == o.c1x && c1y == o.c1y && c2x == o.c2x && c2y == o.c2y;
    }

    bool operator<(const CellCoord &o) const {
        if (c1x < o.c1x) return true;
        if (c1x == o.c1x && c1y < o.c1y) return true;
        if (c1x == o.c1x && c1y == o.c1y && c2x < o.c2x) return true;
        if (c1x == o.c1x && c1y == o.c1y && c2x == o.c2x && c2y < o.c2y) return true;
        else return false;
    }
};

struct hash_fn {
    std::size_t operator() (const CellCoord &coord) const {
        std::size_t h1 = std::hash<int>()(coord.c1x);
        std::size_t h2 = std::hash<int>()(coord.c1y);
        std::size_t h3 = std::hash<int>()(coord.c2x);
        std::size_t h4 = std::hash<int>()(coord.c2y);
        return h1 ^ h2 ^ h3 ^ h4;
    }
};

#include <unordered_map>
void NearestNeighbors::getGridNearestNeighbors (const cv::Mat& points, int cell_sz, std::vector<std::vector<int>> &neighbors) {
    // cell size 25, 50, 100
    std::map<CellCoord, std::vector<int>> neighbors_map;
//    std::unordered_map<CellCoord, std::vector<int>, hash_fn> neighbors_map;

    float *points_p = (float *) points.data;
    unsigned int idx, points_size = points.rows;
    CellCoord c;
    neighbors = std::vector<std::vector<int>>(points_size);
    for (unsigned int i = 0; i < points_size; i++) {
        neighbors[i].reserve(10); // reserve predicted neighbors size

        idx = 4 * i;
        c.init(points_p[idx] / cell_sz, points_p[idx + 1] / cell_sz, points_p[idx + 2] / cell_sz,
               points_p[idx + 3] / cell_sz);
        neighbors_map[c].push_back(i);

    }

    // debug
//    for (auto cells : neighbors_map) {
//        std::cout << "key = (" << cells.first.c1x << " " << cells.first.c1y << " " << cells.first.c2x << " "
//                  << cells.first.c2y <<
//                  ") values = ";
//        for (auto v : cells.second) {
//            std::cout << v << " ";
//        }
//        std::cout << "\n";
//    }

    unsigned long neighbors_in_cell;
    for (auto cells : neighbors_map) {
        neighbors_in_cell = cells.second.size();
        if (neighbors_in_cell < 2) continue;

        for (unsigned int n1 = 0; n1 < neighbors_in_cell; n1++) {
            for (unsigned int n2 = n1+1; n2 < neighbors_in_cell; n2++) {
                neighbors[cells.second[n1]].push_back(cells.second[n2]);
                neighbors[cells.second[n2]].push_back(cells.second[n1]);
            }
        }
    }
}

void NearestNeighbors::test (int knn) {
    knn = 7;

    std::clock_t start;
    double duration;

    std::string img_name = "Brussels";
    cv::Mat points, points1, points2;
    Reader::read_points (points1, points2, "../dataset/homography/"+img_name+"_pts.txt");
    cv::hconcat(points1, points2, points);
    std::vector<std::vector<int>> neighbors;
    start = std::clock();
    getGridNearestNeighbors(points, 50, neighbors);
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
    getNearestNeighbors_nanoflann(points, knn, nearest_neighbors_nanoflann, false, neighbors_distances);
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