// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/nearest_neighbors.hpp"

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
void cv::usac::NearestNeighbors::getNearestNeighbors_nanoflann (const cv::Mat& points, int k_nearest_neighbors,
                                                      cv::Mat &nearest_neighbors, bool get_distances,
                                                      cv::Mat &nearest_neighbors_distances) {
    unsigned int points_size = points.rows;
    unsigned int dim = points.cols;
        
    std::vector<std::vector<float>> samples (points_size, std::vector<float>(dim));
    
    for (unsigned int p = 0; p < points_size; p++) {
        points.row(p).copyTo(samples[p]);
    }

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor< std::vector<std::vector<float> >, float >  my_kd_tree_t;

    my_kd_tree_t   mat_index(dim /*dim*/, samples, 10 /* max leaf */ );
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
            mat_index.index->findNeighbors(resultSet, &samples[p][0], nanoflann::SearchParams(10));
            p_idx = k_nearest_neighbors * p;

            for (int nn = 0; nn < k_nearest_neighbors; nn++) {
                nearest_neighbors_ptr[p_idx + nn] = (int) ret_indexes[nn + 1];
                nearest_neighbors_distances_ptr[p_idx + nn] = out_dists_sqr[nn + 1];
            }
        }
    } else {
        for (unsigned int p = 0; p < points_size; p++) {
            resultSet.init(ret_indexes, out_dists_sqr);
            mat_index.index->findNeighbors(resultSet, &samples[p][0], nanoflann::SearchParams(10));
            p_idx = k_nearest_neighbors * p;
            for (int nn = 0; nn < k_nearest_neighbors; nn++) {
                nearest_neighbors_ptr[p_idx + nn] = ret_indexes[nn + 1];
            }
        }
    }

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
void cv::usac::NearestNeighbors::getNearestNeighbors_flann (const cv::Mat& points, int k_nearest_neighbors, cv::Mat &nearest_neighbors) {
    unsigned int points_size = points.rows;
    cv::flann::LinearIndexParams flannIndexParams;
    cv::flann::Index flannIndex (points.reshape(1), flannIndexParams);
    cv::Mat dists;

    flannIndex.knnSearch(points, nearest_neighbors, dists, k_nearest_neighbors+1);

    // first nearest neighbor of point is this point itself.
    // remove this first column
    nearest_neighbors.colRange(1, k_nearest_neighbors+1).copyTo (nearest_neighbors);
}

void cv::usac::NearestNeighbors::getGridNearestNeighbors (const cv::Mat& points, int cell_sz, std::vector<std::vector<int>> &neighbors) {
    std::map<CellCoord, std::vector<int>> neighbors_map;

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