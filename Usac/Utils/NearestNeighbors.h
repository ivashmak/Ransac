// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_NEARESTNEIGHBORS_H
#define USAC_NEARESTNEIGHBORS_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <nanoflann.hpp>
#include "../../Generator/generator.h"

enum NEAR_NEIGHBORS_METHOD { FLANN, NANOFLANN };

class NearestNeighbors {
public:
    void getNearestNeighbors_flann (const cv::Mat& points, int k_nearest_neighbors, cv::Mat &nearest_neighbors);
    void getNearestNeighbors_nanoflann (const cv::Mat& points, int k_nearest_neighbors,
                                                          cv::Mat &nearest_neighbors, bool get_distances,
                                                          cv::Mat &nearest_neighbors_distances);
    void getGridNearestNeighbors (const cv::Mat& points, int cell_sz, std::vector<std::vector<int>> &neighbors);
    void test (int knn=7);
};
#endif //USAC_NEARESTNEIGHBORS_H
