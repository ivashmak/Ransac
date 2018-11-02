#ifndef USAC_NEARESTNEIGHBORS_H
#define USAC_NEARESTNEIGHBORS_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <nanoflann.hpp>
#include "../../Generator/generator.h"

enum NEAR_NEIGHBORS_METHOD { FLANN, NANOFLANN };

class NearestNeighbors {
public:
    void getNearestNeighbors_flann (cv::InputArray input_points, int k_nearest_neighbors, cv::Mat &nearest_neighbors);
    void getNearestNeighbors_nanoflann (cv::InputArray input_points, int k_nearest_neighbors, cv::Mat &nearest_neighbors);

    void test (int knn=7);
};
#endif //USAC_NEARESTNEIGHBORS_H
