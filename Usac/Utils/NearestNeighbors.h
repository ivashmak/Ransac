#ifndef USAC_NEARESTNEIGHBORS_H
#define USAC_NEARESTNEIGHBORS_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/miniflann.hpp>

#include "../../include/nanoflann.hpp"
#include "../../Generator/generator.h"

enum NEAR_NEIGHBORS_METHOD { FLANN, NANOFLANN };

class NearestNeighbors {
public:
    void getNearestNeighbors_flann (cv::InputArray input_points, int k_nearest_neighbors, cv::Mat &nearest_neighbors);
    void getNearestNeighbors_nanoflann (cv::InputArray input_points, int k_nearest_neighbors, cv::Mat &nearest_neighbors);

    void getNearestNeighbors (cv::InputArray input_points, int k_nearest_neighbors, cv::Mat &nearest_neighbors,
                              NEAR_NEIGHBORS_METHOD method=NEAR_NEIGHBORS_METHOD::FLANN) {

        if (method == NEAR_NEIGHBORS_METHOD::FLANN) {
            getNearestNeighbors_flann(input_points, k_nearest_neighbors, nearest_neighbors);
        } else {
            getNearestNeighbors_nanoflann(input_points, k_nearest_neighbors, nearest_neighbors);
        }
    }
    void test ();
};
#endif //USAC_NEARESTNEIGHBORS_H
