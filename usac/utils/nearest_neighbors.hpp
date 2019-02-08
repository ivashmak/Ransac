// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_NEARESTNEIGHBORS_H
#define USAC_NEARESTNEIGHBORS_H

#include "../precomp.hpp"

#include <nanoflann.hpp>
#include "../../generator/generator.h"

class NearestNeighbors {
public:
    static void getNearestNeighbors_flann (const cv::Mat& points, int k_nearest_neighbors, cv::Mat &nearest_neighbors);
    static void getNearestNeighbors_nanoflann (const cv::Mat& points, int k_nearest_neighbors,
                                        cv::Mat &nearest_neighbors, bool get_distances,
                                        cv::Mat &nearest_neighbors_distances);

    static void getGridNearestNeighbors (const cv::Mat& points, int cell_sz, std::vector<std::vector<int>> &neighbors);
    static void test (int knn=7);
private:
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
};



#endif //USAC_NEARESTNEIGHBORS_H
