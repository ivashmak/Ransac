// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "utils.hpp"
#include "nearest_neighbors.hpp"

void densitySort (const cv::Mat &points, int knn, cv::Mat &sorted_points) {
    NearestNeighbors nn;
    // get neighbors
    cv::Mat neighbors, neighbors_dists;
    nn.getNearestNeighbors_nanoflann(points, knn, neighbors, true, neighbors_dists);
    //

    std::vector<int> sorted_idx(points.rows);
    std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
    float sum1, sum2;
    int idxa, idxb;
    float *neighbors_dists_ptr = (float *) neighbors_dists.data;
    std::sort(sorted_idx.begin(), sorted_idx.end(), [&](int a, int b) {
        sum1 = 0, sum2 = 0;
        idxa = knn * a, idxb = knn * b;
        for (int i = 0; i < 4; i++) {
            sum1 += neighbors_dists_ptr[idxa + i];
            sum2 += neighbors_dists_ptr[idxb + i];
        }
        return sum1 < sum2;
    });

    for (int i = 0; i < points.rows; i++) {
        sorted_points.push_back(points.row(sorted_idx[i]));
    }
}

/*
 * filename = path/name.ext
 * path
 * name
 * ext
 */
void splitFilename (const std::string &filename, std::string &path, std::string &name, std::string &ext) {
    const unsigned long dot = filename.find_last_of('.');
    const unsigned long slash = filename.find_last_of('/');
    // substr (pos, n) take substring of size n starting from position pos.
    path = filename.substr(0, slash+1);
    name = filename.substr(slash+1, dot-slash-1);
    ext = filename.substr(dot+1, filename.length()-1);
}