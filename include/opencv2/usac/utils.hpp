// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_UTILS_H
#define USAC_UTILS_H

#include "../precomp.hpp"

void densitySort (const cv::Mat &points, int max_neighbor, cv::Mat &sorted_points);
void splitFilename (const std::string &filename, std::string &path, std::string &name, std::string &ext);

int quicksort_median (int * array, unsigned int k_minth, unsigned int left, unsigned int right);
int findMedian (int * array, unsigned int length, bool make_copy=false);



#endif //USAC_UTILS_H
