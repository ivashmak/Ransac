// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_UTILS_H
#define USAC_UTILS_H

#include <opencv2/core/mat.hpp>

void densitySort (const cv::Mat &points, int max_neighbor, cv::Mat &sorted_points);

#endif //USAC_UTILS_H
