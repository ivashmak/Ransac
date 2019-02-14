// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef READPOINTS_READPOINTS_H
#define READPOINTS_READPOINTS_H

#include <opencv2/core/mat.hpp>

namespace cv { namespace usac {
class Reader {
public:

    static void getMatrix3x3(const std::string &filename, cv::Mat &model);

    static bool LoadPointsFromFile(cv::Mat &points, const char *file);

    static void readInliers(std::vector<int> &inliers, const std::string &filename);
};
}}
#endif //READPOINTS_READPOINTS_H