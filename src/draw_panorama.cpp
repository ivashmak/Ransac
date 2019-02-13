// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/drawing.hpp"

void cv::usac::draw::drawPanorama (const std::vector<cv::Mat>& imgs, cv::Mat &panorama, const cv::Mat& H) {
    cv::warpPerspective(imgs[1], panorama, H, cv::Size(2*imgs[1].cols, imgs[1].rows));
    cv::Mat half(panorama, cv::Rect(0,0,imgs[0].cols,imgs[0].rows));
    imgs[0].copyTo(half);
}
