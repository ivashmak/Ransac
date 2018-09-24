#include <opencv/cv.hpp>
#include "Drawing.h"

void Drawing::drawPanorama (const std::vector<cv::Mat>& imgs, cv::Mat &panorama, const cv::Mat& H) {
    cv::warpPerspective(imgs[1], panorama, H, cv::Size(2*imgs[1].cols, imgs[1].rows));
    cv::Mat half(panorama, cv::Rect(0,0,imgs[0].cols,imgs[0].rows));
    imgs[0].copyTo(half);

    cv::resize(panorama, panorama, cv::Size (075 * panorama.cols, panorama.rows));
}
