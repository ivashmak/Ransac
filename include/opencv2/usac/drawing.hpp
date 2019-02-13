// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_DRAWING_H
#define RANSAC_DRAWING_H

#include "model.hpp"
#include "line2d_estimator.hpp"
#include "quality.hpp"

namespace cv { namespace usac {
namespace draw {
    void draw(Model *model, const cv::Mat &points, const std::string &img_name1,
                     const std::string &img_name2);

    /*
     * w ~ original width;  h ~ original height; S original square;
     * constraint 1: new square is S' = w' * h' == 480000
     * constraint 2: ratio is the same
     *      S'
     * h' = -
     *      w'
     *
     * w'  w             wh'   wS'                  wS'
     * - = -   =>  w' = ---  = --   =>  w' = sqrt (----)
     * h'  h             h     hw'                  h
     *
     *                                              hS'
     *                                  h' = sqrt (----)
     *                                              w
     */
    void drawing_resize(cv::Mat &image);

    /*
     * Show inliers for image
     * @input_points         all point
     * @input_inliers_idxes  indexes of inliers
     * @image                image
     */
    void showInliers(cv::InputArray input_points, cv::InputArray input_inliers_idxes, cv::Mat &image);

    // y = kx + b
    void draw_line_kx_b(float k, float b, const cv::Scalar &color, cv::Mat img);

    void draw_line_model(Model *const model, cv::Scalar color, cv::Mat img, bool threshold);

    /*
     * Read image.
     * Show inliers of best ransac model.
     * Show inliers of non minimal best model.
     * To show threshold lines change false to true.
     */
    void draw_line(Model *const model, const cv::Mat &points, const std::string &img_name);

    /*
     * Draw epipolar lines by Fundamental Matrix
     */
    void drawEpipolarLines(cv::usac::Model * model, const cv::Mat &points,
                                  const std::string& img_name1,
                                  const std::string& img_name2);

    void drawPanorama(const std::vector<cv::Mat> &imgs, cv::Mat &panorama, const cv::Mat &H);
    void drawEpipolarLines_ (cv::Mat &img1, cv::Mat &img2, const std::vector<int> &inliers, const cv::Mat& lines1, const cv::Mat& lines2, const cv::Mat& pts1, const cv::Mat& pts2);
    void DrawMatches_(cv::Mat points, std::vector<int> inliers, cv::Mat image1, cv::Mat image2, cv::Mat &out_image);

    void drawHomography(Model *model, const cv::Mat &points,
    const std::string &img_name1,
    const std::string &img_name2);

    void
    drawMatches(cv::Mat &img_matches, const cv::Mat &img1, const cv::Mat &img2, const cv::Mat &points1,
                const cv::Mat &points2, const int *const inliers, int inliers_size);

    void
    drawErrors(cv::Mat &img1_error, cv::Mat &img2_error, const cv::Mat &points1, const cv::Mat &points2,
               const cv::Mat &H);

    void mat2keypoint(const cv::Mat &points, std::vector<cv::KeyPoint> &keypoints);
}
}}

#endif //RANSAC_DRAWING_H
