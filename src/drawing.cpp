// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/drawing.hpp"

void cv::usac::draw::draw(Model *model, const cv::Mat &points, const std::string &img_name1,
                 const std::string &img_name2) {
    if (model->estimator == ESTIMATOR::Line2d) {
        draw_line(model, points, img_name1);

    } else if (model->estimator == ESTIMATOR::Essential) {
        drawEpipolarLines(model, points, img_name1, img_name2);

    } else if (model->estimator == ESTIMATOR::Fundamental) {
        drawEpipolarLines(model, points, img_name1, img_name2);

    } else if (model->estimator == ESTIMATOR::Homography) {
        drawHomography(model, points, img_name1, img_name2);

    }
}

void cv::usac::draw::drawing_resize(cv::Mat &image) {
    float nS = 700000; // 600 x 800
    cv::resize(image, image,
               cv::Size(sqrt((image.cols * nS) / image.rows), sqrt((image.rows * nS) / image.cols)));
}

void cv::usac::draw::showInliers(cv::InputArray input_points, cv::InputArray input_inliers_idxes, cv::Mat &image) {
    int *inliers_idxes = (int *) input_inliers_idxes.getMat().data;
    cv::Point_<float> *points = (cv::Point_<float> *) input_points.getMat().data;

    int inliers_size = input_inliers_idxes.size().width;
    for (int i = 0; i < inliers_size; i++) {
        circle(image, points[inliers_idxes[i]], 3, cv::Scalar(20, 90, 250), -1);
    }
}

void cv::usac::draw::draw_line_kx_b(float k, float b, const cv::Scalar &color, cv::Mat img) {
    int max_dimen = std::max(img.cols, img.rows);
    float corner_x1 = max_dimen;
    float corner_y1 = k * corner_x1 + b;

    float corner_x2 = -max_dimen;
    float corner_y2 = k * corner_x2 + b;
    cv::line(img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color, 2, 8);
}

void cv::usac::draw::draw_line_model(Model *const model, cv::Scalar color, cv::Mat img, bool threshold) {
    cv::Mat desc = model->returnDescriptor();
    auto *params = reinterpret_cast<float *>(desc.data);
    // // ax + by + c = 0
    // // y = kx + l, k = -a*x/b, l = -c/b
    float l = -params[2] / params[1];
    float k = -params[0] / params[1];

    draw_line_kx_b(k, l, color, img);

    if (threshold) {
        draw_line_kx_b(k, l + sqrt(pow(model->threshold, 2) * pow(k, 2) + pow(model->threshold, 2)),
                       cv::Scalar(0, 0, 255), img);
        draw_line_kx_b(k, l - sqrt(pow(model->threshold, 2) * pow(k, 2) + pow(model->threshold, 2)),
                       cv::Scalar(0, 0, 255), img);
    }
}

void cv::usac::draw::draw_line(Model *const model, const cv::Mat &points, const std::string &img_name) {
    cv::Mat image = cv::imread(img_name);
    cv::usac::Line2DEstimator est(points);
    std::vector<int> inliers;
    cv::usac::Quality::getInliers(&est, model->returnDescriptor(), model->threshold, points.rows, inliers);
    showInliers(points, inliers, image);
    draw_line_model(model, cv::Scalar(255, 0, 0), image, true);

    drawing_resize(image);
    imshow("Inliers", image);
    cv::waitKey(0);
}

void cv::usac::draw::drawHomography(Model *model, const cv::Mat &points,
                           const std::string &img_name1,
                           const std::string &img_name2) {
    cv::Mat points1 = points.colRange(0,2);
    cv::Mat points2 = points.colRange(2,4);
    cv::hconcat(points1, cv::Mat_<float>::ones(points1.rows, 1), points1);
    cv::hconcat(points2, cv::Mat_<float>::ones(points1.rows, 1), points2);

    cv::Mat img1 = cv::imread(img_name1);
    cv::Mat img2 = cv::imread(img_name2);

    // draw matches
//        cv::Mat img_matches, gt_img_matches;
//        drawMatches(img_matches, img1, img2, points1, points2, inliers, inliers_size);
//        drawMatches(gt_img_matches, img1, img2, points1, points2, &gt_inliers[0], gt_inliers.size());
//        cv::vconcat(img_matches, gt_img_matches, img_matches);
    // ----------------------

    drawErrors(img1, img2, points1, points2, model->returnDescriptor());

    cv::hconcat(img1, img2, img1);

    drawing_resize(img1);
    cv::imshow("homography matrix estimation", img1);

    // draw panorama
    cv::Mat panorama;
    std::vector<cv::Mat> images;
    images.push_back(img1);
    images.push_back(img2);
    drawPanorama(images, panorama, model->returnDescriptor().inv());

//        drawing_resize(img_matches);
//        drawing_resize(panorama);
//        cv::imshow("panorama", panorama);
//        cv::imshow("imgs ", img_matches);

    cv::waitKey(0);
}

void
cv::usac::draw::drawMatches(cv::Mat &img_matches, const cv::Mat &img1, const cv::Mat &img2, const cv::Mat &points1,
            const cv::Mat &points2, const int *const inliers, int inliers_size) {
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    mat2keypoint(points1, keypoints1);
    mat2keypoint(points2, keypoints2);

    for (int i = 0; i < inliers_size; i++) {
        cv::DMatch match(inliers[i], inliers[i], 0);
        good_matches.push_back(match);
    }

    cv::drawMatches(img1, keypoints1, img2, keypoints2,
                    good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
}

void cv::usac::draw::drawErrors(cv::Mat &img1_error, cv::Mat &img2_error, const cv::Mat &points1, const cv::Mat &points2,
           const cv::Mat &H) {
    cv::Mat points1_t, points2_t;
    cv::transpose(points1, points1_t);
    cv::transpose(points2, points2_t);

    cv::Mat est_pts_on_corr2 = H * points1_t;
    cv::Mat est_pts_on_corr1 = H.inv() * points2_t;

    cv::divide(est_pts_on_corr1.row(0), est_pts_on_corr1.row(2), est_pts_on_corr1.row(0), 1);
    cv::divide(est_pts_on_corr1.row(1), est_pts_on_corr1.row(2), est_pts_on_corr1.row(1), 1);
    cv::divide(est_pts_on_corr2.row(0), est_pts_on_corr2.row(2), est_pts_on_corr2.row(0), 1);
    cv::divide(est_pts_on_corr2.row(1), est_pts_on_corr2.row(2), est_pts_on_corr2.row(1), 1);

    for (int i = 0; i < points1.rows; i++) {
        cv::Scalar color = cv::Scalar(random() % 255, random() % 255, random() % 255);

        cv::line(img1_error, cv::Point_<float>(points1.at<float>(i, 0), points1.at<float>(i, 1)),
                 cv::Point_<float>(est_pts_on_corr1.at<float>(0, i), est_pts_on_corr1.at<float>(1, i)),
                 color, 2);

        cv::line(img2_error, cv::Point_<float>(points2.at<float>(i, 0), points2.at<float>(i, 1)),
                 cv::Point_<float>(est_pts_on_corr2.at<float>(0, i), est_pts_on_corr2.at<float>(1, i)),
                 color, 2);

        cv::circle(img1_error, cv::Point_<float>(points1.at<float>(i, 0), points1.at<float>(i, 1)), 3,
                   cv::Scalar(0, 255, 0), -1);
        cv::circle(img2_error, cv::Point_<float>(points2.at<float>(i, 0), points2.at<float>(i, 1)), 3,
                   cv::Scalar(0, 255, 0), -1);

        cv::circle(img1_error,
                   cv::Point_<float>(est_pts_on_corr1.at<float>(0, i), est_pts_on_corr1.at<float>(1, i)), 3,
                   cv::Scalar(0, 0, 255), -1);
        cv::circle(img2_error,
                   cv::Point_<float>(est_pts_on_corr2.at<float>(0, i), est_pts_on_corr2.at<float>(1, i)), 3,
                   cv::Scalar(0, 0, 255), -1);
    }
}

void cv::usac::draw::mat2keypoint(const cv::Mat &points, std::vector<cv::KeyPoint> &keypoints) {
    for (int i = 0; i < points.rows; i++) {
        cv::KeyPoint kp(points.at<float>(i, 0), points.at<float>(i, 1), 1);
        keypoints.push_back(kp);
    }
}
