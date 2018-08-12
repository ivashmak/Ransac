#ifndef RANSAC_DRAWING_H
#define RANSAC_DRAWING_H


#include "Estimator.h"

class Drawing {
public:
    void showInliers (cv::InputArray input_points, cv::InputArray input_inliers_idxes) {
        cv::Mat image = cv::imread("../data/image1.jpg");
        int *inliers_idxes = (int *) input_inliers_idxes.getMat().data;
        cv::Point_<float> *points = (cv::Point_<float> *) input_points.getMat().data;

        int inliers_size = input_inliers_idxes.size().width;
        for (int i = 0; i < inliers_size; i++) {
            circle(image, points[inliers_idxes[i]], 3, cv::Scalar(20, 90, 250), -1);
        }

        imshow("Inliers", image);
        cv::waitKey (0);
    }

    void showResult (Model& model, cv::InputArray ps, cv::InputArray sample) {
        cv::Point_<float> *points = (cv::Point_<float> *) ps.getMat().data;

        int *line_idx = reinterpret_cast<int *>(sample.getMat().data);
        cv::Point_<float> p1 = points[line_idx[0]];
        cv::Point_<float> p2 = points[line_idx[1]];

        cv::Mat image = cv::imread("../data/image1.jpg");

        CV_Assert(image.depth() == CV_8U);
        CV_Assert(!ps.empty());

        int width = image.cols;
        int height = image.rows;

        float k = (p2.x - p1.x)/(p2.y - p1.y);
        float b = (p1.y*p1.x - p1.y*p2.x)/(p2.y-p1.y) + p1.x;

        draw_function (k, b-sqrt(pow(model.threshold,2)*pow(k,2)+pow(model.threshold,2)), std::max(width, height), cv::Scalar(0,255,0), image);
        draw_function (k, b, std::max(width, height), cv::Scalar(255,0,0), image);
        draw_function (k, b+sqrt(pow(model.threshold,2)*pow(k,2)+pow(model.threshold,2)), std::max(width, height), cv::Scalar(0,255,0), image);

        int total_points = ps.size().width;
        float dist;

        for (int kp = 0; kp < total_points; kp++) {
            dist = abs((p2.y-p1.y)*points[kp].x -
                       (p2.x-p1.x)*points[kp].y +
                       p2.x*p1.y - p2.y*p1.x)/sqrt(pow(p2.y-p1.y,2)+pow(p2.x-p1.x,2));

            if (dist < model.threshold) {
                circle(image, points[kp], 3, cv::Scalar(0, 0, 255), -1);
            }
        }
        imshow("Best Line", image);
        cv::waitKey (0);
    }

    void draw_function (float k, float b, float max_dimen, cv::Scalar color, cv::Mat img) {
        float corner_y1 = max_dimen;
        float corner_x1 = k*max_dimen+b;

        float corner_y2 = -max_dimen;
        float corner_x2 = k*(-max_dimen)+b;
        cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
    }

};


#endif //RANSAC_DRAWING_H
