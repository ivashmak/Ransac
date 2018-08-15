#ifndef RANSAC_DRAWING_H
#define RANSAC_DRAWING_H


#include "Estimator.h"

class Drawing {
public:
    void showInliers (cv::InputArray input_points, cv::InputArray input_inliers_idxes, cv::Mat image) {
        int *inliers_idxes = (int *) input_inliers_idxes.getMat().data;
        cv::Point_<float> *points = (cv::Point_<float> *) input_points.getMat().data;

        int inliers_size = input_inliers_idxes.size().width;
        for (int i = 0; i < inliers_size; i++) {
            circle(image, points[inliers_idxes[i]], 3, cv::Scalar(20, 90, 250), -1);
        }
    }

    void draw_model (Model model, float max_dimen, cv::Scalar color, cv::Mat img) {
        cv::Mat desc;
        model.getDescriptor(desc);
        auto * params = reinterpret_cast<float *>(desc.data);
        std::cout <<"model = "<< params[0] << " " << params[1] << " " <<params[2] << '\n';
        float b = -params[2]/params[0];
        float k = -params[1]/params[0];

        float corner_y1 = max_dimen;
        float corner_x1 = k*max_dimen+b;

        float corner_y2 = -max_dimen;
        float corner_x2 = k*(-max_dimen)+b;
        cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
    }

};


#endif //RANSAC_DRAWING_H
