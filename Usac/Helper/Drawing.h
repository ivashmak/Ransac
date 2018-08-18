#ifndef RANSAC_DRAWING_H
#define RANSAC_DRAWING_H

#include "../Estimator/Estimator.h"
#include "../Ransac/Ransac.h"

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

    void draw_line (float k, float b, cv::Scalar color, cv::Mat img) {
        int max_dimen = std::max (img.cols, img.rows);
        float corner_y1 = max_dimen;
        float corner_x1 = k*max_dimen + b;

        float corner_y2 = -max_dimen;
        float corner_x2 = k*(-max_dimen) + b;
        cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
    }

    void draw_model (Model model, float max_dimen, cv::Scalar color, cv::Mat img, bool threshold) {
        cv::Mat desc;
        model.getDescriptor(desc);
        auto * params = reinterpret_cast<float *>(desc.data);
        std::cout <<"model = "<< params[0] << " " << params[1] << " " <<params[2] << '\n';
        float b = -params[2]/params[0];
        float k = -params[1]/params[0];

        draw_line(k, b, color, img);

        if (threshold) {
            draw_line (k, b+sqrt(pow(model.threshold,2)*pow(k,2)+pow(model.threshold,2)), cv::Scalar(0,0,255), img);
            draw_line (k, b-sqrt(pow(model.threshold,2)*pow(k,2)+pow(model.threshold,2)), cv::Scalar(0,0,255), img);
        }

    }

    void draw (cv::InputArray inliers, Model best_model, Model non_minimal_model, cv::InputArray points) {
        cv::Mat image = cv::imread("../images/image1.jpg");
        showInliers(points, inliers, image);
        draw_model(best_model, std::max (image.cols, image.rows), cv::Scalar(255, 0, 0), image, false);
        draw_model(non_minimal_model, std::max (image.cols, image.rows), cv::Scalar(0, 255, 0), image, false);
        imshow("Inliers", image);
        std::string filename = "../res/linefitting_"+best_model.model_name+".jpg";
        cv::imwrite(filename, image);
        cv::waitKey (0);
    }
};


#endif //RANSAC_DRAWING_H
