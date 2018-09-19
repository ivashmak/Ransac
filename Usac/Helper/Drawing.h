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

    void draw_model (Model * const model, float max_dimen, cv::Scalar color, cv::Mat img, bool threshold) {
        cv::Mat desc;
        model->getDescriptor(desc);
        auto * params = reinterpret_cast<float *>(desc.data);
        std::cout <<"model: a = "<< params[0] << " b = " << params[1] << " c = " <<params[2] << '\n';
        float b = -params[2]/params[0];
        float k = -params[1]/params[0];

        draw_line(k, b, color, img);

        if (threshold) {
            draw_line (k, b+sqrt(pow(model->threshold,2)*pow(k,2)+pow(model->threshold,2)), cv::Scalar(0,0,255), img);
            draw_line (k, b-sqrt(pow(model->threshold,2)*pow(k,2)+pow(model->threshold,2)), cv::Scalar(0,0,255), img);
        }

    }

    /*
     * Read image.
     * Show inliers of best ransac model.
     * Show inliers of non minimal best model.
     * To show threshold lines change false to true.
     */
    void draw (cv::InputArray inliers, Model * const best_model, Model * const non_minimal_model, cv::InputArray points) {
        cv::Mat image  = cv::imread("../images/image1.jpg");
        showInliers(points, inliers, image);
        draw_model(best_model, std::max (image.cols, image.rows), cv::Scalar(255, 0, 0), image, false);
        draw_model(non_minimal_model, std::max (image.cols, image.rows), cv::Scalar(0, 255, 0), image, false);
        imshow("Inliers", image);
        std::string filename = "../res/linefitting_"+best_model->model_name+".jpg";
        cv::imwrite(filename, image);
        cv::waitKey (0);
    }

    // Homographies

    /*
     * Draw epipolar lines by Fundamental Matrix
     */
    void drawEpipolarLines (cv::InputArray points1, cv::InputArray points2, const cv::Mat& F) {

        cv::Mat img1 = cv::imread("../images/img1.png"), img2 = cv::imread("../images/img2.png");

        cv::Mat pts1 = points1.getMat(), pts2 = points2.getMat();
        cv::hconcat(pts1, cv::Mat_<float>::ones (pts1.rows, 1), pts1);
        cv::hconcat(pts2, cv::Mat_<float>::ones (pts2.rows, 1), pts2);

        cv::Mat lines1, lines2;
        cv::computeCorrespondEpilines(pts1, 1, F, lines1);
        cv::computeCorrespondEpilines(pts2, 2, F, lines2);

        int c = img1.cols, r = img1.rows;
        float x0, y0_img1, x1, y1_img1, r0_img1, r1_img1, r2_img1;
        float  y0_img2, y1_img2, r0_img2, r1_img2, r2_img2;
        for (int i = 0; i < points1.rows(); i++) {
            r0_img1 = lines1.at<float>(i, 0);
            r1_img1 = lines1.at<float>(i, 1);
            r2_img1 = lines1.at<float>(i, 2);

            r0_img2 = lines2.at<float>(i, 0);
            r1_img2 = lines2.at<float>(i, 1);
            r2_img2 = lines2.at<float>(i, 2);

            x0 = 0;
            y0_img1 = -r2_img1/r1_img1;
            y0_img2 = -r2_img2/r1_img2;
            x1 = c;
            y1_img1 = -(r2_img1 + r0_img1*c)/r1_img1;
            y1_img2 = -(r2_img2 + r0_img2*c)/r1_img2;

            cv::Scalar color = cv::Scalar(rand()%255, rand ()%255, rand()%255);
            cv::line(img1, cv::Point_<float> (x0, y0_img1), cv::Point_<float> (x1, y1_img1), color);
            cv::line(img2, cv::Point_<float> (x0, y0_img2), cv::Point_<float> (x1, y1_img2), color);
            circle (img1, cv::Point_<float> (pts1.at<float>(i, 0), pts1.at<float>(i,1)), 3, color, -1);
            circle (img2, cv::Point_<float> (pts2.at<float>(i, 0), pts2.at<float>(i,1)), 3, color, -1);
        }

        imshow("Epipolar lines using Fundamental matrix 1", img1);
        imshow("Epipolar lines using Fundamental matrix 2", img2);
        cv::waitKey (0);
    }
};


#endif //RANSAC_DRAWING_H
