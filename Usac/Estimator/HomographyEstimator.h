#ifndef RANSAC_HOMOGRAPHYESTIMATOR_H
#define RANSAC_HOMOGRAPHYESTIMATOR_H


#include "Estimator.h"

class HomographyEstimator : public Estimator{
private:
    cv::Mat H;
    unsigned int num_correspondeces;
    // K images x (N x 3) points
    std::vector<cv::Mat> set_points;
public:

    void setPoints (cv::InputArray input_points) override {
//        std::cout << input_points.size().width << '\n';
        for (int i = 0; i < input_points.size().width; i++) {
            cv::Mat points = input_points.getMat(i);
//            std::cout << points << '\n';
            set_points.push_back(points);
        }
    }

    void EstimateModel(const int * const sample, Model &model) override {
        int idxs[4];
        idxs[0] = sample[0];
        idxs[1] = sample[1];
        idxs[2] = sample[2];
        idxs[3] = sample[3];

        cv::Mat H;
        cv::Mat_<float>  points1(4,2), points2(4,2);

        for (int i = 0; i < 4; i++) {
            points1.at<float>(i,0) = set_points[0].at<float>(idxs[i], 0);
            points1.at<float>(i,1) = set_points[0].at<float>(idxs[i], 1);

            points2.at<float>(i,0) = set_points[1].at<float>(idxs[i], 0);
            points2.at<float>(i,1) = set_points[1].at<float>(idxs[i], 1);
        }

        std::cout << points1 << '\n';
        std::cout << points2 << '\n';

        NormalizedDLT(points1, points2, H);
    }

    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {

    }

    float GetError(int pidx) override {
        float error = 0;
        for (int i = 0; i < num_correspondeces; i++) {
            cv::Mat estimated_point = set_points[i]*H; // 1x3 * 3x3 = 1x3
            // declare variables to avoid cache miss
            float x = (set_points[i].at<float>(0,0) - estimated_point.at<float>(0,0));
            float y = (set_points[i].at<float>(0,1) - estimated_point.at<float>(0,1));
//            float z = (set_points[i].at<float>(0,0) - estimated_point.at<float>(0,0)); // is zero

            error += x*x + y*y;
        }
        return error;
    }

    int SampleNumber() override {
        return 4;
    }
};


#endif //RANSAC_HOMOGRAPHYESTIMATOR_H
