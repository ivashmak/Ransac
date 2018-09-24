#ifndef RANSAC_HOMOGRAPHYESTIMATOR_H
#define RANSAC_HOMOGRAPHYESTIMATOR_H


#include "Estimator.h"
#include "../DLT/NormalizedDLT.h"

class HomographyEstimator : public Estimator{
private:
    cv::Mat H;
    cv::Mat H_inv;
    unsigned int num_correspondeces;
    // K images x (N x 3) points
    std::vector<cv::Mat> set_points;
public:

    void setPoints (cv::InputArray input_points) override {
        num_correspondeces = input_points.size().width;

        for (int i = 0; i < input_points.size().width; i++) {
            cv::Mat points = input_points.getMat(i);
            if (points.cols == 2) {
                /*
                 * add third column of ones, because H is 3x3
                 * transpose points, because H*pt = 3x3 * 3x1
                 * or H*pts = 3x3 * 3xN
                 */
                cv::Mat ones = cv::Mat_<float>::ones (points.rows, 1);
                cv::hconcat(points, ones, points);
                cv::transpose(points, points);
            }
            set_points.push_back(points);
        }
    }

    void setModelParametres (Model * const model) override {
        H = cv::Mat_<float>(model->returnDescriptor());
        H_inv = H.inv();
    }

    void EstimateModel(const int * const sample, Model &model) override {
        int idxs[4];
        idxs[0] = sample[0];
        idxs[1] = sample[1];
        idxs[2] = sample[2];
        idxs[3] = sample[3];

//        std::cout << idxs[0] << " " << idxs[1] << " " << idxs[2] << " " << idxs[3] << '\n';

        cv::Mat H;
        cv::Mat_<float>  points1(4,2), points2(4,2);

        for (int i = 0; i < 4; i++) {
            points1.at<float>(i,0) = set_points[0].at<float>(0, idxs[i]);
            points1.at<float>(i,1) = set_points[0].at<float>(1, idxs[i]);

            points2.at<float>(i,0) = set_points[1].at<float>(0, idxs[i]);
            points2.at<float>(i,1) = set_points[1].at<float>(1, idxs[i]);
        }

        NormalizedDLT(points1, points2, H);

        this->H = H;
        this->H_inv = H.inv();

        model.setDescriptor(H);
    }

    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {

    }

    float GetError(int pidx) override {
        float error = 0;
        cv::Mat corr1_pt = set_points[0].col(pidx);
        cv::Mat corr2_pt = set_points[1].col(pidx);

        cv::Mat est_pt_on_corr2 = H * corr1_pt;
        cv::Mat est_pt_on_corr1 = H_inv * corr2_pt;

        est_pt_on_corr1 /= est_pt_on_corr1.at<float>(2);
        est_pt_on_corr2 /= est_pt_on_corr2.at<float>(2);

//        for (int i = 0; i < num_correspondeces - 1; i++) {
//            for (int j = i + 1; j < num_correspondeces; j++) {
//                // compare ith corr with jth corr
//                // e.g. num_corr = 4
//                // 1-2, 1-3, 1-4, 2-3, 2-4, 3-4
//            }
//        }

        // error = d(p(i)H, p'(i)) + d(p(i), p'(i)H^-1)
        error += sqrt ((corr2_pt.at<float>(0) - est_pt_on_corr2.at<float>(0)) * (corr2_pt.at<float>(0) - est_pt_on_corr2.at<float>(0)) +
                       (corr2_pt.at<float>(1) - est_pt_on_corr2.at<float>(1)) * (corr2_pt.at<float>(1) - est_pt_on_corr2.at<float>(1)));

        error += sqrt ((corr1_pt.at<float>(0) - est_pt_on_corr1.at<float>(0)) * (corr1_pt.at<float>(0) - est_pt_on_corr1.at<float>(0)) +
                       (corr1_pt.at<float>(1) - est_pt_on_corr1.at<float>(1)) * (corr1_pt.at<float>(1) - est_pt_on_corr1.at<float>(1)));

        return error;
    }

    int SampleNumber() override {
        return 4;
    }
};


#endif //RANSAC_HOMOGRAPHYESTIMATOR_H