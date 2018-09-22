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

        //cvmSet
        for (int i = 0; i < 4; i++) {
            points1.at<float>(i,0) = set_points[0].at<float>(idxs[i], 0);
            points1.at<float>(i,1) = set_points[0].at<float>(idxs[i], 1);

            points2.at<float>(i,0) = set_points[1].at<float>(idxs[i], 0);
            points2.at<float>(i,1) = set_points[1].at<float>(idxs[i], 1);
        }

//        std::cout << "calling Normalized DLT\n";
        NormalizedDLT(points1, points2, H);
//        DLT(points1, points2, H);
//        std::cout << "get Normalized DLT\n";
        this->H = H;
        this->H_inv = H.inv();

        model.setDescriptor(H);
    }

    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {

    }

    float GetError(int pidx) override {
        float error = 0;
        cv::Mat one = (cv::Mat_<float>(1,1) << 1);

        cv::Mat corr1_pt = set_points[0].row(pidx);
        cv::Mat corr2_pt = set_points[1].row(pidx);

        cv::hconcat(corr1_pt, one, corr1_pt);
        cv::hconcat(corr2_pt, one, corr2_pt);

        cv::Mat est_pt_on_corr2 = corr1_pt*H;
        cv::Mat est_pt_on_corr1 = corr2_pt*H_inv;

        for (int i = 0; i < num_correspondeces - 1; i++) {
            for (int j = i + 1; j < num_correspondeces; j++) {
                // compare ith corr with jth corr
                // e.g. num_corr = 4
                // 1-2, 1-3, 1-4, 2-3, 2-4, 3-4
            }
        }

//        cv::Mat img1 = cv::imread("../images/homography/grafA.png");
//        cv::Mat img2 = cv::imread("../images/homography/grafB.png");
//
//        circle(img1, cv::Point2f(corr1_pt.at<float>(0), corr1_pt.at<float>(1)), 3, cv::Scalar(250, 250, 0), -1);
//        circle(img2, cv::Point2f(corr2_pt.at<float>(0), corr2_pt.at<float>(1)), 3, cv::Scalar(250, 250, 0), -1);
//
//        circle(img1, cv::Point2f(est_pt_on_corr1.at<float>(0), est_pt_on_corr1.at<float>(1)), 3, cv::Scalar(250, 0, 0), -1);
//        circle(img2, cv::Point2f(est_pt_on_corr2.at<float>(0), est_pt_on_corr2.at<float>(1)), 3, cv::Scalar(250, 0, 0), -1);
//
//        std::cout << corr1_pt << "\n";
//        std::cout << est_pt_on_corr1 << "\n";
//        std::cout << corr2_pt << "\n";
//        std::cout << est_pt_on_corr2 << "\n";
//
//        cv::hconcat(img1, img2, img1);
//        cv::resize(img1, img1, cv::Size (img1.cols * 0.75, img1.rows  * 0.75));
//        cv::imshow ("img1", img1);
//
//        cv::waitKey(0);

        // error = d(p(i)H, p'(i)) + d(p(i), p'(i)H^-1)
        error += sqrt ((corr2_pt.at<float>(0) - est_pt_on_corr2.at<float>(0)) * (corr2_pt.at<float>(0) - est_pt_on_corr2.at<float>(0)) +
                       (corr2_pt.at<float>(1) - est_pt_on_corr2.at<float>(1)) * (corr2_pt.at<float>(1) - est_pt_on_corr2.at<float>(1)));

        error += sqrt ((corr1_pt.at<float>(0) - est_pt_on_corr1.at<float>(0)) * (corr1_pt.at<float>(0) - est_pt_on_corr1.at<float>(0)) +
                       (corr1_pt.at<float>(1) - est_pt_on_corr1.at<float>(1)) * (corr1_pt.at<float>(1) - est_pt_on_corr1.at<float>(1)));

//        std::cout << "error = " << error << '\n';
        return error;
    }

    int SampleNumber() override {
        return 4;
    }
};


#endif //RANSAC_HOMOGRAPHYESTIMATOR_H
