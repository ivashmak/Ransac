#ifndef RANSAC_HOMOGRAPHYESTIMATOR_H
#define RANSAC_HOMOGRAPHYESTIMATOR_H

#include "Estimator.h"
#include "../DLT/NormalizedDLT.h"

class HomographyEstimator : public Estimator{
private:
    cv::Mat H;
    cv::Mat H_inv;
    const float * const points;
//    float *samples1_ptr, *samples2_ptr;
public:

    /*
     * input_points must be:
     * img1_x1 img1_y1 img2_x1 img2_y1
     * img1_x2 img1_y2 img2_x2 img2_y2
     * ....
     * img1_xN img1_yN img2_xN img2_yN
     *
     * Size N x (2*|imgs|)
     *
     *
     * float array 4N x 1
     * img1_x1
     * img1_y1
     * img2_x1
     * img2_y1
     * img1_x2
     * img1_y2
     * img2_x2
     * img2_y2
     * ...
     * img1_xN
     * img1_yN
     * img2_xN
     * img2_yN
     */

    HomographyEstimator(cv::InputArray input_points) : points((float *)input_points.getMat().data) {
        assert(!input_points.empty());
    }

    void setModelParameters (Model * const model) override {
        H = cv::Mat_<float>(model->returnDescriptor());
        H_inv = H.inv();
    }

    void EstimateModel(const int * const sample, Model &model) override {
        cv::Mat H;
        DLT (points, sample, 4, H);

//        std::cout << "H = \n" << H << "\n\n--------------------------\n";

        this->H = H;
        this->H_inv = H.inv();

        model.setDescriptor(H);
    }

    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        cv::Mat H;

        NormalizedDLT(points, sample, sample_size, H);

        this->H = H;
        this->H_inv = H.inv();

        model.setDescriptor(H);
    }

    float GetError(int pidx) override {
        float error = 0;

        float x1 = points[4*pidx];
        float y1 = points[4*pidx+1];
        float x2 = points[4*pidx+2];
        float y2 = points[4*pidx+3];

//        std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << '\n';

        float est_x2 = H.at<float>(0,0) * x1 + H.at<float>(0,1) * y1 + H.at<float>(0,2);
        float est_y2 = H.at<float>(1,0) * x1 + H.at<float>(1,1) * y1 + H.at<float>(1,2);
        float est_z2 = H.at<float>(2,0) * x1 + H.at<float>(2,1) * y1 + H.at<float>(2,2);

        est_x2 /= est_z2;
        est_y2 /= est_z2;

        float est_x1 = H_inv.at<float>(0,0) * x2 + H_inv.at<float>(0,1) * y2 + H_inv.at<float>(0,2);
        float est_y1 = H_inv.at<float>(1,0) * x2 + H_inv.at<float>(1,1) * y2 + H_inv.at<float>(1,2);
        float est_z1 = H_inv.at<float>(2,0) * x2 + H_inv.at<float>(2,1) * y2 + H_inv.at<float>(2,2);

        est_x1 /= est_z1;
        est_y1 /= est_z1;

        // error = d(p(i)H, p'(i)) + d(p(i), p'(i)H^-1)
        error += sqrt ((x2 - est_x2) * (x2 - est_x2) + (y2 - est_y2) * (y2 - est_y2));
        error += sqrt ((x1 - est_x1) * (x1 - est_x1) + (y1 - est_y1) * (y1 - est_y1));

        return error/2;
    }

    int SampleNumber() override {
        return 4;
    }
};


#endif //RANSAC_HOMOGRAPHYESTIMATOR_H