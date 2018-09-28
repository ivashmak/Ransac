#ifndef USAC_FUNDAMENTALESTIMATOR_H
#define USAC_FUNDAMENTALESTIMATOR_H

#include "Estimator.h"

class FundamentalEstimator : public Estimator {
private:
    const float * const points;
    cv::Mat F, F_inv;
    float *F_ptr, *F_inv_ptr;
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

    FundamentalEstimator(cv::InputArray input_points) : points((float *)input_points.getMat().data) {
        assert(!input_points.empty());
    }

    void setModelParameters (Model * const model) override {
        F = cv::Mat_<float>(model->returnDescriptor());
        F_inv = F.inv();

        /*
         * Attention!
         * To make pointer from Mat class, this Mat class should exists as long as exists pointer
         * So this->F and this->F_inv must be global in class
         */
        F_ptr = (float *) F.data;
        F_inv_ptr = (float *) F_inv.data;
    }

    void EstimateModel(const int * const sample, Model &model) override {
        
    }

    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {

    }

    float GetError(int pidx) override {
        float error = 0;
        unsigned int smpl = 4*pidx;
        float x1 = points[smpl];
        float y1 = points[smpl+1];
        float x2 = points[smpl+2];
        float y2 = points[smpl+3];
        
        float est_x2 = F_ptr[0] * x1 + F_ptr[1] * y1 + F_ptr[2];
        float est_y2 = F_ptr[3] * x1 + F_ptr[4] * y1 + F_ptr[5];
        float est_z2 = F_ptr[6] * x1 + F_ptr[7] * y1 + F_ptr[8];

        est_x2 /= est_z2;
        est_y2 /= est_z2;

        float est_x1 = F_inv_ptr[0] * x2 + F_inv_ptr[1] * y2 + F_inv_ptr[2];
        float est_y1 = F_inv_ptr[3] * x2 + F_inv_ptr[4] * y2 + F_inv_ptr[5];
        float est_z1 = F_inv_ptr[6] * x2 + F_inv_ptr[7] * y2 + F_inv_ptr[8];

        est_x1 /= est_z1;
        est_y1 /= est_z1;

        // error = d(p(i)F, p'(i)) + d(p(i), p'(i)F^-1)
        error += sqrt ((x2 - est_x2) * (x2 - est_x2) + (y2 - est_y2) * (y2 - est_y2));
        error += sqrt ((x1 - est_x1) * (x1 - est_x1) + (y1 - est_y1) * (y1 - est_y1));

        return error/2;
    }

    int SampleNumber() override {
        return 7;
    }
};

#endif //USAC_FUNDAMENTALESTIMATOR_H
