#ifndef USAC_ESSENTIALESTIMATOR_H
#define USAC_ESSENTIALESTIMATOR_H

#include "Estimator.h"

class EssentialEstimator : public Estimator {
private:
    const float * const points;
    cv::Mat E, E_inv;
    float *E_ptr, *E_inv_ptr;
public:

    /*
     * input_points must be:
     * img1_x1 img1_y1 img2_x1 img2_y1
     * img1_x2 img1_y2 img2_x2 img2_y2
     * ....
     * img1_xN img1_yN img2_xN img2_yN
     */

    EssentialEstimator(cv::InputArray input_points) : points((float *)input_points.getMat().data) {
        assert(!input_points.empty());
    }

    void setModelParameters (Model * const model) override {
        E = cv::Mat_<float>(model->returnDescriptor());
        E_inv = E.inv();

        /*
         * To make pointer from Mat class, this Mat class should exists as long as exists pointer
         * So this->E and this->E_inv must be global in class
         */
        E_ptr = (float *) E.data;
        E_inv_ptr = (float *) E_inv.data;
    }

    unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        return 1;
    }

    bool EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        return true;
    }

    float GetError(int pidx) override {
        float error = 0;
        unsigned int smpl = 4*pidx;
        float x1 = points[smpl];
        float y1 = points[smpl+1];
        float x2 = points[smpl+2];
        float y2 = points[smpl+3];

        return error;
    }

    int SampleNumber() override {
        return 5;
    }
};

#endif //USAC_ESSENTIALESTIMATOR_H
