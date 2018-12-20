#ifndef USAC_ESSENTIALESTIMATOR_H
#define USAC_ESSENTIALESTIMATOR_H

#include "Estimator.h"
#include "Fundamental/FundemantalSolver.h"
#include "Essential/FivePoints.h"

class EssentialEstimator : public Estimator {
private:
    const float * const points;
    cv::Mat E;
    float * E_ptr;
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

    void setModelParameters (const cv::Mat& model) override {
        E = cv::Mat_<float>(model);
//        E = model;

        /*
         * To make pointer from Mat class, this Mat class should exists as long as exists pointer
         * So this->E and this->E_inv must be global in class
         */
        E_ptr = (float *) E.data;
    }

    /*
     * E = K1^T F K2
     *
     * y'^T E y = 0, normalized points by third coordinate.
     * y' = (x'1 x'2 x'3) / x'3
     * y  = (x1  x2  x3)  / x3
     */
    unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        cv::Mat_<float> E;

        unsigned int models_count = FivePoints (points, sample, E);
//        unsigned int models_count2 = FivePointsOpenCV (points, sample, E2);

        std::cout << "models count " << models_count << "\n";

        if (models_count == 0) {
            return 0;
        }

        // todo: fix for more than 3 solutions
        for (int i = 0; i < std::min ((unsigned int)3, models_count); i++) {
            models[i]->setDescriptor(E.rowRange(i * 3, i * 3 + 3));
        }

        return models_count;
    }

    bool EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        cv::Mat_<float> E;

        if (! EightPointsAlgorithm(points, sample, sample_size, E)) {
            return false;
        }

        model.setDescriptor(E);

        return true;
    }

    float GetError(int pidx) override {
        unsigned int smpl = 4*pidx;
        float x1 = points[smpl];
        float y1 = points[smpl+1];
        float x2 = points[smpl+2];
        float y2 = points[smpl+3];

        // pt2 E, line 1
        float l1 = *(E_ptr)* x2 + *(E_ptr + 3) * y2 + *(E_ptr + 6);
        float l2 = *(E_ptr + 1) * x2 + *(E_ptr + 4) * y2 + *(E_ptr + 7);
        float l3 = *(E_ptr + 2) * x2 + *(E_ptr + 5) * y2 + *(E_ptr + 8);

        // E pt1, line 2
        float t1 = *(E_ptr)* x1 + *(E_ptr + 1) * y1 + *(E_ptr + 2);
        float t2 = *(E_ptr + 3) * x1 + *(E_ptr + 4) * y1 + *(E_ptr + 5);
        float t3 = *(E_ptr + 6) * x1 + *(E_ptr + 7) * y1 + *(E_ptr + 8);

        // distance from pt1 to line 1
        float a1 = l1 * x1 + l2 * y1 + l3;
        float a2 = sqrt(l1 * l1 + l2 * l2);

        // distance from pt2 to line 2
        float b1 = t1 * x2 + t2 * y2 + t3;
        float b2 = sqrt(t1 * t1 + t2 * t2);

        // normalized distance
        // abs (d1 + d2) / 2
        return fabsf((a1 / a2) + (b1 / b2)) / 2;
    }

    int SampleNumber() override {
        return 5;
    }
};

#endif //USAC_ESSENTIALESTIMATOR_H
