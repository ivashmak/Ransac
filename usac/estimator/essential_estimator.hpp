// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_ESSENTIALESTIMATOR_H
#define USAC_ESSENTIALESTIMATOR_H

#include "estimator.hpp"
#include "fundamental/fundamental_solver.hpp"
#include "essential/five_points.hpp"

class EssentialEstimator : public Estimator {
private:
    const float * const points;
    float e11, e12, e13, e21, e22, e23, e31, e32, e33;
    EssentialSolver * e_solver;
    FundamentalSolver * f_solver;
public:
    ~EssentialEstimator () {
        delete (e_solver);
        delete (f_solver);
    }
    /*
     * input_points must be:
     * img1_x1 img1_y1 img2_x1 img2_y1
     * img1_x2 img1_y2 img2_x2 img2_y2
     * ....
     * img1_xN img1_yN img2_xN img2_yN
     */

    EssentialEstimator(cv::InputArray input_points) : points((float *)input_points.getMat().data) {
        assert(!input_points.empty());
        e_solver = new EssentialSolver (points);
        f_solver = new FundamentalSolver (points);
    }

    void setModelParameters (const cv::Mat& model) override {
        float *  E_ptr = (float *) model.data;
        e11 = E_ptr[0]; e12 = E_ptr[1]; e13 = E_ptr[2];
        e21 = E_ptr[3]; e22 = E_ptr[4]; e23 = E_ptr[5];
        e31 = E_ptr[6]; e32 = E_ptr[7]; e33 = E_ptr[8];
    }

    /*
     * E = K1^T F K2
     *
     * y'^T E y = 0, normalized points by third coordinate.
     * x' = (y'1 y'2 y'3) / y'3
     * x  = (y1  y2  y3)  / y3
     */
    unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        cv::Mat_<float> E;

        unsigned int models_count = e_solver->FivePoints (sample, E);

        for (unsigned int i = 0; i < models_count; i++) {
            models[i]->setDescriptor(E.rowRange(i * 3, i * 3 + 3));
        }

        return models_count;
    }

    bool EstimateModelNonMinimalSample(const int * const sample, unsigned int sample_size, Model &model) override {
        cv::Mat_<float> E;

        if (! f_solver->EightPointsAlgorithm(sample, sample_size, E)) {
            return false;
        }

        model.setDescriptor(E);

        return true;
    }

    float GetError(unsigned int pidx) override {
        unsigned int smpl = 4*pidx;
        float x1 = points[smpl];
        float y1 = points[smpl+1];
        float x2 = points[smpl+2];
        float y2 = points[smpl+3];

        // pt2^T * E, line 1
        float l1 = e11 * x2 + e21 * y2 + e31;
        float l2 = e12 * x2 + e22 * y2 + e32;
        float l3 = e13 * x2 + e23 * y2 + e33;

        // E * pt1, line 2
        float t1 = e11 * x1 + e12 * y1 + e13;
        float t2 = e21 * x1 + e22 * y1 + e23;
        float t3 = e31 * x1 + e32 * y1 + e33;

        // distance from pt1 to line 1
        float a1 = l1 * x1 + l2 * y1 + l3;
        float a2 = sqrt(l1 * l1 + l2 * l2);

        // distance from pt2 to line 2
        float b1 = t1 * x2 + t2 * y2 + t3;
        float b2 = sqrt(t1 * t1 + t2 * t2);

        // get distances
        // distance1 = abs (a1 / a2)
        // distance2 = abs (b1 / b2)

        // error is average of distances
        return (fabsf(a1 / a2) + fabsf(b1 / b2)) / 2;
    }


    static void getModelbyCameraMatrix (const cv::Mat &K1, const cv::Mat &K2, const cv::Mat &F, cv::Mat &E) {
        E =  K2.t() * F * K1;
    }

    int SampleNumber() override {
        return 5;
    }
};

#endif //USAC_ESSENTIALESTIMATOR_H
