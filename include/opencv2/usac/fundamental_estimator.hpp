// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_FUNDAMENTALESTIMATOR_H
#define USAC_FUNDAMENTALESTIMATOR_H

#include <opencv2/core/mat.hpp>
#include <cassert>
#include <opencv2/core.hpp>
#include "estimator.hpp"
#include "fundamental_solver.hpp"

namespace cv { namespace usac {
class FundamentalEstimator : public Estimator {
private:
    const float *const points;
    float f11, f12, f13, f21, f22, f23, f31, f32, f33;
    FundamentalSolver solver;
public:
    
    /*
     * @input_points: is matrix of size: number of points x 4
     * x1 y1 x'1 y'1
     * ...
     * xN yN x'N y'N
     *
     * X^T F X = 0
     */
    FundamentalEstimator(cv::InputArray input_points) : points((float *) input_points.getMat().data), solver (points) {
        assert(!input_points.empty());
        assert(input_points.getMat().cols == 4 && input_points.getMat().rows >= 7);
    }

    void setModelParameters(const cv::Mat &model) override {

        /*
         * To make pointer from Mat class, this Mat class should exists as long as exists pointer
         * So this->F and this->F_inv must be global in class
         */
        auto *F_ptr = (float *) model.data;
        f11 = F_ptr[0]; f12 = F_ptr[1]; f13 = F_ptr[2];
        f21 = F_ptr[3]; f22 = F_ptr[4]; f23 = F_ptr[5];
        f31 = F_ptr[6]; f32 = F_ptr[7]; f33 = F_ptr[8];
    }

    unsigned int estimateModel(const int *const sample, std::vector<Model *> &models) override {
        cv::Mat_<float> F;

        unsigned int roots = solver.SevenPointsAlgorithm(sample, F);

        unsigned int valid_solutions = 0;
        for (unsigned int i = 0; i < roots; i++) {
            if (isModelValid(F.rowRange(i * 3, i * 3 + 3), sample)) {
                models[valid_solutions++]->setDescriptor(F.rowRange(i * 3, i * 3 + 3));
            }
        }

        return valid_solutions;
    }

    bool
    estimateModelNonMinimalSample(const int *const sample, unsigned int sample_size, Model &model) override {
        cv::Mat_<float> F;

        if (!solver.EightPointsAlgorithm(sample, sample_size, F)) {
            return false;
        }

        model.setDescriptor(F);

        return true;
    }

    /*
     * Sampson error
     *                               (pt2^t * F * pt1)^2)
     * Error =  -------------------------------------------------------------------
     *          (((F⋅pt1)(0))^2 + ((F⋅pt1)(1))^2 + ((F^t⋅pt2)(0))^2 + ((F^t⋅pt2)(1))^2)
     *
     * [ x2 y2 1 ] * [ F(1,1)  F(1,2)  F(1,3) ]   [ x1 ]
     *               [ F(2,1)  F(2,2)  F(2,3) ] * [ y1 ]
     *               [ F(3,1)  F(3,2)  F(3,3) ]   [ 1  ]
     *
     */
    float getError(unsigned int pidx) override {
        unsigned int smpl = 4 * pidx;
        float x1 = points[smpl];
        float y1 = points[smpl + 1];
        float x2 = points[smpl + 2];
        float y2 = points[smpl + 3];

        float F_pt1_x = f11 * x1 + f12 * y1 + f13;
        float F_pt1_y = f21 * x1 + f22 * y1 + f23;

        // Here F is transposed
        float F_pt2_x = f11 * x2 + f21 * y2 + f31;
        float F_pt2_y = f12 * x2 + f22 * y2 + f32;

        float pt2_F_pt1 = x2 * F_pt1_x + y2 * F_pt1_y + f31 * x1 + f32 * y1 + f33; // f33 = 1

        float error = (pt2_F_pt1 * pt2_F_pt1) /
                      (F_pt1_x * F_pt1_x + F_pt1_y * F_pt1_y + F_pt2_x * F_pt2_x + F_pt2_y * F_pt2_y);

        // error >= 0
        return error;
    }

    static void getModelbyCameraMatrix(const cv::Mat &K1, const cv::Mat &K2, const cv::Mat &E, cv::Mat &F) {
        F = K2.inv().t() * E * K1.inv();
    }

    int sampleNumber() override {
        return 7;
    }


    static void getFundamentalFromProjection(const cv::Mat &P1, const cv::Mat &P2, cv::Mat &F) {
        cv::SVD svd(P1, 4);

        // e1 = svd.vt.row(3)
        cv::Mat e2 = P2 * svd.vt.row(3).t();

        cv::Mat e2x = (cv::Mat_<float>(3, 3) << 0, -e2.at<float>(2), e2.at<float>(1),
                e2.at<float>(2), 0, -e2.at<float>(0),
                -e2.at<float>(1), e2.at<float>(0), 0);

        F = e2x * P2 * P1.inv(cv::DECOMP_SVD);
        F = F / F.at<float>(2, 2);
    }


    bool isModelValid(const cv::Mat &F, const int *const sample) override {
        cv::Mat ec;
        float sig, sig1;
        int i;
        epipole(ec, F);

        sig1 = getorisig(F, &ec, 4 * sample[0]);

        for (i = 1; i < 7; i++) {
            sig = getorisig(F, &ec, 4 * sample[i]);

            if (sig1 * sig < 0) return false;
        }
        return true;
    }

private:
    /************** oriented constraints ******************/
    void epipole(cv::Mat &ec, const cv::Mat &F) const {
        ec = F.row(0).cross(F.row(2));

        for (int i = 0; i < 3; i++)
            if ((ec.at<float>(i) > 1.9984e-15) || (ec.at<float>(i) < -1.9984e-15)) return;
        ec = F.row(1).cross(F.row(2));
    }

    // u = x1 y1 1 x2 y2 1
    float getorisig(const cv::Mat &F, const cv::Mat *ec, unsigned int pt_idx) const {
        float s1, s2;

        float y1 = points[pt_idx + 1];
        float x2 = points[pt_idx + 2];
        float y2 = points[pt_idx + 3];

        s1 = F.at<float>(0) * x2 + F.at<float>(3) * y2 + F.at<float>(6);
        s2 = ec->at<float>(1) - ec->at<float>(2) * y1;

        return (s1 * s2);
    }
};
}}
#endif //USAC_FUNDAMENTALESTIMATOR_H