// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_AFFINEESTIMATOR_H
#define RANSAC_AFFINEESTIMATOR_H

#include "estimator.hpp"

class AffineEstimator : public Estimator{
private:
    const float * const points;
    float a, b, c, d, e, f;
public:
    ~AffineEstimator () {
    }

    /*
     * @input_points: is matrix of size: number of points x 4
     * x1 y1 x'1 y'1
     * ...
     * xN yN x'N y'N
     */
    AffineEstimator(cv::InputArray input_points) : points((float *)input_points.getMat().data) {
        assert(!input_points.empty());
    }

    void setModelParameters (const cv::Mat& model) override {
        auto * A_ptr = (float *) model.data;
        // std::cout << "set\n";
        a = A_ptr[0]; b = A_ptr[1]; c = A_ptr[2];
        d = A_ptr[3]; e = A_ptr[4]; f = A_ptr[5];
        // std::cout << "set ok\n";
    }

    /*
        Affine transformation
        x1 y1 1 0  0  0   a   x1'
        0  0  0 x1 y1 1   b   y1'
        x2 y2 1 0  0  0   c   x2'
        0  0  0 x2 y2 1 * d = y2'
        x3 y3 1 0  0  0   e   x3'
        0  0  0 x3 y3 1   f   y3'
    */
        
    unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        cv::Mat_<float> A (6, 1);
        float * A_ptr = (float *) A.data;
        unsigned int smpl1 = 4*sample[0], smpl2 = 4*sample[1], smpl3 = 4*sample[2];
        float x1 = points[smpl1  ], y1 = points[smpl1+1],
              u1 = points[smpl1+2], v1 = points[smpl1+3],
              x2 = points[smpl2  ], y2 = points[smpl2+1],
              u2 = points[smpl2+2], v2 = points[smpl2+3],
              x3 = points[smpl3  ], y3 = points[smpl3+1],
              u3 = points[smpl3+2], v3 = points[smpl3+3];
              
        A_ptr[0] = (u1*y2 - u2*y1 - u1*y3 + u3*y1 + u2*y3 - u3*y2)/(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
        A_ptr[1] = -(u1*x2 - u2*x1 - u1*x3 + u3*x1 + u2*x3 - u3*x2)/(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
        A_ptr[2] = (u1*x2*y3 - u1*x3*y2 - u2*x1*y3 + u2*x3*y1 + u3*x1*y2 - u3*x2*y1)/(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
        A_ptr[3] = (v1*y2 - v2*y1 - v1*y3 + v3*y1 + v2*y3 - v3*y2)/(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
        A_ptr[4] = -(v1*x2 - v2*x1 - v1*x3 + v3*x1 + v2*x3 - v3*x2)/(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
        A_ptr[5] = (v1*x2*y3 - v1*x3*y2 - v2*x1*y3 + v2*x3*y1 + v3*x1*y2 - v3*x2*y1)/(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
        models[0]->setDescriptor(A);
        return 1;
    }

    // TODO: Change LSQ to PCA.
    bool EstimateModelNonMinimalSample (const int * const sample, unsigned int sample_size, Model &model) override {
        cv::Mat_<float> A(2*sample_size, 6, float(0));
        cv::Mat_<float> b(2*sample_size, 1);
        float * A_ptr = (float *) A.data;
        float * b_ptr = (float *) b.data;
        unsigned int smpl;
        float x1, y1, x2, y2;

        for (unsigned int p = 0; p < sample_size; p++) {
            smpl = 4*sample[p];
            x1 = points[smpl];
            y1 = points[smpl+1];
            x2 = points[smpl+2];
            y2 = points[smpl+3];
            
            (*A_ptr++) = x1;
            (*A_ptr++) = y1;
            (*A_ptr++) = 1;
            (*A_ptr++);
            (*A_ptr++);
            (*A_ptr++);
            
            (*A_ptr++);
            (*A_ptr++);
            (*A_ptr++);
            (*A_ptr++) = x1;
            (*A_ptr++) = y1;
            (*A_ptr++) = 1;

            (*b_ptr++) = x2;
            (*b_ptr++) = y2;   
        }

        cv::Mat Aff;
        // cv::Mat Aff = inv (A.t() * A) * A.t() * b;
        bool solved = cv::solve (A, b, Aff, cv::DECOMP_QR);
        if (! solved) return false;
        model.setDescriptor(Aff);
        return true;
    }

    bool EstimateModelNonMinimalSample (const int * const sample, unsigned int sample_size, const float * const weights, Model &model) override {
        return true;
    }

    bool LeastSquaresFitting (const int * const sample, unsigned int sample_size, Model &model) override {
        return EstimateModelNonMinimalSample(sample, sample_size, model);
    }
    /*
     * Error = distance (T pt(i), pt'(i))
     */
    float GetError(unsigned int pidx) override {
        unsigned int smpl = 4*pidx;
        float x1 = points[smpl];
        float y1 = points[smpl+1];
        float x2 = points[smpl+2];
        float y2 = points[smpl+3];

        float est_x2 = a * x1 + b * y1 + c;
        float est_y2 = d * x1 + e * y1 + f;

        return sqrt ((x2 - est_x2) * (x2 - est_x2) + (y2 - est_y2) * (y2 - est_y2));
    }

    int SampleNumber() override {
        return 3;
    }
};


#endif //RANSAC_AFFINEESTIMATOR_H