// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_IRLS_H
#define USAC_IRLS_H

#include "../model.hpp"
#include "../quality/quality.hpp"
#include "../estimator/dlt/dlt.hpp"
#include "../random_generator/uniform_random_generator.hpp"

class Irls : public LocalOptimization {
private:
    Estimator * estimator;
    Quality * quality;
    Model * irls_model;
    UniformRandomGenerator * uniformRandomGenerator;
    Score * irls_score;

    float * weights;
    int *inliers, *sample;
    unsigned int points_size, max_sample_size;
    float threshold;

public:
    ~Irls() {
        delete[] weights; delete[] inliers; delete[] sample; delete[] uniformRandomGenerator;
        delete(irls_model); delete(irls_score);
    }

    Irls (Model * model, Estimator * estimator_, Quality * quality_, unsigned int points_size_) {
        max_sample_size = model->lo_sample_size;
    
        weights = new float[points_size_];
        inliers = new int[points_size_];
        sample = new int [max_sample_size];

        quality = quality_;
        estimator = estimator_;
        points_size = points_size_;
        threshold = model->threshold;
        irls_model = new Model (model);
        irls_score = new Score;

        uniformRandomGenerator = new UniformRandomGenerator;
        if (model->reset_random_generator) {
            uniformRandomGenerator->resetTime();
        }

    }

    void GetModelScore (Model * model, Score * score) override {
        irls_model->setDescriptor(model->returnDescriptor());

        for (unsigned int iter = 1; iter < 20; iter++) {
            estimator->setModelParameters(irls_model->returnDescriptor());

            unsigned int num_inliers = 0;

            if (model->estimator == ESTIMATOR::Homography) {
                float error;

                for (unsigned int i = 0; i < points_size; i++) {
                    error = estimator->GetError(i);
                    if (error < threshold) {
                        weights[i] = 1 / (1 + error);
//                        weights[i] = error;
                        // std::cout << weights[i] << " ";
                        inliers[num_inliers] = i;
                        num_inliers++;
                    }
                }
                //            std::cout << "\n";
            } else {
                estimator->GetError(weights, model->threshold, inliers, &num_inliers);
            }

            if (num_inliers <= model->sample_size)
                return;

            unsigned int num_samples = std::min(max_sample_size, num_inliers);
            uniformRandomGenerator->setSubsetSize(num_samples);
            uniformRandomGenerator->resetGenerator(0, num_inliers-1);
            uniformRandomGenerator->generateUniqueRandomSet(sample);

            for (unsigned int smpl = 0; smpl < num_samples; smpl++) {
                sample[smpl] = inliers[sample[smpl]];
            }
//            std::cout << "\n";

            std::cout << "irls iteration " << iter << "; inliers size " << num_inliers << "\n";
            if (num_inliers > score->inlier_number) {
                std::cout << "UPDATE SCORE\n";
                score->inlier_number = num_inliers;
                score->score = num_inliers;
                model->setDescriptor(irls_model->returnDescriptor());
            }

            // debug
            Model * r_model = new Model (model);
            Score * r_score = new Score;
            estimator->EstimateModelNonMinimalSample(sample, num_samples, *r_model);
            quality->getNumberInliers(r_score, r_model->returnDescriptor(), r_model->threshold);
            std::cout << "irls score without weights " << r_score->inlier_number << "\n";
            // -----c

            estimator->EstimateModelNonMinimalSample(sample, num_samples, weights, *irls_model);

        }

        quality->getNumberInliers(irls_score, irls_model->returnDescriptor());
        std::cout << "irls iteration 20; inliers size " << irls_score->inlier_number << "\n";

        if (irls_score->bigger(score)) {
            std::cout << "UPDATE SCORE\n";
            score->copyFrom(irls_score);
            model->setDescriptor(irls_model->returnDescriptor());
        }
        std::cout << "-----------------------------------------------\n";
    }
};

#endif //USAC_IRLS_H
