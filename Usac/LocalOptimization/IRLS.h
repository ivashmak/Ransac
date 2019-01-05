#ifndef USAC_IRLS_H
#define USAC_IRLS_H

#include "../Model.h"
#include "../Quality/Quality.h"
#include "../Estimator/DLT/DLT.h"

class IRLS {
private:
    float * weights;
    int *inliers, *sample;

    Estimator * estimator;
    Quality * quality;
    unsigned int points_size;
    float threshold;
    Model * irls_model;
    UniformRandomGenerator * uniformRandomGenerator;
    Score * irls_score;
    unsigned int max_sample_size;
public:
    ~IRLS() {
        delete[] weights;
        delete[] inliers;
        delete[] sample;
        delete[] uniformRandomGenerator;
    }

    IRLS (unsigned int points_size_, Model * model, Estimator * estimator_, Quality * quality_) {
        weights = new float[points_size_];
        inliers = new int[points_size_];
        sample = new int [points_size_];

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

        max_sample_size = model->lo_sample_size;
    }

    void getModelScore (Score * score, Model * model) {
        irls_model->setDescriptor(model->returnDescriptor());

        for (unsigned int iter = 1; iter < 20; iter++) {
            estimator->setModelParameters(irls_model->returnDescriptor());

            unsigned int num_inliers = 0;
            float error;
            for (unsigned int i = 0; i < points_size; i++) {
                error = estimator->GetError(i);
                if (error < threshold) {
//                    weights[i] = 1 / (1 + error);
                    weights[i] = 1 / error;
                    inliers[num_inliers] = i;
                    num_inliers++;
                }
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
            estimator->EstimateModelNonMinimalSample(sample, num_samples, *irls_model);
            quality->getNumberInliers(irls_score, irls_model);
            std::cout << "irls score without weights " << irls_score->inlier_number << "\n";
            // -----

            estimator->EstimateModelNonMinimalSample(sample, num_samples, weights, *irls_model);

        }

        quality->getNumberInliers(irls_score, irls_model);
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
