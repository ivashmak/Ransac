#ifndef USAC_SORTED_LO_H
#define USAC_SORTED_LO_H

#include "../Model.h"
#include "../Quality/Quality.h"
#include "../Estimator/DLT/DLT.h"

class SortedLO {
private:
    float * errors;
    int *inliers, *sample;

    Estimator * estimator;
    Quality * quality;
    unsigned int points_size;
    float threshold;
    Model * slo_model;
    UniformRandomGenerator * uniformRandomGenerator;
    Score * slo_score;
    unsigned int max_sample_size;
public:
    ~SortedLO() {
        delete[] errors;
        delete[] inliers;
        delete[] sample;
        delete[] uniformRandomGenerator;
    }

    SortedLO (unsigned int points_size_, Model * model, Estimator * estimator_, Quality * quality_) {
        errors = new float[points_size_];
        inliers = new int[points_size_];
        sample = new int [points_size_];

        quality = quality_;
        estimator = estimator_;
        points_size = points_size_;
        threshold = model->threshold;
        slo_model = new Model (model);
        slo_score = new Score;

        uniformRandomGenerator = new UniformRandomGenerator;
        if (model->reset_random_generator) {
            uniformRandomGenerator->resetTime();
        }

        max_sample_size = model->lo_sample_size;
    }

    void getModelScore (Score * score, Model * model) {
        slo_model->setDescriptor(model->returnDescriptor());

        for (unsigned int iter = 1; iter < 10; iter++) {
            estimator->setModelParameters(slo_model->returnDescriptor());

            unsigned int num_inliers = 0;
            float error;
            for (unsigned int i = 0; i < points_size; i++) {
                error = estimator->GetError(i);
                if (error < threshold) {
                    errors[i] = error;
                    inliers[num_inliers] = i;
                    num_inliers++;
                }
            }

            if (num_inliers <= model->sample_size) return;

            std::cout << "SortedLO iteration " << iter << "; inliers size " << num_inliers << "\n";
            if (num_inliers > score->inlier_number) {
                std::cout << "UPDATE SCORE\n";
                score->inlier_number = num_inliers;
                score->score = num_inliers;
                model->setDescriptor(slo_model->returnDescriptor());
            }

            unsigned int num_samples = std::min(max_sample_size, num_inliers);
            
            // unsigned int idxes = new unsigned int[num_inliers];
            // for (int i = 0; i < num_inliers; i++) {
            //     idxes[i] = i;
            // }

            std::sort(inliers, inliers+num_inliers, [&](int a, int b) {
                return errors[a] > errors[b];
            });

            for (int i = 0; i < num_samples; i++) {
                sample[i] = inliers[i];
                // std::cout << errors[sample[i]] << "\n";
            }
            

            estimator->EstimateModelNonMinimalSample(sample, num_samples, *slo_model);
            
            // debug
            uniformRandomGenerator->setSubsetSize(num_samples);
            uniformRandomGenerator->resetGenerator(0, num_inliers-1);
            uniformRandomGenerator->generateUniqueRandomSet(sample);
            for (unsigned int smpl = 0; smpl < num_samples; smpl++) {
                sample[smpl] = inliers[sample[smpl]];
            }
            Model * temp_model = new Model(slo_model);
            estimator->EstimateModelNonMinimalSample(sample, num_samples, *temp_model);
            quality->getNumberInliers(slo_score, temp_model->returnDescriptor());
            std::cout << "sorted lo random subset " << slo_score->inlier_number << "\n";
            // -----

            // exit (0);
        }
        std::cout << "------------------------------------\n";
    }
};

#endif //USAC_SORTED_LO_H
