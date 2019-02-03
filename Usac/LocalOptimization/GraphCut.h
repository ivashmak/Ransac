// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_GRAPHCUT_H
#define USAC_GRAPHCUT_H

#include <opencv2/core/mat.hpp>
#include "../Estimator/Estimator.h"
#include "../../include/gco-v3.0/GCoptimization.h"
#include "LocalOptimization.h"
#include "../Sampler/UniformSampler.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"


class GraphCut : public LocalOptimization {
protected:
    float threshold;
    unsigned int points_size;
    Estimator * estimator;
    Quality * quality;
    int knn;
    float spatial_coherence;
    float sqr_thr;
    int * neighbors;
    Score * gc_score;
    Model * gc_model;

    int * inliers;
    int * sample;
    UniformRandomGenerator * uniform_random_generator;

    unsigned int sample_limit;
    unsigned int sample_size;
    unsigned int lo_inner_iterations;
    float * errors;

    std::vector<std::vector<int>> neighbors_v;
    NeighborsSearch neighborsType = NeighborsSearch::NullN;

    int neighbor_number;
    bool isInit = false;
public:
    int gc_iterations;

    ~GraphCut() override {
        delete[] errors; delete[] inliers; delete[] sample;
        delete (gc_score); delete (gc_model); delete (uniform_random_generator);
    }
    
    GraphCut (unsigned int points_size_, Model * model, Estimator * estimator_, Quality * quality_, NeighborsSearch neighborsType_) {
        neighborsType = neighborsType_;
        spatial_coherence = model->spatial_coherence_gc;
        knn = model->k_nearest_neighbors;
        threshold = model->threshold;
        estimator = estimator_;
        quality = quality_;
        sqr_thr = 2 * threshold * threshold;
        points_size = points_size_;

        gc_score = new Score;
        gc_model = new Model (model);

        sample_limit = 7 * model->sample_size;

        sample_size = model->sample_size;

        errors = new float[points_size];

        inliers = new int [points_size];
        sample = new int [sample_limit];

        uniform_random_generator = new UniformRandomGenerator;
        uniform_random_generator->setSubsetSize(sample_limit);
        if (model->reset_random_generator) {
            uniform_random_generator->resetTime();
        }

        lo_inner_iterations = model->lo_inner_iterations;

        gc_iterations = 0;

        if (neighborsType == NeighborsSearch::Nanoflann) {
            neighbor_number = knn * points_size;
        }
        isInit = true;
    }

    void setNeighbors (const std::vector<std::vector<int>>& neighbors_v_) {
        neighbors_v = neighbors_v_;
        assert(isInit); // check if GC has already initialized
        assert(neighborsType == NeighborsSearch::Grid); // check if we use grid search

        neighbor_number = 0;
        for (int i = 0; i < points_size; i++) {
            neighbor_number += neighbors_v[i].size();
        }
    }

    void setNeighbors (const int * const neighbors_) {
        neighbors = const_cast<int *>(neighbors_);
    }

    // calculate lambda
    void calculateSpatialCoherence (float inlier_number) {
        spatial_coherence = (points_size * (inlier_number / points_size)) / static_cast<float>(neighbor_number);
    }

	void labeling (const cv::Mat& model, Score * score, int * inliers);

    void GetModelScore (Model * best_model, Score * best_score) override {
//        std::cout << "begin best score " << best_score->inlier_number << "\n";
        // improve best model by non minimal estimation

//        OneStepLO (best_model);
        // update score after one step lo
//        quality->getNumberInliers(best_score, best_model->returnDescriptor());

//        std::cout << "best score after one step lo " << best_score->inlier_number << "\n";

        bool is_best_model_updated = true;
        while (is_best_model_updated) {
            is_best_model_updated = false;
//            std::cout << "while loop\n";

            // build graph problem
            // apply graph cut to G
//            calculateSpatialCoherence(best_score->inlier_number);
            labeling(best_model->returnDescriptor(), gc_score, inliers);
//            std::cout << "virtual inliers " << gc_score->inlier_number << "\n";

//             if number of "virtual" inliers is too small then break
            if (gc_score->inlier_number <= sample_size) break;
            unsigned int labeling_inliers_size = gc_score->inlier_number;

//            std::cout << gc_sample_size << " vs " << inner_inliers_size << "\n";
            for (unsigned int iter = 0; iter < lo_inner_iterations; iter++) {
                // sample to generate min (|I_7m|, |I|)
                if (labeling_inliers_size > sample_limit) {
                    // generate random subset in range <0; |I|>
                    uniform_random_generator->generateUniqueRandomSet(sample, labeling_inliers_size-1);
                    // sample from inliers of labeling
                    for (unsigned int smpl = 0; smpl < sample_limit; smpl++) {
                        sample[smpl] = inliers[sample[smpl]];
                    }
                    if (! estimator->EstimateModelNonMinimalSample(sample, sample_limit, *gc_model)) {
                        break;
                    }
                } else {
                    if (iter > 0) {
                        /*
                         * If iterations are more than 0 and there are not enough inliers for random sampling,
                         * so break. Because EstimateModelNonMinimalSample for same inliers gives same model, it
                         * is redundant to use it more than 1 time.
                         */
                        break;
                    }
                    if (! estimator->EstimateModelNonMinimalSample(inliers, labeling_inliers_size, *gc_model)) {
                        break;
                    }
                }

                quality->getNumberInliers(gc_score, gc_model->returnDescriptor());

//                std::cout << "GC score " << gc_score->inlier_number << "\n";

                if (gc_score->bigger(best_score)) {
//                    std::cout << "Update best score\n";
//                    std::cout << "UPDATE best score " << gc_score->inlier_number << "\n";
                    is_best_model_updated = true;
                    best_score->copyFrom(gc_score);
                    best_model->setDescriptor(gc_model->returnDescriptor());
                }

                // only for test
                gc_iterations++;
                //
            }
        }

//        std::cout << "end best score " << best_score->inlier_number << "\n";
    }

private:
    void OneStepLO (Model * model) {
        /*
         * Do one step local optimization on min (|I|, |max_I|) before doing Graph Cut.
         * This LSQ must give better model for next GC labeling.
         */
        // use gc_score variable, but we are not getting gc score.
        quality->getNumberInliers(gc_score, model->returnDescriptor(), model->threshold, true, inliers);

        // return if not enough inliers
        if (gc_score->inlier_number <= model->sample_size)
            return;

        unsigned int one_step_lo_sample_limit = model->lo_sample_size;
        if (sample_limit < one_step_lo_sample_limit) {
            one_step_lo_sample_limit = sample_limit;
        }

        if (gc_score->inlier_number < one_step_lo_sample_limit) {
            // if score is less than limit number sample then take estimation of all inliers
            estimator->EstimateModelNonMinimalSample(inliers, gc_score->inlier_number, *model);
        } else {
            // otherwise take some inliers as sample at random
            if (model->sampler == SAMPLER::Prosac) {
                // if we use prosac sample, so points are ordered by some score,
                // so take first N inliers, because they have higher score
                for (unsigned int smpl = 0; smpl < one_step_lo_sample_limit; smpl++) {
                    sample[smpl] = inliers[smpl];
                }
            } else {
                uniform_random_generator->generateUniqueRandomSet(sample, one_step_lo_sample_limit, gc_score->inlier_number-1);
                for (unsigned int smpl = 0; smpl < one_step_lo_sample_limit; smpl++) {
                    sample[smpl] = inliers[sample[smpl]];
                }
            }

            estimator->EstimateModelNonMinimalSample(sample, one_step_lo_sample_limit, *model);
        }
    }
};

#endif //USAC_GRAPHCUT_H
