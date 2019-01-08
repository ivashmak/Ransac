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

    int num_sample;
    int sample_size;
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
    void init (unsigned int points_size_, Model * model, Estimator * estimator_, Quality * quality_, NeighborsSearch neighborsType_) {
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

        num_sample = 7 * model->sample_size;

        sample_size = model->sample_size;

        errors = new float[points_size];

        inliers = new int [points_size];
        sample = new int [num_sample];
        uniform_random_generator = new UniformRandomGenerator;

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

	void labeling (const cv::Mat& model, Score * score, int * inliers);

    void GraphCutLO (Model * best_model, Score * best_score) {
//        std::cout << "begin best score " << best_score->inlier_number << "\n";

        gc_model->setDescriptor(best_model->returnDescriptor());

        OneStepLO (gc_model);

        bool is_best_model_updated;
        while (true) {
            // build graph problem
            // apply graph cut to G
            labeling(best_model->returnDescriptor(), gc_score, inliers);
//            std::cout << "virtual inliers " << gc_score->inlier_number << "\n";

//             if number of "virtual" inliers is too small then break
            if (gc_score->inlier_number <= sample_size) break;
            is_best_model_updated = false;

            // sample to generate min (|I_7m|, |I|)
            unsigned int gc_sample_size = std::min (num_sample, gc_score->inlier_number);

            // reset random generator
            uniform_random_generator->setSubsetSize(gc_sample_size);
            // generate random subset in range <0; |I|>
            uniform_random_generator->resetGenerator(0, gc_score->inlier_number-1);

            unsigned int inner_inliers_size = gc_score->inlier_number;
//            std::cout << gc_sample_size << " vs " << inner_inliers_size << "\n";
            for (unsigned int iter = 0; iter < lo_inner_iterations; iter++) {
                if (gc_sample_size < inner_inliers_size) {

                    uniform_random_generator->generateUniqueRandomSet(sample);
                    // set sample as inliers from labeling
                    for (int smpl = 0; smpl < gc_sample_size; smpl++) {
                        sample[smpl] = inliers[sample[smpl]];
//                         std::cout << sample[smpl] << " ";
                    }
//                     std::cout << "\n";
                    if (! estimator->EstimateModelNonMinimalSample(sample, gc_sample_size, *gc_model)) {
                        continue;
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
                    if (! estimator->EstimateModelNonMinimalSample(inliers, inner_inliers_size, *gc_model)) {
                        // break loop of inner iterations if estimation failed.
                        break;
                    }
                }

                quality->getNumberInliers(gc_score, gc_model->returnDescriptor());

//                std::cout << "GC score " << gc_score->inlier_number << "\n";

                if (gc_score->bigger(best_score)) {
//                    std::cout << "Update best score\n";
//                     std::cout << "UPDATE best score " << gc_score->inlier_number << "\n";
                    is_best_model_updated = true;
                    best_score->copyFrom(gc_score);
                    best_model->setDescriptor(gc_model->returnDescriptor());
                }

                // only for test
                gc_iterations++;
                //
            }

            // break if best score was not updated
            if (! is_best_model_updated) {
                break;
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

//        std::cout << "inliers before one step LO " << gc_score->inlier_number << "\n";

        unsigned int sample_generate = std::min ((unsigned int)gc_score->inlier_number, model->lo_sample_size);

        if (sample_generate < model->sample_size)
            return;


        if (model->sampler == SAMPLER::Prosac) {
            // if we use prosac sample, so points are ordered by some score,
            // so take first N inliers, because they have higher score
            for (unsigned int smpl = 0; smpl < sample_generate; smpl++) {
                sample[smpl] = inliers[smpl];
            }
        } else {
            uniform_random_generator->setSubsetSize(sample_generate);
            uniform_random_generator->resetGenerator(0, gc_score->inlier_number -1);
            uniform_random_generator->generateUniqueRandomSet(sample);
            for (unsigned int smpl = 0; smpl < sample_generate; smpl++) {
                sample[smpl] = inliers[sample[smpl]];
            }
        }

        estimator->EstimateModelNonMinimalSample(sample, sample_generate, *model);

        // debug
//        quality->getNumberInliers(gc_score, model);
//        std::cout << "inliers after one step LO " << gc_score->inlier_number << "\n";
    }
};

#endif //USAC_GRAPHCUT_H
