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
    float lambda;
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
public:
    int gc_iterations;

    ~GraphCut() {
        delete gc_score, gc_model, inliers, sample, uniform_random_generator;
    }
    void init (unsigned int points_size_, Model * model, Estimator * estimator_, Quality * quality_, const int * const neighbors_) {
        lambda = model->lambda_graph_cut;
        knn = model->k_nearest_neighbors;
        threshold = model->threshold;
        estimator = estimator_;
        quality = quality_;
        sqr_thr = 2 * threshold * threshold;
        points_size = points_size_;
        neighbors = const_cast<int *>(neighbors_);

        gc_score = new Score;

        gc_model = new Model (model);

        num_sample = 7 * model->sample_number;
        sample_size = model->sample_number;

        inliers = new int [points_size];
        sample = new int [num_sample];
        uniform_random_generator = new UniformRandomGenerator;
        // comment to debug
        uniform_random_generator->resetTime();

        lo_inner_iterations = model->lo_inner_iterations;

        gc_iterations = 0;
    }

	void labeling (const cv::Mat& model, Score * score, int * inliers = nullptr);

    void GraphCutLO (Model * best_model, Score * best_score) {
//        std::cout << "begin best score " << best_score->inlier_number << "\n";

        gc_model->setDescriptor(best_model->returnDescriptor());
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
            for (int iter = 0; iter < lo_inner_iterations; iter++) {
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
                    if (inner_inliers_size >= gc_sample_size && iter > 0) {
                        break;
                    }
                    if (! estimator->EstimateModelNonMinimalSample(inliers, inner_inliers_size, *gc_model)) {
                        // break loop of inner iterations if inliers are bad.
                        break;
                    }
                }

                quality->getNumberInliers(gc_score, gc_model, false, nullptr);

//                std::cout << "gc score " << gc_score->inlier_number << "\n";

                if (gc_score->bigger(best_score)) {
//                    std::cout << "UPDATE best score " << gc_score->inlier_number << "\n";
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

};

#endif //USAC_GRAPHCUT_H
