// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_RANSACLOCALOPTIMIZATION_H
#define USAC_RANSACLOCALOPTIMIZATION_H

#include "local_optimization.hpp"
#include "../quality/quality.hpp"
#include "../sampler/uniform_sampler.hpp"
#include "../random_generator/uniform_random_generator.hpp"
#include "iterative_local_optimization.hpp"

/*
 * Reference:
 * http://cmp.felk.cvut.cz/~matas/papers/chum-dagm03.pdf
 */

class InnerLocalOptimization : public LocalOptimization {
private:
    Quality * quality;
    Estimator * estimator;
    Score * lo_score;
    Model * lo_model;
    UniformRandomGenerator * uniform_random_generator;
    IterativeLocalOptimization * iterativeLocalOptimization;

    int *lo_inliers, *max_inliers, *lo_sample;
    unsigned int lo_inner_max_iterations, sample_limit;
    bool limited;
public:
    unsigned int lo_inner_iters = 0, lo_iterative_iters = 0;

    ~InnerLocalOptimization () override {
        // std::cout << "cleaning in ransac lo\n";
        delete[] lo_inliers; delete[] lo_sample; delete[] max_inliers;
        delete (lo_score); delete (lo_model); delete (uniform_random_generator); delete (iterativeLocalOptimization);
    }

    InnerLocalOptimization (Model *model, Estimator *estimator_, Quality *quality_, unsigned int points_size) {

        estimator = estimator_;
        quality = quality_;

        lo_inner_max_iterations = model->lo_inner_iterations;
        sample_limit = model->lo_sample_size;

        // Allocate max memory to avoid reallocation
        lo_inliers = new int [points_size];
        max_inliers = new int [points_size];
        lo_sample = new int [sample_limit];

        lo_score = new Score;
        lo_model = new Model (model);

        // ------------- set random generator --------------
        uniform_random_generator = new UniformRandomGenerator;
        uniform_random_generator->setSubsetSize(sample_limit);
        if (model->reset_random_generator) uniform_random_generator->resetTime();
        // -----------------------------------

        limited = model->lo == LocOpt ::InItFLORsc;
        // ----------------------
        iterativeLocalOptimization = new IterativeLocalOptimization
                (points_size, model, uniform_random_generator, estimator, quality);

        lo_inner_iters = 0;
        lo_iterative_iters = 0;
    }

    /*
     * Implementation of Locally Optimized Ransac
     * Inner + Iterative
     */
    void GetModelScore (Model * best_model,
                           Score * best_score) override {

        // get inliers from so far the best model of Ransac.
        quality->getInliers(best_model->returnDescriptor(), max_inliers);

        /*
         * Inner Local Optimization Ransac
         */
        for (unsigned int iters = 0; iters < lo_inner_max_iterations; iters++) {
            /*
             * Generate sample of lo_sample_size from inliers from the best model.
             */
            if (best_score->inlier_number > sample_limit) {
                // if there are many inliers take limited number at random
                // sample from best model.
                uniform_random_generator->generateUniqueRandomSet(lo_sample, best_score->inlier_number-1);
                // get inliers from maximum inliers from lo
                for (int smpl = 0; smpl < sample_limit; smpl++) {
                    lo_sample[smpl] = max_inliers[lo_sample[smpl]];
                }
            
                if (!estimator->LeastSquaresFitting(lo_sample, sample_limit, *lo_model)) continue;
            } else {
                // if inliers are less than limited number of sample then take all of them for estimation
                // if it fails -> end Lo.
                if (!estimator->LeastSquaresFitting(max_inliers, best_score->inlier_number, *lo_model)) return;
            }

            // Start evaluating a model with new threshold. And get inliers for iterative lo ransac.
            // multiply threshold K * θ
            lo_model->threshold = lo_model->lo_threshold_multiplier * lo_model->threshold;
            quality->getNumberInliers(lo_score, lo_model->returnDescriptor(), lo_model->threshold, true, lo_inliers);

            // continue if there are not enough inliers for non minimal estimation
            if (lo_score->inlier_number <= lo_model->sample_size) continue;

            bool fail;
            if (limited) {
                // fixing locally optimized ransac with limited iterative lo.
                fail = iterativeLocalOptimization->GetScoreLimited(lo_score, lo_model, lo_inliers);
            } else {
                // unlimited iterative lo
                fail = iterativeLocalOptimization->GetScoreUnlimited(lo_score, lo_model, best_score, lo_inliers);
            }

            // only for test
            lo_iterative_iters = iterativeLocalOptimization->lo_iterative_iters;
            //

            // update best model
            if (!fail && lo_score->bigger(best_score)) {
//                std::cout << "Update best score\n";
                best_model->setDescriptor(lo_model->returnDescriptor());
                best_score->copyFrom(lo_score);
                //copy inliers to max inliers for sampling from best model.
                std::copy(lo_inliers, lo_inliers + lo_score->inlier_number, max_inliers);
            }

            // only for test
            lo_inner_iters++;
            //
        }
    }
};

#endif //USAC_RANSACLOCALOPTIMIZATION_H
