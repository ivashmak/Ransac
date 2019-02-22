// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_ITERATIVELOCALOPTIMIZATION_H
#define USAC_ITERATIVELOCALOPTIMIZATION_H

#include "quality.hpp"
#include "uniform_random_generator.hpp"

namespace cv { namespace usac {
class IterativeLocalOptimization : public LocalOptimization {
private:
    unsigned int max_iters, threshold_multiplier, sample_limit, points_size;
    bool is_sample_limit;
    float threshold_step, threshold;
    UniformRandomGenerator *uniformRandomGenerator;
    Estimator *estimator;
    Quality *quality;

    int *lo_sample;
    unsigned int lo_iterative_iters;
public:

    ~IterativeLocalOptimization() {
        if (is_sample_limit) {
            delete[] lo_sample;
        }
    }

    IterativeLocalOptimization(unsigned int points_size_, Model *model,
                               UniformRandomGenerator *uniformRandomGenerator_,
                               Estimator *estimator_, Quality *quality_) {
        points_size = points_size_;
        max_iters = model->lo_iterative_iterations;
        threshold_multiplier = model->lo_threshold_multiplier;
        is_sample_limit = model->lo == LocOpt::InItFLORsc;
        sample_limit = model->lo_sample_size;
        if (is_sample_limit) {
            lo_sample = new int[sample_limit];
        }

        threshold = model->threshold;
        /*
         * reduce multiplier threshold K·θ by this number in each iteration.
         * In the last iteration there be original threshold θ.
         */
        threshold_step = (threshold * threshold_multiplier - threshold) / max_iters;

        uniformRandomGenerator = uniformRandomGenerator_;
        estimator = estimator_;
        quality = quality_;

        lo_iterative_iters = 0;
    }

    void getModelScore (Model * model, Score * score) override {
        int * inliers = new int[points_size];
        quality->getInliers(model->returnDescriptor(), inliers);
        getScoreLimited(score, model, inliers);
        delete[] inliers;
    }

    /*
     * Iterative LO Ransac
     * Reduce threshold of current model
     * Estimate model parametres from limited sample size of lo model.
     * Evaluate model
     * Get inliers
     * Repeat until iteration < lo iterative iterations
     */

    bool getScoreLimited(Score *lo_score, Model *lo_model, int *lo_inliers) {
        for (unsigned int iterations = 0; iterations < max_iters; iterations++) {
            lo_model->threshold -= threshold_step;

            // break if there are not enough inliers to estimate non minimal model
            if (lo_score->inlier_number <= lo_model->sample_size) break;
            if (lo_score->inlier_number > sample_limit) {
                // if there are more inliers than limit for sample size then generate at random
                // sample from LO model.
                uniformRandomGenerator->generateUniqueRandomSet(lo_sample, lo_score->inlier_number - 1);
                for (unsigned int smpl = 0; smpl < sample_limit; smpl++) {
                    lo_sample[smpl] = lo_inliers[lo_sample[smpl]];
                }
                if (!estimator->leastSquaresFitting(lo_sample, sample_limit, *lo_model)) continue;
            } else {
                // if inliers less than sample limit then use all of them to estimate model.
                // if estimation fails break iterative loop.
                if (!estimator->leastSquaresFitting(lo_inliers, lo_score->inlier_number, *lo_model)) break;
            }

            quality->getNumberInliers(lo_score, lo_model->returnDescriptor(), lo_model->threshold, true,
                                      lo_inliers);

            // only for test
            lo_iterative_iters++;
            //
        }

        bool fail = false;
        // if threshold differs, so there was fail.
        if (fabsf(lo_model->threshold - threshold) > 0.00001) {
            fail = true;
            // get original threshold back in case lo iterative ransac had break.
            lo_model->threshold = threshold;
        }

        return fail;
    }


    /*
     * Iterative LO Ransac
     * Reduce threshold of current model
     * Estimate model parametres with all inliers
     * Evaluate model
     * Get inliers
     * Repeat until iteration < lo iterative iterations
     */
    bool getScoreUnlimited(Score *lo_score, Model *lo_model, Score *best_score, int *lo_inliers) {
        for (unsigned int iterations = 0; iterations < max_iters; iterations++) {
            lo_model->threshold -= threshold_step;

            // break if there are not enough inliers to estimate non minimal model
            if (lo_score->inlier_number <= lo_model->sample_size) break;
            if (!estimator->leastSquaresFitting(lo_inliers, lo_score->inlier_number, *lo_model)) break;
            quality->getNumberInliers(lo_score, lo_model->returnDescriptor(), lo_model->threshold, true,
                                      lo_inliers);

            // break if best score is bigger, because after all points normalization and
            // decreasing threshold lo score could not be bigger in next iterations.
            if (best_score->bigger(lo_score)) {
                break;
            }
            // only for test
            lo_iterative_iters++;
            //
        }

        bool fail = false;
        // if threshold differs, so there was fail.
        if (fabsf(lo_model->threshold - threshold) > 0.00001) {
            fail = true;
            // get original threshold back in case lo iterative ransac had break.
            lo_model->threshold = threshold;
        }

        return fail;
    }


    unsigned int getNumberIterations () override {
        return lo_iterative_iters;
    }

};
}}
#endif //USAC_ITERATIVELOCALOPTIMIZATION_H
