// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_ITERATIVELOCALOPTIMIZATION_H
#define USAC_ITERATIVELOCALOPTIMIZATION_H

#include "../quality/quality.hpp"
#include "../random_generator/uniform_random_generator.hpp"

class IterativeLocalOptimization {
private:
    unsigned int max_iters, threshold_multiplier, sample_limit;
    bool is_sample_limit;
    float threshold_step, threshold;
    UniformRandomGenerator * uniformRandomGenerator;
    Estimator * estimator;
    Quality * quality;

    int * lo_sample;
public:
    unsigned int lo_iterative_iters = 0;
    ~IterativeLocalOptimization() {
        if (is_sample_limit) {
            delete[] lo_sample;
        }
    }
    IterativeLocalOptimization (unsigned int points_size, Model * model, UniformRandomGenerator * uniformRandomGenerator_,
                                Estimator * estimator_, Quality * quality_) {
        max_iters = model->lo_iterative_iterations;
        threshold_multiplier = model->lo_threshold_multiplier;
        is_sample_limit = model->lo == LocOpt ::InItFLORsc;
        sample_limit = model->lo_sample_size;
        if (is_sample_limit) {
            lo_sample = new int [sample_limit];
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

     /*
      * Iterative LO Ransac
      * Reduce threshold of current model
      * Estimate model parametres from limited sample size of lo model.
      * Evaluate model
      * Get inliers
      * Repeat until iteration < lo iterative iterations
      */

    bool GetScoreLimited (Score * lo_score, Model * lo_model, int * lo_inliers) {
        for (unsigned int iterations = 0; iterations < max_iters; iterations++) {
            lo_model->threshold -= threshold_step;

            // break if there are not enough inliers to estimate non minimal model
            if (lo_score->inlier_number <= lo_model->sample_size) break;
            if (lo_score->inlier_number > sample_limit) {
                // if there are more inliers than limit for sample size then generate at random
                // sample from LO model.
                uniformRandomGenerator->generateUniqueRandomSet(lo_sample, lo_score->inlier_number-1);
                for (unsigned int smpl = 0; smpl < sample_limit; smpl++) {
                    lo_sample[smpl] = lo_inliers[lo_sample[smpl]];
                }
                if (! estimator->LeastSquaresFitting(lo_sample, sample_limit, *lo_model)) continue;
            } else {
                // if inliers less than sample limit then use all of them to estimate model.
                // if estimation fails break iterative loop.
                if (! estimator->LeastSquaresFitting(lo_inliers, lo_score->inlier_number, *lo_model)) break;
            }

            quality->getNumberInliers(lo_score, lo_model->returnDescriptor(), lo_model->threshold, true, lo_inliers);

            // only for test
            lo_iterative_iters++;
            //
        }

        bool fail = false;
        // if threshold differs, so there was fail.
        if (fabsf (lo_model->threshold - threshold) > 0.00001) {
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
    bool GetScoreUnlimited (Score * lo_score, Model * lo_model, Score * best_score, int * lo_inliers) {
        for (unsigned int iterations = 0; iterations < max_iters; iterations++) {
            lo_model->threshold -= threshold_step;

            // break if there are not enough inliers to estimate non minimal model
            if (lo_score->inlier_number <= lo_model->sample_size) break;
            if (! estimator->LeastSquaresFitting(lo_inliers, lo_score->inlier_number, *lo_model)) break;
            quality->getNumberInliers(lo_score, lo_model->returnDescriptor(), lo_model->threshold, true, lo_inliers);

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
        if (fabsf (lo_model->threshold - threshold) > 0.00001) {
            fail = true;
            // get original threshold back in case lo iterative ransac had break.
            lo_model->threshold = threshold;
        }

        return fail;
    }

};

#endif //USAC_ITERATIVELOCALOPTIMIZATION_H
