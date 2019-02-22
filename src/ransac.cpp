// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/ransac.hpp"

void cv::usac::Ransac::run() {
    auto begin_time = std::chrono::steady_clock::now();

    Score *best_score = new Score, *current_score = new Score;

    std::vector<Model*> models;
    models.push_back (new Model(model));

    // Allocate max size of models for fundamental matrix
    // estimation to avoid reallocation
    if (model->estimator == ESTIMATOR::Fundamental ) {
        // for fundamental matrix can be up to 3 solutions
        models.push_back (new Model(model));
        models.push_back (new Model(model));
    } else if (model->estimator == ESTIMATOR::Essential) {
        // for essential matrix can be up to 10 solutions
        for (int sol = 0; sol < 9; sol++) {
            models.push_back(new Model(model));
        }
    }

    Model * best_model = new Model (model);
    unsigned int number_of_models;

    /*
     * Allocate inliers of points_size, to avoid reallocation in getModelScore()
     */
    int * inliers = new int[points_size];
    int sample [estimator->sampleNumber()];

    // prosac
    bool is_prosac = model->sampler == SAMPLER::Prosac;
    
    //------------- SPRT -------------------
    bool is_good_model, is_sprt = model->sprt;
    
    // LO
    bool LO = model->lo != LocOpt ::NullLO;
    bool GraphCutLO = model->lo == LocOpt ::GC;
    //------------------------------------------

    unsigned int iters = 0;
    unsigned int max_iters = model->max_iterations;

    while (iters < max_iters) {
        sampler->generateSample(sample);

        number_of_models = estimator->estimateModel(sample, models);


        for (unsigned int i = 0; i < number_of_models; i++) {

            if (is_sprt) {
                is_good_model = sprt->verifyModelAndGetModelScore(models[i], iters, best_score->inlier_number,
                                                                 current_score);

                if (!is_good_model) {
                    // do not skip bad model until predefined iterations reached
                    if (iters >= model->max_hypothesis_test_before_sprt) {
                        iters++;
                        continue;
                    }
                }
            } else {
                 quality->getNumberInliers(current_score, models[i]->returnDescriptor());
            }

            if (current_score->bigger(best_score)) {

                if (LO) {
                    local_optimization->getModelScore(models[i], current_score);
                }

                // copy current score to best score
                best_score->copyFrom(current_score);

                // remember best model
                best_model->setDescriptor (models[i]->returnDescriptor());

                // Termination conditions:
                if (is_prosac) {
                    max_iters = ((ProsacTerminationCriteria *) termination_criteria)->
                            getUpBoundIterations(iters, best_model->returnDescriptor());
                } else {
                    max_iters = termination_criteria->getUpBoundIterations (best_score->inlier_number);
                }
                if (is_sprt) {
                    max_iters = std::min (max_iters, sprt->getUpperBoundIterations(best_score->inlier_number));
                }
            } // end of if so far the best score
        } // end loop of number of models
        iters++;
    } // end main while loop

    if (best_score->inlier_number == 0) {
        std::cout << "Best score is 0. Check it!\n";
        exit (1);
    }

    // Graph Cut lo was set, but did not run, run it
    if (GraphCutLO && ((GraphCut *)local_optimization)->getNumberIterations() == 0) {
        // update best model and best score
        local_optimization->getModelScore(best_model, best_score);
    }

    Model *non_minimal_model = new Model (model);

    unsigned int previous_non_minimal_num_inlier = 0;

    int * max_inliers = new int[points_size];
    // get inliers from the best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);

    for (unsigned int norm = 0; norm < 4 /* normalizations count */; norm++) {
        // estimate non minimal model with max inliers
        if (!estimator->estimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *non_minimal_model)) {
            break;
        }

        quality->getNumberInliers(current_score, non_minimal_model->returnDescriptor(), model->threshold, true, max_inliers);

        // break if non minimal model score is less than 80% of the best minimal model score
        if ((float) current_score->inlier_number / best_score->inlier_number < 0.8) {
            break;
        }

        // if normalization score is less or equal, so next normalization is equal too, so break.
        if (current_score->inlier_number <= previous_non_minimal_num_inlier) {
            break;
        }

        previous_non_minimal_num_inlier = current_score->inlier_number;

        best_score->copyFrom(current_score);
        best_model->setDescriptor(non_minimal_model->returnDescriptor());
    }

    std::chrono::duration<float> fs = std::chrono::steady_clock::now() - begin_time;
    // ================= here is ending ransac main implementation ===========================

    // get final inliers from the best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);

    unsigned int num_lo_iters = model->lo == NullLO ? 0 : local_optimization->getNumberIterations();

    // Store results
    ransac_output = new RansacOutput (best_model, max_inliers,
            std::chrono::duration_cast<std::chrono::microseconds>(fs).count(),
                                      best_score->inlier_number, iters, num_lo_iters);

    delete[] inliers; delete[] max_inliers;
    delete (current_score); delete (best_score); delete (best_model); delete (non_minimal_model);
    for (int i = 0; i < models.size(); i++) {
        delete models[i];
    }
}
