// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "ransac.hpp"
#include "../local_optimization/irls.hpp"
#include "../local_optimization/inner_local_optimization.hpp"
#include "../local_optimization/graphcut.hpp"
#include "../sprt.hpp"

#include "../sampler/prosac_sampler.hpp"
#include "../termination_criteria/prosac_termination_criteria.hpp"

void Ransac::run() {
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
    int sample [estimator->SampleNumber()];

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
//        std::cout << "generate sample\n";
        sampler->generateSample(sample);

//         std::cout << "samples are generated\n";

        number_of_models = estimator->EstimateModel(sample, models);

//         std::cout << "minimal model estimated\n";

        for (unsigned int i = 0; i < number_of_models; i++) {
//             std::cout << i << "-th model\n";

            if (is_sprt) {
//                std::cout << "sprt verify\n";
                is_good_model = sprt->verifyModelAndGetModelScore(models[i], iters, best_score->inlier_number,
                                                                 current_score);
//                std::cout << "sprt verified\n";

                if (!is_good_model) {
//                    std::cout << "model is bad\n";

                    // do not skip bad model until predefined iterations reached
                    if (iters >= model->max_hypothesis_test_before_sprt) {
                        iters++;
//                        std::cout << "skip bad model in iteration " << iters << "\n";
                        continue;
                    }
//                    else {
//                        std::cout << "sprt " << current_score->inlier_number << "\n";
//                        quality->getNumberInliers(current_score, models[i]);
//                        std::cout << "std " << current_score->inlier_number << "\n";
//                    }
                }
//                else {
//                    std::cout << "model is good\n";
//                }
            } else {
//                std::cout << "Get quality score\n";
                 quality->getNumberInliers(current_score, models[i]->returnDescriptor());
            }
//
//           std::cout << "Ransac, iteration " << iters << "; score " << current_score->inlier_number << "\n";
//            std::cout << models[i]->returnDescriptor() << "\n\n";

            if (current_score->bigger(best_score)) {

//                  std::cout << "update best score\n";

                // update current model and current score by inner and iterative local optimization
                // if inlier number is too small, do not update

                if (LO) {
                    local_optimization->GetModelScore (models[i], current_score);
                }

                // copy current score to best score
                best_score->copyFrom(current_score);

                // remember best model
                best_model->setDescriptor (models[i]->returnDescriptor());

//                  std::cout << "Ransac, update best score " << best_score->inlier_number << '\n';

                // Termination conditions:
                if (is_prosac) {
                    max_iters = ((ProsacTerminationCriteria *) termination_criteria)->
                            getUpBoundIterations(iters, best_model->returnDescriptor());
                } else {
                    max_iters = termination_criteria->getUpBoundIterations (best_score->inlier_number);
                }
                if (is_sprt) {
//                     std::cout << "std = " << max_iters << " vs sprt = " << sprt->getUpperBoundIterations(best_score->inlier_number) << "\n";
//                    std::cout << "sprt (usac) = " << sprt->updateSPRTStopping(best_score->inlier_number) << "\n";
                    max_iters = std::min (max_iters, sprt->getUpperBoundIterations(best_score->inlier_number));
//                    std::cout << "got max iters \n";
                }
//                 std::cout << "max iters prediction = " << max_iters << '\n';
            } // end of if so far the best score
        } // end loop of number of models
        iters++;
    } // end main while loop

//    std::cout << "end:\n";

    if (best_score->inlier_number == 0) {
        std::cout << "Best score is 0. Check it!\n";
        best_model->setDescriptor(cv::Mat_<float>::eye(3,3));
        exit (111);
    }

    // Graph Cut lo was set, but did not run, run it
    if (GraphCutLO && ((GraphCut *)local_optimization)->gc_iterations == 0) {
        // update best model and best score
        local_optimization->GetModelScore(best_model, best_score);
    }

//    std::cout << "Calculate Non minimal model\n";

    Model *non_minimal_model = new Model (model);

//    std::cout << "end best inl num " << best_score->inlier_number << '\n';

    unsigned int previous_non_minimal_num_inlier = 0;

    int * max_inliers = new int[points_size];
    // get inliers from the best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);

    for (unsigned int norm = 0; norm < 1 /* normalizations count */; norm++) {
        /*
         * TODO:
         * Calculate and Save Covariance Matrix and use it next normalization with adding or
         * extracting some points.
         */
//        std::cout << "estimate non minimal\n";
//        std::cout << best_score->inlier_number << " -\n";
        // estimate non minimal model with max inliers
        float * weights = (float *) calloc (points_size, sizeof(float));
        for (int p = 0; p < best_score->inlier_number; p++) {
            weights[p] = 1;
        }
        int * inls = new int[points_size];
        for (int i = 0; i < points_size; i++) {
            inls[i] = i;
        }
        if (! estimator->EstimateModelNonMinimalSample(inls, points_size, weights, *non_minimal_model)) {
//        if (! estimator->EstimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *non_minimal_model)) {
            std::cout << "\033[1;31mNON minimal model 2 completely failed!\033[0m \n";
            break;
        }

        //
//        std::cout << "get non minimal score\n";
        quality->getNumberInliers(current_score, non_minimal_model->returnDescriptor(), model->threshold, true, max_inliers);
//        std::cout << "end get non minimal score\n";

        // Priority is for non minimal model estimation
        std::cout << "non minimal score " << current_score->inlier_number << '\n';

        // break if non minimal model score is less than 80% of the best minimal model score
        if ((float) current_score->inlier_number / best_score->inlier_number < 0.8) {
//            std::cout << "break; non minimal score is significanlty worse than best score: RSC* "
//                         << best_score->inlier_number << " vs PCA " << current_score->inlier_number <<"\n";
            break;
        }

        // if normalization score is less or equal, so next normalization is equal too, so break.
        if (current_score->inlier_number <= previous_non_minimal_num_inlier) {
            // std::cout << "break; previous non minimal score is the same.\n";
            break;
        }

        previous_non_minimal_num_inlier = current_score->inlier_number;

        best_score->copyFrom(current_score);
        best_model->setDescriptor(non_minimal_model->returnDescriptor());
    }

    std::chrono::duration<float> fs = std::chrono::steady_clock::now() - begin_time;
    // ================= here is ending ransac main implementation ===========================
//    std::cout << "get results\n";

    // get final inliers from the best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);
//    std::cout << "FINAL best inl num " << best_score->inlier_number << '\n';
//    std::cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";

    unsigned int lo_inner_iters = 0;
    unsigned int lo_iterative_iters = 0;
    if (model->lo == LocOpt::InItLORsc || model->lo == LocOpt::InItFLORsc) {
        lo_inner_iters = ((InnerLocalOptimization *) local_optimization)->lo_inner_iters;
        lo_iterative_iters = ((InnerLocalOptimization *) local_optimization)->lo_iterative_iters;
    }
    unsigned int gc_iters = 0;
    if (GraphCutLO) {
        gc_iters = ((GraphCut *)local_optimization)->gc_iterations;
    }
    // Store results
    ransac_output = new RansacOutput (best_model, max_inliers,
            std::chrono::duration_cast<std::chrono::microseconds>(fs).count(),
                                      best_score->inlier_number, iters, lo_inner_iters, lo_iterative_iters, gc_iters);

    delete[] inliers;
    delete[] max_inliers;
    delete (current_score);
    delete (best_score);
    delete (best_model);
}
