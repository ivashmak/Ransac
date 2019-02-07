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

unsigned int getPointsSize (cv::InputArray points) {
//    std::cout << points.getMat(0).total() << '\n';

    if (points.isVector()) {
        return points.size().width;
    } else {
        return points.getMat().rows;
    }
}

void Ransac::run() {
    auto begin_time = std::chrono::steady_clock::now();

    Score *best_score = new Score, *current_score = new Score;

    std::vector<Model*> models;
    models.push_back (new Model(model));

    // Allocate max size of models for fundamental matrix
    // estimation to avoid reallocation
    if (model->estimator == ESTIMATOR::Fundamental) {
        models.push_back (new Model(model));
        models.push_back (new Model(model));
    }
    Model * best_model = new Model (model);
    unsigned int number_of_models;

    bool is_prosac = model->sampler == SAMPLER::Prosac;

    /*
     * Allocate inliers of points_size, to avoid reallocation in getModelScore()
     */
    int * inliers = new int[points_size];
    int sample [estimator->SampleNumber()];

    //------------- SPRT -------------------
    bool SprtLO = model->sprt;
    bool is_good_model;


    bool LO = model->lo != LocOpt ::NullLO;

    //---------- Graph cut local optimization ----------
    bool GraphCutLO = model->lo == LocOpt ::GC;
    //------------------------------------------

    int iters = 0;
    int max_iters = model->max_iterations;

    // delete, just for test
//    int * best_sample = new int[4];

    while (iters < max_iters) {

        sampler->generateSample(sample);

//      debug
//        bool eq = false;
//        for (int s = 0; s < model->sample_size; s++) {
//            std::cout << sample[s] << " ";
//            for (int j = 0; j < model->sample_size; j++) {
//                if (s == j) continue;
//                if (sample[s] == sample[j]) {
//                    eq = true;
//                }
//            }
//        }
//        std::cout << "\n";
//        if (eq) std::cout << "SAMPLE EQUAL\n";

//         std::cout << "samples are generated\n";

        number_of_models = estimator->EstimateModel(sample, models);

//         std::cout << "minimal model estimated\n";

        for (int i = 0; i < number_of_models; i++) {
//             std::cout << i << "-th model\n";

            if (SprtLO) {
//                std::cout << "sprt verify\n";
                is_good_model = sprt->verifyModelAndGetModelScore(models[i], iters,
                        std::max (best_score->inlier_number, current_score->inlier_number), current_score);
//                std::cout << "sprt verified\n";

                if (!is_good_model) {
                    iters++;
//                    std::cout << "model is bad\n";

                    // do not skip bad model until predefined iterations reached
                    if (iters >= model->max_hypothesis_test_before_sprt) {
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
//            std::cout << "Ransac, iteration " << iters << "; score " << current_score->inlier_number << "\n";
//            std::cout << models[i]->returnDescriptor() << "\n\n";

            if (current_score->bigger(best_score)) {

//                  std::cout << "update best score\n";

                // update current model and current score by inner and iterative local optimization
                // if inlier number is too small, do not update

                if (LO) {
//                    std::cout << "score before LO " << current_score->inlier_number << "\n";
                    local_optimization->GetModelScore (models[i], current_score);
//                    std::cout << "score after LO " << current_score->inlier_number << "\n";
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
                if (SprtLO) {
//                     std::cout << "std = " << max_iters << " vs sprt = " << sprt->getUpperBoundIterations(best_score->inlier_number) << "\n";
//                    std::cout << "sprt (usac) = " << sprt->updateSPRTStopping(best_score->inlier_number) << "\n";
                    max_iters = std::min (max_iters, (int)sprt->getUpperBoundIterations(best_score->inlier_number));
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
    // get inliers from best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);

    for (unsigned int norm = 0; norm < 5 /* normalizations count */; norm++) {
        /*
         * TODO:
         * Calculate and Save Covariance Matrix and use it next normalization with adding or
         * extracting some points.
         */
//        std::cout << "estimate non minimal\n";
//        std::cout << best_score->inlier_number << " -\n";
        // estimate non minimal model with max inliers
        if (! estimator->EstimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *non_minimal_model)) {
//            std::cout << "\033[1;31mNON minimal model completely failed!\033[0m \n";
            break;
        }
//        std::cout << non_minimal_model->returnDescriptor() << " std ndlt\n\n";
        //
//        std::cout << "get non minimal score\n";
        quality->getNumberInliers(current_score, non_minimal_model->returnDescriptor(), non_minimal_model->threshold, true, max_inliers);
//        std::cout << "end get non minimal score\n";

        // Priority is for non minimal model estimation
//        std::cout << "non minimal inlier number " << current_score->inlier_number << '\n';


        // break if non minimal model score is less than 80% of the best minimal model score
        if ((float) current_score->inlier_number / best_score->inlier_number < 0.8) {
            break;
//                std::cout << "|I|best = " << best_score->inlier_number << "\n";
//                std::cout << "|I|non minimal = " << current_score->inlier_number << "\n";
//                std::cout << "\033[1;31mNON minimal model has less than 50% of inliers to compare with best score!\033[0m \n";
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
//    std::cout << "get results\n";

    // get final inliers from the best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);
//    std::cout << "FINAL best inl num " << best_score->inlier_number << '\n';
//    std::cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";

    unsigned int lo_inner_iters = 0;
    unsigned int lo_iterative_iters = 0;
    if (LO) {
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

    delete[] current_score;
    delete[] best_score;
    delete[] inliers;
    delete[] max_inliers;
//    delete[] best_model;
}
