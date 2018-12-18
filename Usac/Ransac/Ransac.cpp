#include "Ransac.h"
#include "../LocalOptimization/RansacLocalOptimization.h"
#include "../Estimator/DLT/DLT.h"
#include "../LocalOptimization/GraphCut.h"
#include "../SPRT.h"

#include "../Sampler/ProsacSampler.h"
#include "../TerminationCriteria/ProsacTerminationCriteria.h"
#include "../Estimator/HomographyEstimator.h"
#include "../LocalOptimization/GreedyLocalOptimization.h"

int getPointsSize (cv::InputArray points) {
//    std::cout << points.getMat(0).total() << '\n';

    if (points.isVector()) {
        return points.size().width;
    } else {
        return points.getMat().rows;
    }
}

void Ransac::run(cv::InputArray input_points) {
    // todo: initialize (= new) estimator, quality, sampler and others here...

    /*
     * Check if all components are initialized and safe to run
     * todo: add more criteria
     */
//    assert(model->estimator != ESTIMATOR::NullE && model->sampler != SAMPLER::NullS);
    assert(!input_points.empty());
    assert(estimator != nullptr);
    assert(model != nullptr);
    assert(quality != nullptr);
    assert(sampler != nullptr);
    assert(termination_criteria != nullptr);
    assert(sampler->isInit());
    assert(model->neighborsType != NullN);

    auto begin_time = std::chrono::steady_clock::now();

    int points_size = getPointsSize(input_points);

//   std::cout << "Points size " << points_size << '\n';

    // initialize termination criteria
    termination_criteria->init(model, points_size);

    // initialize quality
    quality->init(points_size, model->threshold, estimator);

    Score *best_score = new Score, *current_score = new Score;

    std::vector<Model*> models;
    models.push_back (new Model(model));

    // Allocate max size of models for fundamental matrix
    // estimation to avoid reallocation
    if (model->estimator == ESTIMATOR::Fundamental) {
        models.push_back (new Model(model));
        models.push_back (new Model(model));
    }

    Model *best_model = new Model (model);

    // get information from model about LO
    bool LO = model->LO;
    bool GraphCutLO = model->GraphCutLO;
    bool SprtLO = model->Sprt;
    
//    std::cout << "General ransac Local Optimization " << LO << "\n";
//    std::cout << "graphCut Local Optimization " << GraphCutLO << "\n";
//    std::cout << "Sprt Local Optimization " << Sprt << "\n";

    // prosac
    ProsacTerminationCriteria * prosac_termination_criteria;
    ProsacSampler * prosac_sampler;
    bool is_prosac = model->sampler == SAMPLER::Prosac;
    if (is_prosac) {
        prosac_sampler = (ProsacSampler *) sampler;
        prosac_termination_criteria = (ProsacTerminationCriteria *) termination_criteria;
    }
    //

    /*
     * Allocate inliers of points_size, to avoid reallocation in getModelScore()
     */
    int * inliers = new int[points_size];
    int * sample = new int[estimator->SampleNumber()];

    unsigned int number_of_models;

    LocalOptimization * lo_ransac;
    if (LO) {
        lo_ransac = new RansacLocalOptimization (model, sampler, termination_criteria, quality, estimator, points_size);
    }

    //--------------------------------------------
    SPRT * sprt;
    bool is_good_model;
    if (SprtLO) {
        sprt = new SPRT;
        sprt->initialize(estimator, model, points_size);
    }
    //--------------------------------------------

    // Graph cut local optimization
    GraphCut * graphCut;
    if (GraphCutLO) {
        graphCut = new GraphCut;
        graphCut->init(points_size, model, estimator, quality, model->neighborsType);
        if (model->neighborsType == NeighborsSearch::Nanoflann) {
            graphCut->setNeighbors(neighbors);
        } else {
            graphCut->setNeighbors(neighbors_v);
        }
    }

    // if we have small number of data points, so LO will run with quarter of them,
    // otherwise it requires at least 3 sample size.
    int min_inlier_count_for_LO = std::min ((int) points_size/4, (int) 3 * model->sample_number);
    int gc_runs = 0;

    int iters = 0;
    int max_iters = model->max_iterations;

    // delete, just for test
//    int * best_sample = new int[4];

    while (iters < max_iters) {
//        std::cout << "current iteration = " << iters << '\n';

        if (is_prosac) {
            prosac_sampler->generateSampleProsac (sample, prosac_termination_criteria->getStoppingLength());
        } else {
            sampler->generateSample(sample);
        }

//      debug
//        bool eq = false;
//        for (int s = 0; s < model->sample_number; s++) {
//            std::cout << sample[s] << " ";
//            for (int j = 0; j < model->sample_number; j++) {
//                if (s == j) continue;
//                if (sample[s] == sample[j]) {
//                    eq = true;
//                }
//            }
//        }
//        std::cout << "\n";
//        if (eq) std::cout << "SAMPLE EQUAL\n";

//        sample[0] = 124;
//        sample[1] = 119;
//        sample[2] = 53;
//        sample[3] = 5;

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
                }
//                else {
//                    std::cout << "model is good\n";
//                }
            } else {
                quality->getNumberInliers(current_score, models[i]);
            }

//            std::cout << "current num inl " << current_score->inlier_number << "\n";
//            std::cout << models[i]->returnDescriptor() << "\n\n";

            if (current_score->bigger(best_score)) {

//                GreedyLO * greedyLO = new GreedyLO;
//                greedyLO->getLOScore(current_score, models[i], quality, estimator, points_size);

//                  std::cout << "current score = " << current_score->score << '\n';

                // update current model and current score by inner and iterative local optimization
                // if inlier number is too small, do not update
                if (LO && current_score->inlier_number > min_inlier_count_for_LO) {
//                    std::cout << "score before LO " << current_score->inlier_number << "\n";
                    lo_ransac->GetLOModelScore (models[i], current_score);
//                    std::cout << "score after LO " << current_score->inlier_number << "\n";
                }

               // todo: termination conditions at first

                // update current model and current score by graph cut local optimization
                // if inlier number is too small, do not update
                if (GraphCutLO && current_score->inlier_number > min_inlier_count_for_LO) {
//                    std::cout << "score before GC LO " << current_score->inlier_number << "\n";
                    graphCut->GraphCutLO(models[i], current_score);
//                    std::cout << "score after GC LO " << current_score->inlier_number << "\n";
                    gc_runs++;
                }

                // copy current score to best score
                best_score->copyFrom(current_score);

                // remember best model
                best_model->setDescriptor (models[i]->returnDescriptor());

//                  std::cout << "best score inlier number " << best_score->inlier_number << '\n';

                // only for debug
//                best_sample[0] = sample[0];
//                best_sample[1] = sample[1];
//                best_sample[2] = sample[2];
//                best_sample[3] = sample[3];
                //

                // Termination conditions:
                if (is_prosac) {
                    max_iters = prosac_termination_criteria->
                            getUpBoundIterations(iters, prosac_sampler->getLargestSampleSize(),
                                                 best_model->returnDescriptor());
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
//        exit (111);
    }

    // Graph Cut lo was set, but did not run, run it
    if (GraphCutLO && gc_runs == 0) {
        // update best model and best score
        graphCut->GraphCutLO(best_model, best_score);
    }


    int * max_inliers = new int[points_size];
//    std::cout << "Calculate Non minimal model\n";

    Model *non_minimal_model = new Model (model);

//    std::cout << "end best inl num " << best_score->inlier_number << '\n';

    // usually 4-5 iterations are enough
    unsigned int normalizations = 10;
    unsigned int previous_non_minimal_num_inlier = 0;

    // get inliers from best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);

    for (unsigned int norm = 0; norm < normalizations; norm++) {
//        std::cout << "end estimate non minimal\n";

        // estimate non minimal model with max inliers
        if (estimator->EstimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *non_minimal_model)) {

//            std::cout << "end get non minimal score\n";

            quality->getNumberInliers(current_score, non_minimal_model, true, max_inliers);

            // Priority is for non minimal model estimation
//            std::cout << "non minimal inlier number " << current_score->inlier_number << '\n';

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
        } else {
            std::cout << "\033[1;31mNON minimal model completely failed!\033[0m \n";
            break;
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    // ================= here is ending ransac main implementation ===========================

    // get final inliers from the best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);
//    std::cout << "FINAL best inl num " << best_score->inlier_number << '\n';
//    std::cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";

    unsigned int lo_inner_iters = 0;
    unsigned int lo_iterative_iters = 0;
    if (LO) {
        RansacLocalOptimization * lo_r = (RansacLocalOptimization *) lo_ransac;
        lo_inner_iters = lo_r->lo_inner_iters;
        lo_iterative_iters = lo_r->lo_iterative_iters;
    }
    unsigned int gc_iters = 0;
    if (GraphCutLO) {
        gc_iters = graphCut->gc_iterations;
    }
    // Store results
    ransac_output = new RansacOutput (best_model, max_inliers,
            std::chrono::duration_cast<std::chrono::microseconds>(fs).count(),
                                      best_score->inlier_number, iters, lo_inner_iters, lo_iterative_iters, gc_iters);

    if (LO) {
        delete lo_ransac;
    }
    if (GraphCutLO) {
        delete graphCut;
    }
    if (SprtLO) {
        delete sprt;
    }
    delete sample, current_score, best_score, inliers, max_inliers, best_model;
}








//            Drawing drawing;
//            cv::Mat img1 = cv::imread ("../dataset/homography/LePoint1A.png");
//            cv::Mat img2 = cv::imread ("../dataset/homography/LePoint1B.png");
//            cv::Mat pts1 = input_points.getMat().colRange (0, 2);
//            cv::Mat pts2 = input_points.getMat().colRange (2, 4);
//            cv::hconcat (pts1, cv::Mat_<float>::ones(points_size, 1), pts1);
//            cv::hconcat (pts2, cv::Mat_<float>::ones(points_size, 1), pts2);
//            cv::Mat pt11 = pts1.row (sample[0]);
//            cv::Mat pt12 = pts1.row (sample[1]);
//            cv::Mat pt13 = pts1.row (sample[2]);
//            cv::Mat pt14 = pts1.row (sample[3]);
//
//            cv::Mat pt21 = pts2.row (sample[0]);
//            cv::Mat pt22 = pts2.row (sample[1]);
//            cv::Mat pt23 = pts2.row (sample[2]);
//            cv::Mat pt24 = pts2.row (sample[3]);
//
//            std::cout << sample[0] << " " << sample[1] << " " << sample[2] << " " << sample[3] << "\n";
//            std::cout << pts1.row(sample[0]) << "\n" << pts1.row(sample[1]) << "\n" << pts1.row(sample[2]) << "\n" <<pts1.row(sample[3]) << "\n";
//            std::cout << models[i]->returnDescriptor() << "\n";
//            std::cout << "-----------------------------\n";
//            drawing.drawErrors (img1, img2, pts1, pts2, models[i]->returnDescriptor());
//            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[0], 0), pts1.at<float>(sample[0], 1)), 7, cv::Scalar(255, 255, 0), -1);
//            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[1], 0), pts1.at<float>(sample[1], 1)), 7, cv::Scalar(255, 100, 0), -1);
//            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[2], 0), pts1.at<float>(sample[2], 1)), 7, cv::Scalar(100, 255, 0), -1);
//            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[3], 0), pts1.at<float>(sample[3], 1)), 7, cv::Scalar(255, 255, 255), -1);
//
//            cv::circle (img2, cv::Point_<float>(pts2.at<float>(sample[0], 0), pts2.at<float>(sample[0], 1)), 7, cv::Scalar(255, 255, 0), -1);
//            cv::circle (img2, cv::Point_<float>(pts2.at<float>(sample[1], 0), pts2.at<float>(sample[1], 1)), 7, cv::Scalar(255, 100, 0), -1);
//            cv::circle (img2, cv::Point_<float>(pts2.at<float>(sample[2], 0), pts2.at<float>(sample[2], 1)), 7, cv::Scalar(100, 255, 0), -1);
//            cv::circle (img2, cv::Point_<float>(pts2.at<float>(sample[3], 0), pts2.at<float>(sample[3], 1)), 7, cv::Scalar(255, 255, 255), -1);
//
//            cv::hconcat (img1, img2, img1);
//            cv::imshow ("homography", img1);
//            cv::waitKey(0);





// ---------- for debug ----------------------
//            Drawing drawing;
//            cv::Mat img = cv::imread ("../dataset/image1.jpg");
//            // std::vector<int> inl;
//            // for (int i = 0; i < lo_score->inlier_number; i++) {
//            //     inl.push_back(lo_inliers[i]);
//            // }
//            // drawing.showInliers(input_points, inl, img);
//            cv::Point_<float> * pts = (cv::Point_<float> *) input_points.getMat().data;
//            drawing.draw_model(models[0], cv::Scalar(255, 0, 0), img, false);
//            cv::circle (img, pts[sample[0]], 3, cv::Scalar(255, 255, 0), -1);
//            cv::circle (img, pts[sample[1]], 3, cv::Scalar(255, 255, 0), -1);
//            cv::imshow("samples img", img); cv::waitKey(0);
//            cv::imwrite( "../results/"+model->model_name+"_"+std::to_string(iters)+".jpg", img);
// -------------------------------------------
//            Drawing drawing;
//            cv::Mat img = cv::imread ("../dataset/homography/boatA.png");
//            // std::vector<int> inl;
//            // for (int i = 0; i < lo_score->inlier_number; i++) {
//            //     inl.push_back(lo_inliers[i]);
//            // }
//            // drawing.showInliers(input_points, inl, img);
//            cv::imshow("H", img); cv::waitKey(0);
// -------------------------------------------



//     Drawing drawing;
//     cv::Mat img1 = cv::imread ("../dataset/homography/LePoint1A.png");
//     cv::Mat img2 = cv::imread ("../dataset/homography/LePoint1B.png");
//     cv::Mat pts1 = input_points.getMat().colRange (0, 2);
//     cv::Mat pts2 = input_points.getMat().colRange (2, 4);
//     cv::hconcat (pts1, cv::Mat_<float>::ones(points_size, 1), pts1);
//     cv::hconcat (pts2, cv::Mat_<float>::ones(points_size, 1), pts2);
//     drawing.drawErrors (img1, img2, pts1, pts2, best_model->returnDescriptor());

// quality->getInliers(best_model->returnDescriptor(), inliers);
// for (int i = 0; i < best_score->inlier_number; i++) {
//    cv::circle (img1, cv::Point_<float>(pts1.at<float>(inliers[i], 0), pts1.at<float>(inliers[i], 1)), 4, cv::Scalar(255, 255, 255), -1);
// }
// cv::hconcat (img1, img2, img1);
//     cv::imshow ("homography", img1);
//     cv::imwrite("../results/homography/lepoint1.png", img1);
//     cv::waitKey(0);

