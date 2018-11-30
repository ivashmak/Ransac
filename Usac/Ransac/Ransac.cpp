#include "Ransac.h"
#include "../LocalOptimization/RansacLocalOptimization.h"
#include "../Estimator/DLT/DLT.h"
#include "../LocalOptimization/GraphCut.h"
#include "../SPRT.h"

#include "../Sampler/ProsacSampler.h"
#include "../TerminationCriteria/ProsacTerminationCriteria.h"

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

    auto begin_time = std::chrono::steady_clock::now();

    int points_size = getPointsSize(input_points);

//   std::cout << "Points size " << points_size << '\n';

    // initialize termination criteria
    termination_criteria->init(model, points_size);

    // initialize quality
    quality->init(points_size, model->threshold, estimator);

    Score *best_score = new Score, *current_score = new Score;

    std::vector<Model*> models;
    models.push_back (new Model(*model));

    // Allocate max size of models for fundamental matrix
    // estimation to avoid reallocation
    if (model->estimator == ESTIMATOR::Fundamental) {
        models.push_back (new Model(*model));
        models.push_back (new Model(*model));
    }

    Model *best_model = new Model;
    best_model->copyFrom (model);

    // get information from model about LO
    bool LO = model->LO;
    bool GraphCutLO = model->GraphCutLO;
    bool SprtLO = model->SprtLO;
    
//    std::cout << "General ransac Local Optimization " << LO << "\n";
//    std::cout << "graphCut Local Optimization " << GraphCutLO << "\n";
//    std::cout << "Sprt Local Optimization " << SprtLO << "\n";

    // prosac
    ProsacTerminationCriteria * prosac_termination_criteria;
    ProsacSampler * prosac_sampler;
    bool is_prosac = model->sampler == SAMPLER::Prosac;
    if (is_prosac) {
        prosac_sampler = (ProsacSampler *) sampler;
        prosac_termination_criteria = (ProsacTerminationCriteria *) termination_criteria;
    }
    //
    // we want inliers in case using prosac sampler or local optimization
    bool get_inliers = LO || is_prosac;

    /*
     * Allocate inliers of points_size, to avoid reallocation in getModelScore()
     */
    int * inliers = new int[points_size];
    int * sample = new int[estimator->SampleNumber()];

    unsigned int lo_runs = 0;
    unsigned int lo_iterations = 0;
    unsigned int number_of_models;

    LocalOptimization * lo_ransac;
    Score *lo_score;
    Model *lo_model;   
    if (LO) {
        lo_score = new Score;
        lo_model = new Model; lo_model->copyFrom(model);
        lo_ransac = new RansacLocalOptimization (model, sampler, termination_criteria, quality, estimator);
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
        graphCut->init(points_size, model, estimator, getNeighbors());
    }

    int iters = 0;
    int max_iters = model->max_iterations;

    // delete, just for test
    int * best_sample = new int[4];

    while (iters < max_iters) {

        if (is_prosac) {
            prosac_sampler->generateSampleProsac (sample, prosac_termination_criteria->getStoppingLength());
        } else {
            sampler->generateSample(sample);
        }

        // sample[0] = 20;
        // sample[1] = 25;
        // sample[2] = 26;
        // sample[3] = 27;        

//         std::cout << "samples are generated\n";

        number_of_models = estimator->EstimateModel(sample, models);

//         std::cout << "minimal model estimated\n";

        for (int i = 0; i < number_of_models; i++) {
//             std::cout << i << "-th model\n";

            if (GraphCutLO) {
                // if LO is true than get inliers
                // we need inliers only for local optimization

                graphCut->labeling(models[i]->returnDescriptor(), current_score);

                if (SprtLO) {
                    // we don't no model score, no inliers
                    // as soon as we use graph cut labeling, the inlier number is score
                    is_good_model = sprt->verifyModelAndGetModelScore(models[i], iters,
                          std::max ((int) best_score->score, (int) current_score->score)/*inlier_number*/, false, nullptr);
                    if (!is_good_model) {
                        iters++;
                        continue;
                    }
                }
            } else
            if (SprtLO) {
                is_good_model = sprt->verifyModelAndGetModelScore(models[i], iters,
                        std::max (best_score->inlier_number, current_score->inlier_number), true, current_score);

//                std::cout << "sprt decision " << good << "\n";
//                std::cout << "current num inl " << current_score->inlier_number << "\n";
                if (!is_good_model) {
//                    std::cout << "SKIP MODEL\n";
//                    std::cout << "inlier number is " << current_score->inlier_number << "\n";
                    iters++;
                    continue;
                }
            } else {
                quality->getNumberInliers(current_score, models[i]->returnDescriptor());
            }


            Drawing drawing;
            cv::Mat img1 = cv::imread ("../dataset/homography/LePoint3A.png");
            cv::Mat img2 = cv::imread ("../dataset/homography/LePoint3B.png");
            cv::Mat pts1 = input_points.getMat().colRange (0, 2);
            cv::Mat pts2 = input_points.getMat().colRange (2, 4);
            cv::hconcat (pts1, cv::Mat_<float>::ones(points_size, 1), pts1);
            cv::hconcat (pts2, cv::Mat_<float>::ones(points_size, 1), pts2);
            std::cout << sample[0] << " " << sample[1] << " " << sample[2] << " " << sample[3] << "\n";
            std::cout << pts1.row(sample[0]) << "\n" << pts1.row(sample[1]) << "\n" << pts1.row(sample[2]) << "\n" <<pts1.row(sample[3]) << "\n";
            std::cout << "-----------------------------\n";
            drawing.drawErrors (img1, img2, pts1, pts2, models[i]->returnDescriptor());
            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[0], 0), pts1.at<float>(sample[0], 1)), 9, cv::Scalar(255, 255, 0), -1);
            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[1], 0), pts1.at<float>(sample[1], 1)), 9, cv::Scalar(255, 255, 0), -1);
            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[2], 0), pts1.at<float>(sample[2], 1)), 9, cv::Scalar(255, 255, 0), -1);
            cv::circle (img1, cv::Point_<float>(pts1.at<float>(sample[3], 0), pts1.at<float>(sample[3], 1)), 9, cv::Scalar(255, 255, 0), -1);
            cv::hconcat (img1, img2, img1);
            cv::imshow ("homography", img1);
            cv::waitKey(0);

           // std::cout << "current num inl " << current_score->inlier_number << "\n";
           // std::cout << "current score " << current_score->score << "\n";
//            std::cout << models[i]->returnDescriptor() << "\n\n";

            if (current_score->bigger(best_score)) {

//                  std::cout << "current score = " << current_score->score << '\n';
                if (get_inliers) {
                    quality->getInliers(models[i]->returnDescriptor(), inliers);
                }

                if (LO) {
                    lo_ransac->GetLOModelScore (lo_model, lo_score,
                            current_score, input_points, points_size, iters, inliers);
//                     std::cout << "lo score " << lo_score->inlier_number << '\n';
                    if (lo_score->bigger(current_score)) {
                        // std::cout << "LO score is better than current score\n";
                        best_score->copyFrom(lo_score);
                        best_model->setDescriptor(lo_model->returnDescriptor());
//                        std::cout << "best model " << best_model->returnDescriptor() << "\n";
                    } else{
                        best_score->copyFrom(current_score);
                        best_model->setDescriptor(models[i]->returnDescriptor());
                    }

                    // no need, just for experiments
                    lo_runs++;
                    //
                } else {
                    // copy current score to best score
                    best_score->copyFrom(current_score);

//                     std::cout << "best score inlier number " << best_score->inlier_number << '\n';
//                     std::cout << "best score " << best_score->score << '\n';

                    // remember best model
                    best_model->setDescriptor (models[i]->returnDescriptor());
                }


                best_sample[0] = sample[0];
                best_sample[1] = sample[1];
                best_sample[2] = sample[2];
                best_sample[3] = sample[3];                

                unsigned int inlier_number;
                /*
                 * If we use graph cut local optimization, so number of inliers is score (?).
                 */
                if (GraphCutLO) {
                    inlier_number = std::max ((int) best_score->score, (int) best_score->inlier_number);
                } else {
                    inlier_number = best_score->inlier_number;
                }

                if (is_prosac) {
                    max_iters = prosac_termination_criteria->
                            getUpBoundIterations(iters, prosac_sampler->getLargestSampleSize(),
                                                 inliers, inlier_number);
                } else {
                    max_iters = termination_criteria->getUpBoundIterations (inlier_number);
                }

                if (SprtLO) {
//                    std::cout << "SPRT stop\n";
                    // std::cout << max_iters << " vs " << sprt->getUpperBoundIterations(inlier_number) << "\n";
                    max_iters = std::min (max_iters, (int)sprt->getUpperBoundIterations(inlier_number));

                }

                // std::cout << "max iters prediction = " << max_iters << '\n';
            }
            // std::cout << "current iteration = " << iters << '\n';
            iters++;
        }
    }

    // Drawing drawing;
    // cv::Mat img1 = cv::imread ("../dataset/homography/LePoint3A.png");
    // cv::Mat img2 = cv::imread ("../dataset/homography/LePoint3B.png");
    // cv::Mat pts1 = input_points.getMat().colRange (0, 2);
    // cv::Mat pts2 = input_points.getMat().colRange (2, 4);
    // cv::hconcat (pts1, cv::Mat_<float>::ones(points_size, 1), pts1);
    // cv::hconcat (pts2, cv::Mat_<float>::ones(points_size, 1), pts2);
    // std::cout << best_sample[0] << " " << best_sample[1] << " " << best_sample[2] << " " << best_sample[3] << "\n";
    // std::cout << pts1.row(best_sample[0]) << "\n" << pts1.row(best_sample[1]) << "\n" << pts1.row(best_sample[2]) << "\n" <<pts1.row(best_sample[3]) << "\n";
    // std::cout << "-----------------------------\n";
    // drawing.drawErrors (img1, img2, pts1, pts2, best_model->returnDescriptor());
    // cv::circle (img1, cv::Point_<float>(pts1.at<float>(best_sample[0], 0), pts1.at<float>(best_sample[0], 1)), 9, cv::Scalar(255, 255, 0), -1);
    // cv::circle (img1, cv::Point_<float>(pts1.at<float>(best_sample[1], 0), pts1.at<float>(best_sample[1], 1)), 9, cv::Scalar(255, 255, 0), -1);
    // cv::circle (img1, cv::Point_<float>(pts1.at<float>(best_sample[2], 0), pts1.at<float>(best_sample[2], 1)), 9, cv::Scalar(255, 255, 0), -1);
    // cv::circle (img1, cv::Point_<float>(pts1.at<float>(best_sample[3], 0), pts1.at<float>(best_sample[3], 1)), 9, cv::Scalar(255, 255, 0), -1);
    // cv::hconcat (img1, img2, img1);
    // cv::imshow ("homography", img1);
    // cv::waitKey(0);

    end:

    int * max_inliers = new int[points_size];
//    std::cout << "Calculate Non minimal model\n";

    Model *non_minimal_model = new Model;
    non_minimal_model->copyFrom (model);

//    std::cout << "end best inl num " << best_score->inlier_number << '\n';
//    std::cout << "end best score " << best_score->score << '\n';

    // usually converges in 4-5 iterations
    unsigned int normalizations = 10;
    unsigned int previous_non_minimal_num_inlier = 0;

    // get inliers from best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);

    for (unsigned int norm = 0; norm < normalizations; norm++) {

        // estimate non minimal model with max inliers
        if (estimator->EstimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *non_minimal_model)) {
            quality->getNumberInliers(current_score, non_minimal_model->returnDescriptor(), true, max_inliers);

            // Priority is for non minimal model estimation
//            std::cout << "non minimal inlier number " << current_score->inlier_number << '\n';

            if ((float) current_score->inlier_number / best_score->inlier_number < 0.5) {
                std::cout << "\033[1;31mNON minimal model has less than 50% of inliers to compare with best score!\033[0m \n";
            }

            // if normalization score is less or equal, so next normalization are equal too, so break.
            if (current_score->inlier_number <= previous_non_minimal_num_inlier) {
                break;
            }

            previous_non_minimal_num_inlier = current_score->inlier_number;

            best_score->copyFrom(current_score);
            best_model->setDescriptor(non_minimal_model->returnDescriptor());
        } else {
            std::cout << "\033[1;31mNON minimal model completely failed!\033[0m \n";
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    // here is ending ransac main implementation

    // get final inliers of the best model
    quality->getInliers(best_model->returnDescriptor(), max_inliers);
//    std::cout << "FINAL best inl num " << best_score->inlier_number << '\n';
//    std::cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";

    float average_error = quality->getAverageError (best_model->returnDescriptor(), max_inliers, best_score->inlier_number);

    // Store results
    ransac_output = new RansacOutput (best_model, max_inliers,
            std::chrono::duration_cast<std::chrono::microseconds>(fs).count(), average_error, best_score->inlier_number, iters, lo_iterations, lo_runs);

    if (LO) {
        delete lo_ransac, lo_model, lo_score;
    }
    if (GraphCutLO) {
        delete graphCut;
    }
    if (SprtLO) {
        delete sprt;
    }
    delete sample, current_score, best_score, inliers, max_inliers, best_model;
}











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