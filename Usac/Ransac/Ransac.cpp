#include "Ransac.h"
#include "../LocalOptimization/RansacLocalOptimization.h"

int getPointsSize (cv::InputArray points) {
//    std::cout << points.getMat(0).total() << '\n';

    if (points.isVector()) {
        return points.size().width;
    } else {
        return points.getMat().rows;
    }
}

void Ransac::run(cv::InputArray input_points, bool LO) {
    /*
     * Check if all components are initialized and safe to run
     * todo: add more criteria
     */
    assert(!input_points.empty());
    assert(estimator != nullptr);
    assert(model != nullptr);
    assert(quality != NULL);
    assert(sampler != nullptr);
    assert(termination_criteria != nullptr);
    assert(sampler->isInit());

//    std::cout << "asserted\n";

    auto begin_time = std::chrono::steady_clock::now();

    int points_size = getPointsSize(input_points);

//    std::cout << "Points size " << points_size << '\n';

    // initialize termination criteria
    termination_criteria->init(model);

    int iters = 0;
    int max_iters = model->max_iterations;

    Score *best_score = new Score, *current_score = new Score;
    best_score->inlier_number = 0;
    best_score->score = 0;

    int *sample = new int[estimator->SampleNumber()];

    Model **models = new Model*[1];
    Model *best_model = new Model (*model);
    models[0] = model;

//    std::cout << "begin\n";

    /*
     * Allocate inliers of points_size, to avoid reallocation in getModelScore()
     */
    int * inliers = new int[points_size];
    int * max_inliers = new int[points_size];

    LO = false;
    bool best_LO_model = false;
    unsigned int lo_runs = 0;
    unsigned int lo_iterations = 0;

    LocalOptimization * lo_ransac;    
    if (LO) lo_ransac = new RansacLocalOptimization (model, sampler, termination_criteria, quality, estimator);


    while (iters < max_iters) {
        sampler->generateSample(sample);

        int number_of_models = estimator->EstimateModel(sample, models);

        for (int i = 0; i < number_of_models; i++) {
//            std::cout << i << "-th model\n";

            // we need inliers only for local optimization
            quality->GetModelScore(estimator, models[i], input_points, points_size, *current_score, inliers, LO);


            // ---------- for debug ----------------------
            Drawing drawing;
            cv::Mat img = cv::imread ("../dataset/image1.jpg");
            // std::vector<int> inl;
            // for (int i = 0; i < lo_score->inlier_number; i++) {
            //     inl.push_back(lo_inliers[i]);
            // }
            // drawing.showInliers(input_points, inl, img);
            cv::Point_<float> * pts = (cv::Point_<float> *) input_points.getMat().data;
            drawing.draw_model(models[0], cv::Scalar(255, 0, 0), img, false);
            cv::circle (img, pts[sample[0]], 3, cv::Scalar(255, 255, 0), -1);
            cv::circle (img, pts[sample[1]], 3, cv::Scalar(255, 255, 0), -1);
            cv::imshow("samples img", img); cv::waitKey(0);
            cv::imwrite( "../results/"+model->model_name+"_"+std::to_string(iters)+".jpg", img);
            // -------------------------------------------

            if (*current_score > best_score) {

                // std::cout << "current score = " << current_score->score << '\n';

                if (LO) {

                    Score *lo_score = new Score;
                    Model *lo_model = new Model (*model);
                    bool can_finish;
                    unsigned int lo_iters = lo_ransac->GetLOModelScore (*lo_model, *lo_score, current_score, 
                                                    input_points, points_size, iters, inliers, &can_finish);

                    // std::cout << "lo iterations " << lo_iters << '\n';
                    // std::cout << "lo score " << lo_score->inlier_number << '\n';
                    // std::cout << "curr score " << current_score->inlier_number << '\n';
                    // std::cout << "lo model best found " << lo_model->returnDescriptor() << '\n';
                    if (*lo_score > current_score) {
                        // std::cout << "LO score is better than current score\n";
                        best_score->copyFrom(lo_score);
                        best_LO_model = true;

                        best_model->setDescriptor(lo_model->returnDescriptor());
                    } else{
                        best_score->copyFrom(current_score);
                        best_LO_model = false;

                        best_model->setDescriptor(models[i]->returnDescriptor());
                    }

                    // std::cout << "best model descriptor " << best_model->returnDescriptor() << '\n';
                    // std::cout << "best score " << best_score->inlier_number << '\n';

                    iters += lo_iters;
                    lo_iterations += lo_iters;
                    lo_runs++;

                    if (can_finish) {
                        iters++;
                        // std::cout << "CAN FINISH\n";
                        goto end;
                    }
                } else {

                    // copy current score to best score
                    best_score->inlier_number = current_score->inlier_number;
                    best_score->score = current_score->score;

                    // remember best model
                    best_model->setDescriptor (models[i]->returnDescriptor());

                }

                max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, points_size);
               // std::cout << "max iters prediction = " << max_iters << '\n';
            }
        }

       // std::cout << "current iteration = " << iters << '\n';
        iters++;
    }

    end:

    
    /*
     * If best model is LO model, so lo used non minimal model estimation
     * so we don't need to run it again. And model will be equal to non minimal model.
     */
    if (!best_LO_model) {
        // get inliers from best model
        quality->getInliers(estimator, points_size, best_model, max_inliers);
        // estimate model with max inliers
        estimator->EstimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *best_model);
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    // here is ending ransac main implementation

    quality->GetModelScore(estimator, best_model, input_points, points_size, *best_score, max_inliers, true);

    // Store results
    ransac_output = new RansacOutput (best_model, max_inliers,
            std::chrono::duration_cast<std::chrono::microseconds>(fs).count(), best_score->inlier_number, iters, lo_iterations, lo_runs);

    // if (LO) {
    //     delete lo_ransac;
    // }
    // delete sample, current_score, best_score, inliers, max_inliers, best_model;
}
