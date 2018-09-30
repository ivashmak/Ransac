#include "Ransac.h"

int getPointsSize (cv::InputArray points) {
//    std::cout << points.getMat(0).total() << '\n';

    if (points.isVector()) {
        return points.size().width;
    } else {
        return points.getMat().rows;
    }
}

void Ransac::run(cv::InputArray input_points, Estimator* const estimator) {
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

    // initialize estimator and termination criteria
    estimator->setPoints(input_points);
    termination_criteria->init(model);

    int iters = 0;
    int max_iters = model->max_iterations;

    Score *best_score = new Score, *current_score = new Score;
    best_score->inlier_number = 0;
    best_score->score = 0;

    int *sample = new int[estimator->SampleNumber()];

    Model **models = new Model*[1];
    models[0] = model;

//    std::cout << "begin\n";

    while (iters < max_iters) {
        sampler->generateSample(sample);

        int number_of_models = estimator->EstimateModel(sample, models);

        for (int i = 0; i < number_of_models; i++) {
//            std::cout << i << "-th model\n";

            estimator->setModelParameters(models[i]);

            current_score->score = 0;
            current_score->inlier_number = 0;

            quality->GetModelScore(estimator, models[i], input_points, points_size, *current_score);

            if (quality->IsBetter(best_score, current_score)) {
                // copy current score to best score
                best_score->inlier_number = current_score->inlier_number;
                best_score->score = current_score->score;

                // remember best model
                best_model = *models[i];

                max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, points_size);
//            std::cout << "max iters prediction " << max_iters << '\n';
            }
        }

//        std::cout << "current iteration " << iters << '\n';

        iters++;
    }

    most_inliers = std::vector<int> (best_score->inlier_number);
    quality->getInliers(estimator, points_size, &best_model, most_inliers);

    estimator->EstimateModelNonMinimalSample(&most_inliers[0], best_score->inlier_number, non_minimal_model);

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;

    // store quality results
    quality->total_iterations = iters;
    quality->points_under_threshold = best_score->inlier_number;
    quality->total_time = std::chrono::duration_cast<std::chrono::microseconds>(fs);

    delete sample, current_score, best_score;
}
