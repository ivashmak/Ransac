#include "Ransac.h"


void Ransac::run(cv::InputArray input_points, Estimator* const estimator) {

    auto begin_time = std::chrono::steady_clock::now();

    int total_points = input_points.size().width;

    // initialize estimator and termination criteria
    estimator->setPoints(input_points);
    termination_criteria->init(model);

    int iters = 0;
    int max_iters = model->max_iterations;

    Score *best_score = new Score, *current_score = new Score;
    best_score->inlier_number = 0;
    best_score->score = 0;

    int *sample = new int[estimator->SampleNumber()];

    while (iters < max_iters) {

        sampler->generateSample(sample);

        estimator->EstimateModel(sample, *model);
        estimator->setModelParametres(model);

        current_score->score = 0;
        current_score->inlier_number = 0;

        quality->GetModelScore(estimator, model, input_points, total_points, *current_score);

        if (quality->IsBetter(best_score, current_score)) {
            best_score = new Score (*current_score);

            // remember best model
            best_model = *model;

            max_iters = (int) termination_criteria->getUpBoundIterations2Points(best_score->inlier_number, total_points);
        }

        iters++;
    }

    quality->getInliers(estimator, total_points, &best_model, most_inliers);

    estimator->EstimateModelNonMinimalSample(&most_inliers[0], best_score->inlier_number, non_minimal_model);

    delete sample;

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;

    // store quality results
    quality->total_iterations = iters;
    quality->points_under_threshold = best_score->inlier_number;
    quality->total_time = std::chrono::duration_cast<std::chrono::microseconds>(fs);
}
