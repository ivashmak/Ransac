#include "Ransac.h"

void Ransac::run(cv::InputArray input_points, Estimator* const estimator2d) {

    int total_points = input_points.size().width;
    auto begin_time = std::chrono::steady_clock::now();

    float iters = 0;
    float max_iters = model->max_iterations;

    Score *best_score = new Score, *current_score = new Score;
    best_score->inlier_number = 0;
    best_score->score = 0;

    int *sample = new int[estimator2d->SampleNumber()];

    while (iters < max_iters) {

        sampler->generateSample(sample);

        estimator2d->EstimateModel(sample, *model);
        estimator2d->setModelParametres(model);

        current_score->score = 0;
        current_score->inlier_number = 0;

        quality->GetModelScore(estimator2d, model, input_points, total_points, *current_score);

        if (quality->IsBetter(best_score, current_score)) {
            best_score = new Score (*current_score);

            // remember best model
            best_model = *model;

            max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, total_points);
        }

        iters++;
    }

    quality->getInliers(estimator2d, total_points, &best_model, most_inliers);

    estimator2d->EstimateModelNonMinimalSample(&most_inliers[0], best_score->inlier_number, non_minimal_model);

    delete sample;

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;

    quality->total_iterations = (int) iters;
    quality->points_under_threshold = best_score->inlier_number;
    quality->total_time = std::chrono::duration_cast<std::chrono::microseconds>(fs);

}
