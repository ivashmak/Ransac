#include "Ransac.h"

void Ransac::run(cv::InputArray input_points, Estimator *estimator2d) {

    int total_points = input_points.size().width;
    auto begin_time = std::chrono::steady_clock::now();

    float iters = 0;
    float max_iters = model->max_iterations;

    Score *best_score = new Score, *current_score = new Score;
    best_score->inlier_number = 0;
    best_score->score = 0;

    int *sample = new int[estimator2d->SampleNumber()];

    while (iters < max_iters) {
        sampler->getSample(sample, estimator2d->SampleNumber(), total_points);

        estimator2d->EstimateModel(input_points, sample, *model);

        current_score->score = 0;
        current_score->inlier_number = 0;

        inliers.clear();
        inliers.reserve(2);
        quality->GetModelScore(estimator2d, model, input_points, true, *current_score, inliers);

        if (quality->IsBetter(best_score, current_score)) {
            best_score->inlier_number = current_score->inlier_number;
            best_score->score = current_score->score;

            inliers.swap(most_inliers);

            // remember best model
            best_model = *model;

            max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, total_points);
            quality->points_under_threshold = best_score->inlier_number;
        }

        iters++;
    }

    int non_minimal_sample = 5;
    sample = new int[non_minimal_sample];
    int *sample_inliers = new int[non_minimal_sample];
    sampler->getSample(sample, non_minimal_sample, most_inliers.size());

    for (int i = 0; i < non_minimal_sample; i++) {
        sample_inliers[i] = most_inliers[sample[i]];
    }

    estimator2d->EstimateModelNonMinimalSample(input_points, sample_inliers, non_minimal_sample, non_minimal_model);


    quality->total_iterations = (int) iters;
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    quality->total_time = std::chrono::duration_cast<std::chrono::microseconds>(fs);
    delete sample;
}
