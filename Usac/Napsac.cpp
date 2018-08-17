#include "Napsac.h"

void get_points_under_sphere (cv::InputArray input_points, int center_idx, float radius, std::vector<int> &sphere_set);

void Napsac::run (cv::InputArray input_points, Estimator *estimator2d) {

    int total_points = input_points.size().width;
    auto begin_time = std::chrono::steady_clock::now();

    float iters = 0;
    float max_iters = model->max_iterations;

    Score *best_score = new Score, *current_score = new Score;
    best_score->inlier_number = 0;
    best_score->score = 0;

    int *initial_point = new int[1];
    float radius = model->threshold;

    int *sample = new int[estimator2d->SampleNumber()];
    std::vector<int> inliers;

    std::vector<int> sphere_set;

    while (iters < max_iters) {
        // Select an initial point x0 randomly from all points
        sampler->getSample(initial_point, 1, total_points);

        // Find a set of points Sx0 such that all points in the set lie within a
        // hyper-sphere of radius r, centered around x0
        sphere_set.clear();
        get_points_under_sphere (input_points, initial_point[0], radius, sphere_set);

        // If the number of points in Sx0 is less than the size of the minimal sample,
        // then  this  set  is  discarded,  and  the  sampling  process repeats afresh
        if (sphere_set.size() < estimator2d->SampleNumber()) {
            continue;
        }

        sampler->getSample(sample, estimator2d->SampleNumber(), (int) sphere_set.size());

        for (int i = 0; i < estimator2d->SampleNumber(); i++) {
            sample[i] = sphere_set[sample[i]];
        }

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

    estimator2d->EstimateModelNonMinimalSample(input_points, &most_inliers[0], most_inliers.size(), non_minimal_model);

    quality->total_iterations = (int) iters;
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    quality->total_time = std::chrono::duration_cast<std::chrono::microseconds>(fs);
    delete sample;
}

void get_points_under_sphere (cv::InputArray input_points, int center_idx, float radius, std::vector<int> &sphere_set) {
    cv::Point_<float> * points = (cv::Point_<float> *) input_points.getMat().data;
    int total_points = input_points.size().width;
    cv::Point_<float> center = points[center_idx], point;

    float dist;
    for (int i = 0; i < total_points; i++) {
        point = points[i];
        dist = (float) (sqrt(pow(center.x - point.x, 2) + pow(center.y - point.y, 2)));
        if (dist < radius) {
            sphere_set.push_back(i);
        }
    }
}