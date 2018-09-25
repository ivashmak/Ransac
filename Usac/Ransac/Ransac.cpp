#include "Ransac.h"

int getPointsSize (cv::InputArray points) {
//    std::cout << points.getMat(0).total() << '\n';

    if (points.isVector()) {
        return points.size().width;
    } else {
        return points.getMat(0).rows;
    }
}

void Ransac::run(cv::InputArray input_points, Estimator* const estimator) {
    assert(!input_points.empty());
    assert(estimator != nullptr);
    assert(model != nullptr);
    assert(quality != NULL);
    assert(sampler != nullptr);
    assert(termination_criteria != nullptr);

    auto begin_time = std::chrono::steady_clock::now();

    int points_size = getPointsSize(input_points);

    std::cout << "Points size " << points_size << '\n';

    // initialize estimator and termination criteria
    estimator->setPoints(input_points);
    termination_criteria->init(model);

    /*
     * Check if all components are initialized and safe to run
     * todo: add more criteria
     */
    if (!sampler->isInit()) {
        std::cerr << "Sampler is not initialized\n";
    }

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

        quality->GetModelScore(estimator, model, input_points, points_size, *current_score);

        if (quality->IsBetter(best_score, current_score)) {
            best_score = new Score (*current_score);

            // remember best model
            best_model = *model;

            max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, points_size);
        }

        iters++;
    }

    most_inliers = std::vector<int> (best_score->inlier_number);
    quality->getInliers(estimator, points_size, &best_model, most_inliers);

    estimator->EstimateModelNonMinimalSample(&most_inliers[0], best_score->inlier_number, non_minimal_model);

    delete sample;

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;

    // store quality results
    quality->total_iterations = iters;
    quality->points_under_threshold = best_score->inlier_number;
    quality->total_time = std::chrono::duration_cast<std::chrono::microseconds>(fs);
}
