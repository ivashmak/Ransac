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
    std::vector<int> inliers;

    long time_gen_sample = 0;
    long time_get_model_score = 0;
    long time_estimate_model = 0;

    std::chrono::duration<float> diff_t;
    std::chrono::microseconds to_mcs;

    auto begin_time_gen_sample = std::chrono::steady_clock::now(), end_time_gen_sample = std::chrono::steady_clock::now();


    while (iters < max_iters) {

//        begin_time_gen_sample = std::chrono::steady_clock::now();

        sampler->generateSample(sample);

//        end_time_gen_sample = std::chrono::steady_clock::now();
//        diff_t = end_time_gen_sample - begin_time_gen_sample;
//        to_mcs = std::chrono::duration_cast<std::chrono::microseconds>(diff_t);
//        time_gen_sample += to_mcs.count();


//        begin_time_gen_sample = std::chrono::steady_clock::now();

        estimator2d->EstimateModel(sample, *model);
        estimator2d->setModelParametres(model);

//        end_time_gen_sample = std::chrono::steady_clock::now();
//        diff_t = end_time_gen_sample - begin_time_gen_sample;
//        to_mcs = std::chrono::duration_cast<std::chrono::microseconds>(diff_t);
//        time_estimate_model += to_mcs.count();

        current_score->score = 0;
        current_score->inlier_number = 0;

        inliers.clear();
        inliers.reserve(2);
//        begin_time_gen_sample = std::chrono::steady_clock::now();

        quality->GetModelScore(estimator2d, model, input_points, true, *current_score, inliers);

//        end_time_gen_sample = std::chrono::steady_clock::now();
//        diff_t = end_time_gen_sample - begin_time_gen_sample;
//        to_mcs = std::chrono::duration_cast<std::chrono::microseconds>(diff_t);
//        time_get_model_score += to_mcs.count();

        if (quality->IsBetter(best_score, current_score)) {
            best_score->inlier_number = current_score->inlier_number;
            best_score->score = current_score->score;

            // copy without pointing
            most_inliers = inliers;

            // remember best model
            best_model = *model;

            max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, total_points);
            quality->points_under_threshold = best_score->inlier_number;
        }

        iters++;
    }

    estimator2d->EstimateModelNonMinimalSample(&most_inliers[0], most_inliers.size(), non_minimal_model);

    quality->total_iterations = (int) iters;
    auto end_time = std::chrono::steady_clock::now();

//    std::cout << "time_gen_sample " << time_gen_sample << '\n';
//    std::cout << "time_get_model_score " << time_get_model_score << '\n';
//    std::cout << "time_estimate_model " << time_estimate_model << '\n';


    std::chrono::duration<float> fs = end_time - begin_time;
    quality->total_time = std::chrono::duration_cast<std::chrono::microseconds>(fs);
    delete sample;
}
