#include "Ransac.h"

void Ransac::run(cv::InputArray input_points, cv::OutputArray &line, Line2DEstimator estimator2d) {

    points = (cv::Point_<float> *) input_points.getMat().data;

    std::vector<cv::Point_<float>> best_line(estimator2d.SampleNumber());

    auto begin_time = std::chrono::steady_clock::now();

    float iters = 0;
    float max_iters = model->max_iterations;

    float inlier_points = estimator2d.SampleNumber();
    float quantity_in_polygon;

    float dist;

    while (iters < max_iters) {

        std::vector<cv::Point_<float>> random_points;

        sampler->getRandomPoints(random_points, estimator2d.SampleNumber());

        quantity_in_polygon = 0;

        // useful for big data
        // #pragma omp parallel for reduction (+:quantity_in_polygon)
        for (int kp = 0; kp < total_points; kp++) {

            dist = estimator2d.GetError2(random_points, kp);

            if (dist < model->threshold) {
                quantity_in_polygon++;
            }
        }

        if (inlier_points < quantity_in_polygon) {
            inlier_points = quantity_in_polygon;

            cv::OutputArray best_line(random_points);

            best_line.copyTo(line);

            max_iters = termination_criteria->getUpBoundIterations(inlier_points, total_points);
            quality->points_under_threshold = inlier_points;
        }
        iters++;
    }

    quality->total_iterations = iters;
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    quality->total_time = std::chrono::duration_cast<std::chrono::milliseconds>(fs);

}
