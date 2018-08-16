#include "Prosac.h"
#include "Estimator.h"

void Prosac::run (cv::InputArray input_points, Estimator *estimator2d) {
    int t = 0;
    int m = model->sample_number; // size of sample set
    int n = m;
    int N = total_points;
    int nstar = N;

    int Tn_prime = 1;

    int In_star = model->sample_number; // In_star is number of inliers
    int In_min = total_points;

    auto *sample = new int[model->sample_number];
    Score *score = new Score;

    while (true) {
        // 1. choice of the hypothesis generation set
        t += 1;
        if (t == Tn_prime && n < nstar) {
            n += 1;
        }

        // 2. semi-random sample Mt from size m
        if (Tn_prime >= t) {
            // select m points at random
            sampler->getSample(sample, model->sample_number, total_points);
        }

        // 3. model parameter estimation
        estimator2d->EstimateModel(input_points, sample, *model);

        // 4. model verification

    }

    delete sample;
}