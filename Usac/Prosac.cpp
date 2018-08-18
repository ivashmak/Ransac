#include "Prosac.h"
#include "Estimator/Estimator.h"

// https://github.com/jordyantunes/GPGPU-Region-Growing-for-Dynamic-3D-Reconstruction/blob/5afa243ee0245ff9723cacf22f30012384a4274f/sample_consensus/include/pcl/sample_consensus/impl/prosac.hpp
// https://github.com/imbinwang/posest/blob/master/prosac.c

void Prosac::run (cv::InputArray input_points, Estimator *estimator2d) {
    int t = 0;
    int m = model->sample_number; // size of sample set
    int n = m;
    int N = total_points;
    int nstar = N;

    // Imagine standard RANSAC drawing T_N samples of size m out of N data points
    int T_N = (int) termination_criteria->getUpBoundIterations(total_points/3, total_points);
    int T_n = T_N;
    for (int i = 0; i < m; i++) {
        T_n *= (n-i)/(N-i);
    }
    float Tn_prime = 1;

    int In_star = model->sample_number; // In_star is number of inliers within set Un*
    int In_min = total_points;


    float kn_star = 1;
    float n0 = 1 - model->desired_prob;
    float fi = 1 - model->desired_prob;

    float PIn_star = 1;

    auto *sample = new int[model->sample_number];
    Score *score = new Score;
    std::vector<int> inliers;

    while (In_star < In_min || kn_star < (log(n0)/log(1-PIn_star))) {
        // 1. choice of the hypothesis generation set
        t += 1;
        if (t == Tn_prime && n < nstar) {
            n += 1;

            float T_n_minus_1 = T_n;
            T_n *= ((float) n + 1) / ((float) n + 1 - (float) m);
            Tn_prime += ceilf (T_n - T_n_minus_1);
        }

        // 2. semi-random sample Mt from size m
        if (Tn_prime >= t) {
            // select m points at random
            sampler->getSample(sample, model->sample_number, total_points);
        }

        // 3. model parameter estimation
        estimator2d->EstimateModel(input_points, sample, *model);

        // 4. model verification
        inliers.clear();
        inliers.reserve(2);
        quality->GetModelScore(estimator2d, model, input_points, true, *score, inliers);
        In_star = (int) inliers.size();

        int inliers_size = std::max((int) most_inliers.size(), model->sample_number);
        // the probability, that an incorrect model calculated from a random sample containing
        // an outlier is supported by a correspondence not included in the sample
        float beta = ((total_points-inliers_size)/total_points) * // prob that sample contains outlier
                     pow(inliers_size/total_points, model->sample_number); // prob that all points in sample are inliers

    }

    delete sample;
}

float factorial(int n) {
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

float P_Rand_n_from_i (int i, int m, int n, float beta) {
    return (float) pow(beta, i-m)*(float)pow(1-beta, n-i-m)*factorial(n-m)/(factorial(i-m)*factorial(n-(i-m)));
}