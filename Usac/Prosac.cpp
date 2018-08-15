#include "Prosac.h"
#include "Estimator.h"

void Prosac::run (cv::InputArray input_points, Estimator *estimator2d) {
    int t = 0;
    int m = model->sample_number; // size of sample set
    int n = m;
    int N = total_points;
    int nstar = N;

    int Tn_prime = 1;

    int *sample = new int[model->sample_number];

    while (true) {
        // choice of the hypothesis generation set
        t += 1;
        if (t == Tn_prime && n < nstar) {
            n += 1;
        }

        // semi-random sample Mt from size m
        if (Tn_prime >= t) {
            sampler->getSample(sample, model->sample_number, total_points);
        }

    }

    delete sample;
}