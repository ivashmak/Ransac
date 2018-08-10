#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "Estimator.h"

class UniformSampler : public Sampler {
public:
    UniformSampler () {
        srand (time(NULL));
    }

    void getSample (int *sample, int npoints, int total_points) {
        for (int i = 0; i < npoints; i++) {
            sample[i] = rand() % total_points;
            for (int j = i-1; j >=0 ; j--) {
                if (sample[j] == sample[i]) {
                    i--;
                    break;
                }
            }
        }
    }

};


#endif //RANSAC_UNIFORMSAMPLER_H
