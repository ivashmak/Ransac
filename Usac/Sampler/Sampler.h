#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

class Sampler {
protected:
    int sample_size;
    int N_points;
    int k_iterations = 0;
public:
    virtual void getSample (int *points) = 0;
    void resetTime () {
        srand (time(NULL));
    }

    void setSampleSize (int sample_size) {
        this->sample_size = sample_size;
    }
    void setNumberPoints (int N_points) {
        this->N_points = N_points;
    }

    int getNumberOfIterations () {
        return k_iterations;
    }
};

#endif //RANSAC_SAMPLER_H