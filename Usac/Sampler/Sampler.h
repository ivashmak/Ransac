#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

class Sampler {
protected:
    int sample_size;
    int N_points;
    int k_iterations = 0;
    int * sample;
public:

    ~Sampler () {
        free (sample);
    }

    virtual void generateSample (int *points) = 0;

    void resetTime () {
        srand (time(NULL));
    }

    void setSampleSize (int sample_size) {
        this->sample_size = sample_size;
    }

    void setMaxRange (int N_points) {
        this->N_points = N_points;
    }

    int getNumberOfIterations () {
        return k_iterations;
    }

    void reallocateSample () {
        sample = (int *) realloc(sample, sizeof (int) * sample_size);
//        if (!sample) std::cout << "realloc failed\n";
    }

};

#endif //RANSAC_SAMPLER_H