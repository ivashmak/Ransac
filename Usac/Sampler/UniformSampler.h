#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "../Estimator/Estimator.h"
#include "../RandomGenerator/ArrayRandomGenerator.h"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
protected:
    std::random_device rand_dev;
    std::mt19937 generator;
    std::uniform_int_distribution<int> generate;

public:
    UniformSampler (int sample_size, const int N_points, const bool reset_time=true) {
        if (reset_time) resetTime();

        this->sample_size = sample_size;
        this->N_points = N_points;

        generator = std::mt19937(rand_dev());
        generate = std::uniform_int_distribution<int>(0, N_points-1);

    }

    void resetGenerator (int maxSize) {
        generate = std::uniform_int_distribution<int>(0, maxSize-1);
    }

    void generateSample (int *sample) override {

        std::vector<int> random_numbers;
        for (int i = 0; i < sample_size; i++) {
//            std::cout << "i = " << i << '\n';
            int rand_number;
            // Generate a random number that has not already been used.
            while (std::find(random_numbers.begin(),
                             random_numbers.end(),
                             (rand_number = generate(generator))) !=
                   random_numbers.end()) {
            }

            random_numbers.push_back(rand_number);
            sample[i] = rand_number;
        }
    }

};


#endif //RANSAC_UNIFORMSAMPLER_H
