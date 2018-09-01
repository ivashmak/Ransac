#ifndef USAC_UNIFORMRANDOMGENERATOR_H
#define USAC_UNIFORMRANDOMGENERATOR_H

#include <cstdlib>
#include <vector>
#include <random>
#include <algorithm>
#include "RandomGenerator.h"

class UniformRandomGenerator : public RandomGenerator {
protected:
    std::mt19937 generator;
    std::uniform_int_distribution<int> generate;

public:
    UniformRandomGenerator () {
        std::random_device rand_dev;
        generator = std::mt19937(rand_dev());
    }

    int getRandomNumber () override {
        return generate (generator);
    }

    void resetGenerator (int min_range, int max_range) override {
        generate = std::uniform_int_distribution<int>(min_range, max_range);
    }

    void generateUniqueRandomSample (int * sample) override {
        std::vector<int> random_numbers;
        for (int i = 0; i < sample_size; i++) {
            int rand_number;
            // Generate a random number that has not already been used.
            while (std::find(random_numbers.begin(),
                             random_numbers.end(),
                             (rand_number = generate (generator))) !=
                   random_numbers.end()) {
            }

            random_numbers.push_back(rand_number);
            sample[i] = rand_number;
    }

};

#endif //USAC_UNIFORMRANDOMGENERATOR_H
