#ifndef USAC_UNIFORMRANDOMGENERATOR_H
#define USAC_UNIFORMRANDOMGENERATOR_H

#include <cstdlib>
#include <vector>
#include <random>
#include "RandomGenerator.h"

class UniformRandomGenerator : public RandomGenerator {
protected:
    std::random_device rand_dev;
    std::mt19937 generator;
    std::uniform_int_distribution<unsigned int> generate;
    unsigned int N_points;

public:
    UniformRandomGenerator (unsigned int N_points) {
        this->N_points = N_points;

        generator = std::mt19937(rand_dev());
        generate = std::uniform_int_distribution<unsigned int>(0, N_points-1);
    }

    unsigned int getRandomNumber () override {
        return generate (generator);
    }

    void generateRandomSample (int * sample) override {

    }
};

#endif //USAC_UNIFORMRANDOMGENERATOR_H
