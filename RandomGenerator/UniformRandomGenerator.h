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

    void generateUniqueRandomSet (int * sample, unsigned int sample_size) override {
        for (unsigned int i = 0; i < sample_size; i++) {
            sample[i] = generate (generator);
            for (int j = i - 1; j >= 0; j--) {
                if (sample[i] == sample[j]) {
                    i--;
                    break;
                }
            }
        }
    }


//    void generateUniqueRandomSet (int * sample, unsigned int sample_size) override {
//        sample[0] = generate (generator);
//        while (sample[0] == (sample[1] = generate (generator)));
//    }

};



#endif //USAC_UNIFORMRANDOMGENERATOR_H
