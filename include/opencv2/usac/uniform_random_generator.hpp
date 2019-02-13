// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_UNIFORMRANDOMGENERATOR_H
#define USAC_UNIFORMRANDOMGENERATOR_H

#include "random_generator.hpp"

namespace cv { namespace usac {
class UniformRandomGenerator : public RandomGenerator {
protected:
    std::mt19937 generator;
    std::uniform_int_distribution<int> generate;
public:
    UniformRandomGenerator() {
        std::random_device rand_dev;
        generator = std::mt19937(rand_dev());
    }

    int getRandomNumber() override {
        return generate(generator);
    }

    void resetGenerator(int min_range, int max_range) override {
        generate = std::uniform_int_distribution<int>(min_range, max_range);
    }

    void generateUniqueRandomSet(int *sample) override {
        for (unsigned int i = 0; i < subset_size; i++) {
            sample[i] = generate(generator);
            for (int j = i - 1; j >= 0; j--) {
                if (sample[i] == sample[j]) {
                    i--;
                    break;
                }
            }
        }
    }

    void generateUniqueRandomSet(int *sample, unsigned int subset_size, unsigned int max) {
        resetGenerator(0, max);
        for (unsigned int i = 0; i < subset_size; i++) {
            sample[i] = generate(generator);
            for (int j = i - 1; j >= 0; j--) {
                if (sample[i] == sample[j]) {
                    i--;
                    break;
                }
            }
        }
    }

    // closed interval <0; max>
    void generateUniqueRandomSet(int *sample, unsigned int max) {
        resetGenerator(0, max);
        for (unsigned int i = 0; i < subset_size; i++) {
            sample[i] = generate(generator);
            for (int j = i - 1; j >= 0; j--) {
                if (sample[i] == sample[j]) {
                    i--;
                    break;
                }
            }
        }
    }
};
}}


#endif //USAC_UNIFORMRANDOMGENERATOR_H
