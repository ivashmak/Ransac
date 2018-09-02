#ifndef USAC_UNIFORMRANDOMGENERATOR_H
#define USAC_UNIFORMRANDOMGENERATOR_H

#include <cstdlib>
#include <vector>
#include <random>
#include <algorithm>
#include "RandomGenerator.h"


class Tree  {
public:
    int num = -1;
    Tree *left;
    Tree *right;
};

static void sampleInsert (Tree *tree, int val) {
    if (tree->num == -1) {
        tree->left = new Tree;
        tree->right = new Tree;
        tree->num = val;
    } else {
        if (val < tree->num) {
            sampleInsert(tree->left, val);
        } else {
            sampleInsert(tree->right, val);
        }
    }
}

static bool sampleExist (Tree *tree, int val) {
    if (tree->num == -1) return false;
    if (tree->num == val) return true;
    if (val < tree->num) {
        return sampleExist(tree->left, val);
    } else {
        return sampleExist(tree->right, val);
    }
}

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


    // std::find for unsorted array has linear complexity

//    void generateUniqueRandomSample (int * sample, unsigned int sample_size) override {
//        std::vector<int> random_numbers;
//        for (int i = 0; i < sample_size; i++) {
//            int rand_number;
//            // Generate a random number that has not already been used.
//            while (std::find(random_numbers.begin(),
//                             random_numbers.end(),
//                             (rand_number = generate (generator))) !=
//                   random_numbers.end()) {
//            }
//
//            random_numbers.push_back(rand_number);
//            sample[i] = rand_number;
//        }
//    }

    void generateUniqueRandomSample (int * sample, unsigned int sample_size) override {
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



//    void generateUniqueRandomSample (int * sample, unsigned int sample_size) override {
//        sample[0] = generate (generator);
//        while (sample[0] == (sample[1] = generate (generator)));
//    }



//    void generateUniqueRandomSample (int * sample, unsigned int sample_size) override {
//        Tree *tree = new Tree;
//
//        for (unsigned int i = 0; i < sample_size; i++) {
//            sample[i] = generate (generator);
//
//            Tree * root = tree;
//            if (sampleExist(root, sample[i])) {
//                i--;
//            }
//            root = tree;
//            sampleInsert(root, sample[i]);
//
//        }
//    }

};



#endif //USAC_UNIFORMRANDOMGENERATOR_H
