// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_EVALUATERANDOMGENERATORS_H
#define USAC_EVALUATERANDOMGENERATORS_H

#include "test_precomp.hpp"
#include "tests.hpp"
#include "../include/opencv2/usac/random_generator.hpp"
#include "../include/opencv2/usac/uniform_random_generator.hpp"
#include "../include/opencv2/usac/array_random_generator.hpp"

void calculateEntropy (cv::usac::RandomGenerator * random_generator, int size, const std::string& name);
void getAverageTime (cv::usac::RandomGenerator * random_generator, int size, int unique_set_size, const std::string& name);
void checkUnique (cv::usac::RandomGenerator * random_generator, int size, const std::string& name);
void checkReset (cv::usac::RandomGenerator * random_generator, const std::string& name);

void Tests::evaluateRandomGenerators () {
    cv::usac::RandomGenerator * array_random_generator = new cv::usac::ArrayRandomGenerator;
    cv::usac::RandomGenerator * uniform_random_generator = new cv::usac::UniformRandomGenerator;

    checkReset (array_random_generator, "array");
    checkUnique (array_random_generator, 20, "array");
    // return;

    int size = 10000;
    int uniques_set_size = 200;
    calculateEntropy (array_random_generator, size, "array");
    getAverageTime (array_random_generator, size, uniques_set_size, "array");

    calculateEntropy (uniform_random_generator, size, "shuffle");
    getAverageTime (uniform_random_generator, size, uniques_set_size, "shuffle");
}

void checkReset (cv::usac::RandomGenerator * random_generator, const std::string& name) {
    std::cout << name << " random generator\n";
    
    int size1 = 10000;
    int size2 = 100;
    random_generator->resetGenerator(0, size1);
    
    for (int i = 0; i < size1/2; i++) {
        random_generator->getRandomNumber();
    }    


    random_generator->resetGenerator(0, size2);
    std::vector<int> histogram(size2+1, 0);
    for (int i = 0; i < size2+1; i++) {
        int num = random_generator->getRandomNumber();
        std::cout << num << ' ';
        if (num > size2) {
            std::cout << "WRONG RANGE IN " << name << '\n';
            return;
        }
        histogram[num]++;
    }    
    std::cout << '\n';
    std::cout << "RANGE is OK\n";
    
    for (int i = 0; i < size2+1; i++) {
        // std::cout << histogram[i] << " " ;
        if (histogram[i] != 1) {
            std::cout << name << "is NOT UNIQUE!\n";
            return;
        }
    }

    std::cout << name << " is UNIQUE\n";

}

void checkUnique (cv::usac::RandomGenerator * random_generator, int size, const std::string& name) {
    std::cout << name << " random generator, size =  " << size << "\n";
    random_generator->resetGenerator(0, size);


    for (int shuffle = 0; shuffle < 10; shuffle++) {
        std::vector<int> histogram (size+1, 0);
    
        for (int i = 0; i < size+1; i++) {
            int num = random_generator->getRandomNumber();
            std::cout << num << ' ';
            histogram[num]++;
        }
        std::cout << '\n';

        float E = 0;
        for (int i = 0; i < size+1; i++) {
            // std::cout << histogram[i] << " " ;
            if (histogram[i] != 1) {
                std::cout << name << "is NOT UNIQUE!\n";
                return;
            }
        }
        // now array generator must have shuffle 
    }    
    std::cout << name << " is UNIQUE\n";

}

void calculateEntropy (cv::usac::RandomGenerator * random_generator, int size, const std::string& name) {
    std::cout << name << " random generator, size =  " << size << "\n";
    random_generator->resetGenerator(0, size);

    std::vector<int> histogram (size, 0);

    for (int i = 0; i < size; i++) {
        histogram[random_generator->getRandomNumber()]++;
    }

    float E = 0;
    for (int i = 0; i < size; i++) {
//        std::cout << histogram[i] << " " ;
        if (histogram[i] == 0) continue;
        E -= ((float)histogram[i]/size) * log ((float) histogram[i]/size);
    }
//    std::cout << '\n';

    std::cout << "Entropy (min: 0 (non uniform), max " << (-log ((float) 1/size)) << " (full uniform)): " << E << '\n';
//    std::cout << "------------------------------------------------------------\n";
}

void getAverageTime (cv::usac::RandomGenerator * random_generator, int size, int unique_set_size, const std::string& name) {
//    std::cout << name << " random generator, size =  " << size << " unique_set_size = " << unique_set_size << "\n";

    random_generator->resetGenerator(0, size);

    auto begin_time = std::chrono::steady_clock::now();

    for (int i = 0; i < size; i++) {
//        std::cout << random_generator->getRandomNumber() << " ";
        random_generator->getRandomNumber();
    }
//    std::cout << '\n';

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    std::chrono::microseconds t = std::chrono::duration_cast<std::chrono::microseconds>(fs);
    std::cout << "getRandomNumber time = " << t.count() << '\n';


    random_generator->resetGenerator(0, size);
    random_generator->setSubsetSize(unique_set_size);

    int * sample = new int [unique_set_size];

    begin_time = std::chrono::steady_clock::now();
    for (int i = 0; i < size; i++) {
        random_generator->generateUniqueRandomSet(sample);
//        for (int j = 0; j < 10; j++) {
//            std::cout << sample[j] << " ";
//        }
//        std::cout << '\n';
    }
    end_time = std::chrono::steady_clock::now();
    fs = end_time - begin_time;
    t = std::chrono::duration_cast<std::chrono::microseconds>(fs);
    std::cout << "generateUniqueRandomSet time = " << t.count() << '\n';
    std::cout << "------------------------------------------------------------\n";
}

#endif //USAC_EVALUATERANDOMGENERATORS_H
