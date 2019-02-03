// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_GRADUALNAPSAC_H
#define USAC_GRADUALNAPSAC_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>
#include "Sampler.h"
#include "../Helper/Drawing/Drawing.h"
#include "../../RandomGenerator/ArrayRandomGenerator.h"
#include "../Utils/Math.h"

/*
 * https://pdfs.semanticscholar.org/cec1/2adbb307124e0c62efbaaa870836c3846b5f.pdf
 * http://www.bmva.org/bmvc/2002/papers/164/full_164.pdf
 *
 */
class ProgressiveNapsac : public Sampler {
private:
    int taking_sample_size_from_block = 0;
    int current_block = 0;
    int taken_sample_counter = 0;
    RandomGenerator * array_randomGenerator; // random generator for k nearest neighbors
    std::vector< std::vector<int> > points_in_blocks;
    int * block_samples;
    int fact_sample_size;
    int max_taken = 20;
    int first_best_blocks;
public:


    ProgressiveNapsac (cv::InputArray input_points, unsigned int sample_size, bool reset_time = true) {
        assert (!input_points.empty());

        this->sample_size = sample_size;
        this->points_size = input_points.size().width;

        float * points = (float *) input_points.getMat().data;

        int rows_split = 10;
        int cols_split = 10;

        /*
         *  Block are rectangles with left up and right down corners (x1,y1) (x2,y2)
            b11 b12 ... b1n
            b21 b22 ... b2n
            ...
            bm1 bm2 ... bmn
         */
        int blocks_size = rows_split*cols_split;
        float * blocks = new float[blocks_size*4];
        float bx = 0, by = 0;

        // get maximum size of image
        float max_x = 0;
        float max_y = 0;
        unsigned int idx;
        for (int i = 0; i < points_size; i++) {
            idx = 2 * i;
            if (max_x < points[idx]) {
                max_x = points[idx];
            }
            if (max_y < points[idx+1]) {
                max_y = points[idx+1];
            }
        }
        // --------------------------

        float step_x = max_x / cols_split;
        float step_y = max_y / rows_split;
        unsigned int blk;
        for (int i = 0; i < rows_split; i++) {
            bx = 0;
            for (int j = 0; j < cols_split; j++) {
                blk = 4*(i*cols_split+j);
                blocks[blk] = bx;
                blocks[blk+1] = by;
                bx += step_x;
                blocks[blk+2] = bx;
                blocks[blk+3] = by+step_y;
                // std::cout << "(" << blocks[blk] << ", " << blocks[blk+1] << ", " <<
                //                     blocks[blk+2] << ", " << blocks[blk+3] << ") ";
            }
            // std::cout << '\n';
            by += step_y;
        }


        points_in_blocks = std::vector< std::vector<int> > (blocks_size);

        for (int i = 0; i < points_size; i++) {
            idx = 2*i;
            for (int b = 0; b < blocks_size; b++) {
                blk = 4*b;
                if (points[idx] >= blocks[blk] && points[idx] <= blocks[blk+2] &&
                    points[idx+1] >= blocks[blk+1] && points[idx+1] <= blocks[blk+3]) {
                    points_in_blocks[b].push_back(i);
                    break;
                }
            }
        }


        // for debug
        cv::Mat img = cv::imread ("../dataset/image1.jpg");
        for (int b = 0; b < blocks_size; b++) {
            cv::Scalar color = cv::Scalar (random() % 256, random() % 256, random() % 256);

            cv::rectangle (img, cv::Point_<float>(blocks[4*b], blocks[4*b+1]),
                                 cv::Point_<float>(blocks[4*b+2], blocks[4*b+3]), color, 8);

            for (int p = 0; p < points_in_blocks[b].size(); p++) {
                cv::circle(img, cv::Point_<float>(points[2*points_in_blocks[b][p]],
                                                  points[2*points_in_blocks[b][p]+1]), 3, color, -1);
            }
        }
        cv::imshow ("rect ", img);
//        cv::waitKey (0);
        cv::imwrite ("../results/blocks.jpg", img);
        // -------------------------

        /*
         * Complexity O(n*log(n))
         *   https://en.cppreference.com/w/cpp/algorithm/sort
         */
        std::sort(points_in_blocks.begin(), points_in_blocks.end(), [] (const std::vector<int>& v1,
                                                                        const std::vector<int>& v2) {
            return v1.size() > v2.size();
        });
        std::cout << "blocks sorted\n";
        // for debug
        for (int b = 0; b < blocks_size; b++) {
            std::cout << points_in_blocks[b].size() << '\n';
        }
        std::cout << "===========================================================\n";
        // ---------------------

        array_randomGenerator = new ArrayRandomGenerator;
        array_randomGenerator->setSubsetSize(sample_size);
        if (reset_time) array_randomGenerator->resetTime();

        current_block = 0;
        taken_sample_counter = 0;
        k_iterations = 0;
        fact_sample_size = fast_factorial (sample_size);
        first_best_blocks = std::min ((int) points_in_blocks.size(), 10);

        getSamplesFromBlocks ();
    }

    void getSamplesFromBlocks () {
        // std::cout << "getSamplesFromBlocks " << '\n';
        // quantity of all possible combination of taken sample_size from k nearest neighbors
        /*
                                             n         n!          (n-k+1) * (n-k+2) *...* n
            taking_sample_size_from_block = ( ) = ----------- = ---------------------------
                                             k     k! (n - k)!          k!
            k is sample_size, n is number of points in block

            This value is large enough. We need to be carefull.
            So in case data has big noise and points in block are not inliers, then
            take only 20 (should be set as parameter).

            taking_sample_size_from_block = min (taking_sample_size_from_block, 20);
        */
        taking_sample_size_from_block = 1;
        for (int i = points_in_blocks[current_block].size() - sample_size + 1; i < points_in_blocks[current_block].size(); i++) {
            taking_sample_size_from_block *= i;
        }
        taking_sample_size_from_block = std::min (taking_sample_size_from_block/fact_sample_size, max_taken);
        block_samples = &points_in_blocks[current_block][0];

        array_randomGenerator->resetGenerator(0, points_in_blocks[current_block].size()-1);
    }

    void generateSample (int *sample) override {
        // std::cout << "generateSample\n";

        if (taken_sample_counter > taking_sample_size_from_block) {
            /*
             *   If first best blocks used -> start again.
             */
            current_block = (current_block + 1) % first_best_blocks;
            getSamplesFromBlocks();
            taken_sample_counter = 0;
        }

        array_randomGenerator->generateUniqueRandomSet(sample);

        for (int i = 0; i < sample_size; i++) {
            sample[i] = block_samples[sample[i]];
        }

        // std::cout << "sampled generated\n";

        taken_sample_counter++;
        k_iterations++;
    }


    bool isInit () override {
        return true;
    }
};



#endif //USAC_GRADUALNAPSAC_H
