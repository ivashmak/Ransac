// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_GREEDYLOCALOPTIMIZATION_H
#define USAC_GREEDYLOCALOPTIMIZATION_H


#include "../Quality/Quality.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"

class GreedyLO {
private:
    int iterations = 0;
public:
    void init (Quality * quality, Estimator * estimator, int points_size) {

    }

    /*
        Greedy LO:

        If model is so far the best, then:
        While (true)
            Estimate non minimal model with all inliers of the best model.
            Get model score of the new model.
            if model score is better than best score
                Update best score and best model.
            else
                Two options
                1. break.
                2. 1. Make relaxation, e.g. increase threshold like in iterative ransac LO.
                2. 2. Or get subset of inliers of best model and estimate new model with them like
                      in inner ransac.
                Do small finite number of relaxations if it is not help (e.g. last update > k iters) then break.
                         
     */
    void getLOScore (Score * best_score, Model * best_model, Quality * quality, Estimator * estimator, int points_size) {
        if (best_score->inlier_number <= best_model->sample_size)
            return;

        int inliers_relax_size = points_size/2;

        int * inliers = new int[points_size];
        int * max_inliers = new int [points_size];
        int * new_inliers = new int [inliers_relax_size];

        quality->getInliers(best_model->returnDescriptor(), max_inliers);


        Model * model = new Model (best_model);
        Score * score = new Score;
        model->setDescriptor(best_model->returnDescriptor());
        unsigned int num_relaxations = 0;
        unsigned int num_inl_relaxations = 0;

//        std::cout << "begin best score " << best_score->inlier_number << "\n";
        bool same_threshold = true;
        bool inlier_relaxation = false;
        bool last_inlier_relax = false;
        unsigned int last_update = 0;
        while (true) {
            last_update++;
            if (last_update > 5) {
                break;
            }

            if (last_update > 6) {
//                std::cout << "threshold relaxation\n";
                if (last_update == 5) {
                    model->threshold *= 3;
                } else {
                    // t = 3 * t - iters * x, x = 2t / iters
                    model->threshold = std::max (best_model->threshold, (float)(model->threshold - 2*model->threshold/5));
                }
                same_threshold = false;
            } else
            if (last_update > 1) {
                inlier_relaxation = true;
            }

            if (inlier_relaxation && !last_inlier_relax) {
                std::cout << "inliers relaxation\n";
                inliers_relaxation(max_inliers, best_score->inlier_number, new_inliers);
                if (!estimator->EstimateModelNonMinimalSample(new_inliers, best_score->inlier_number/2, *model)) {
                    continue;
                }
                last_inlier_relax = true;
            } else {
                if (! estimator->EstimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *model)) {
                     inlier_relaxation = true;
                     continue;
                }
                last_inlier_relax = false;
            }


            // for test
            iterations++;
            //

            quality->getNumberInliers(score, model->returnDescriptor(), model->threshold, true, inliers);

            std::cout << "Greedy LO, score " << score->inlier_number << "\n";
//            std::cout << "num relaxations " << num_relaxations << "\n";

            if (score->bigger(best_score)) {
                std::copy (inliers, inliers + score->inlier_number, max_inliers);

                if (! same_threshold) {
                    continue;
                }

                std::cout << "Update best score\n";
                best_score->copyFrom(score);
                best_model->setDescriptor(model->returnDescriptor());
                last_update = 0;
            }
        }
//         std::cout << "end best score " << best_score->inlier_number << "\n";
    }


private:
    void inliers_relaxation (const int * const inliers, int inliers_size, int * new_inliers) {
        UniformRandomGenerator * uniformRandomGenerator = new UniformRandomGenerator;
        uniformRandomGenerator->resetTime();
        uniformRandomGenerator->setSubsetSize(inliers_size/2);
        uniformRandomGenerator->resetGenerator(0, inliers_size-1);
        uniformRandomGenerator->generateUniqueRandomSet(new_inliers);
        for (int i = 0; i < inliers_size/2; i++) {
            new_inliers[i] = inliers[new_inliers[i]];
        }
    }

    void threshold_relaxation (Model * model, unsigned int num_relaxations) {
        model->threshold += model->threshold * 3;
    }

};

#endif //USAC_GREEDYLOCALOPTIMIZATION_H
