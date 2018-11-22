#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include <omp.h>
#include <thread>
#include "../Estimator/Estimator.h"
#include "../Model.h"
#include <cmath>
#include <chrono>

/*
 * Score for model estimation
 */
class Score {
public:
    int inlier_number = 0;
    float score = 0;

    inline bool bigger (const Score * const score2) {
        return score > score2->score;
    }
    inline bool bigger (const Score& score2) {
        return score > score2.score;
    }

    /*
     * Compare score of model evaluation
     */
//    inline bool operator>(const Score& score2) {
//        return score > score2.score;
//    }
//    inline bool operator>(const Score *const score2) {
//        return score > score2->score;
//    }

    void copyFrom (const Score * const score_to_copy) {
        score = score_to_copy->score;
        inlier_number = score_to_copy->inlier_number;
    }
};


class Quality {
public:

	/*
	 * Compute Model Score.
	 * Find number of inliers and calculate coefficient of determination as score of model
	 * https://en.wikipedia.org/wiki/Coefficient_of_determination
	 */
    inline void GetModelScore(Estimator * const estimator,
                       Model * const model,
                       cv::InputArray input_points,
                       int points_size,
                       Score &score,
                       int * inliers,
                       bool get_inliers,
                       bool parallel=false) {

        estimator->setModelParameters(model);
        score.inlier_number = 0;

//        float SS_tot = 0, SS_res = 0;
        if (parallel) {
            int score_inlier_number = 0;
//            std::cout << "PARALLEL MODE\n";

            #pragma omp parallel for reduction (+:score_inlier_number)
            for (int point = 0; point < points_size; point++) {
                if (estimator->GetError(point) < model->threshold) {
                    score_inlier_number++;
                }
            }

            score.inlier_number = score_inlier_number;

        } else {
            // calculate coefficient of determination r^2
//            float * points = (float * ) input_points.getMat().data;
//            float * truth = new float[points_size];
//            auto * params = (float * ) model->returnDescriptor().data;
//            float a = params[0], b = params[1], c = params[2];
//            float mean = 0;
//            int pt;

            if (get_inliers) {
                for (int point = 0; point < points_size; point++) {
                    if (estimator->GetError(point) < model->threshold) {
                        inliers[score.inlier_number++] = point;
                    }    
                }
            } else {
                for (int point = 0; point < points_size; point++) {
                    if (estimator->GetError(point) < model->threshold) {
    //                    pt = 2*point;
    //                    truth[score.inlier_number-1] = points[pt+1];
    //                    mean += truth[score.inlier_number-1];
    //
    //                    // The sum of squares of residuals
    //                    SS_res += (truth[score.inlier_number] + c + a*points[pt])/b) *
    //                            (truth[score.inlier_number] +c + a*points[pt])/b); // y - y' = y - (-c -ax)/b
                        score.inlier_number++;
                    }
                }
            }
//            mean /= score.inlier_number;

            // The total sum of squares
//            for (int i = 0; i < score.inlier_number; i++) {
//                SS_tot += (truth[i] - mean) * (truth[i] - mean) ;
//            }
        }

        score.score = score.inlier_number;

        // store coefficient of determination
//        score.score = 1 - SS_res/SS_tot;

	}


    /*
     * We don't need to get inliers in GetModelScore as we use them only once for estimation
     * non minimal model. As result faster way will be implement separate function for getting
     * inliers. Works same as getModelScore, however save inlier's indexes.
     */
    void getInliers (Estimator * const estimator, int points_size, Model * const  model, int * inliers, bool parallel=false) {
        estimator->setModelParameters(model);

	    int num_inliers = 0;
	    for (int point = 0; point < points_size; point++) {
            if (estimator->GetError(point) < model->threshold) {
                inliers[num_inliers] = point;
                num_inliers++;
            }
        }
    }

    /*
     * Calculate average error for any inliers (e.g. GT inliers or output inliers).
     */
    float getAverageError (Estimator * const estimator,
                           Model * const model,
                           const int * const inliers,
                           int inliers_size) {

        estimator->setModelParameters(model);
        float sum_errors = 0;

        for (int point = 0; point < inliers_size; point++) {
            sum_errors += estimator->GetError(inliers[point]);
        }
        return sum_errors/inliers_size;
    }

};


#endif //RANSAC_QUALITY_H