#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include <omp.h>
#include <thread>
#include "Estimator/Estimator.h"
#include "Model.h"
#include <cmath>

struct Score {
    int inlier_number;
    float score;
};

class Quality {
public:
    int total_iterations = 0;
	std::chrono::microseconds total_time;
	int points_under_threshold = 0;
public:

	long getComputationTime () {
		return total_time.count();
    }

	int getIterations () {
		return total_iterations;
	}

	int getNumberOfPointsUnderThreshold () {
		return points_under_threshold;
	}


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
                       bool parallel=false) {

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
//            cv::Point_<float> * points = (cv::Point_<float> * ) input_points.getMat().data;
//            std::vector <float> truth (points_size);
//            cv::Mat desc;
//            model->getDescriptor(desc);
//            auto * params = (float * ) desc.data;
//            float a = params[0], b = params[1], c = params[2];
//            float mean = 0;


            for (int point = 0; point < points_size; point++) {
                if (estimator->GetError(point) < model->threshold) {
                    score.inlier_number++;
//                    truth[score.inlier_number-1] = points[point].y;
//                    mean += truth[score.inlier_number-1];
//
//                    // The sum of squares of residuals
//                    SS_res += (truth[score.inlier_number-1] - (-c - a*points[point].x)/b) *
//                            (truth[score.inlier_number-1] - (-c - a*points[point].x)/b);
                }
            }
//            std::cout << r << " " << score.inlier_number << '\n';
//            mean = mean / score.inlier_number;

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
     * Fixing the Locally Optimized RANSAC
	 * http://cmp.felk.cvut.cz/software/LO-RANSAC/Lebeda-2012-Fixing_LORANSAC-BMVC.pdf
	 * Algorithm 2
     */

    /*inline void GetLocallyOptimizedModelScore(Estimator * const estimator,
                              Model * const best_model,
                              cv::InputArray input_points,
                              int points_size,
                              Score &lo_score,
                               int * inliers, int inliers_size, Sampler * const sampler) {
	    Model *lo_model = new Model;
	    estimator->EstimateModelNonMinimalSample(inliers, inliers_size, *lo_model);
	    GetModelScore(estimator, lo_model, input_points, points_size, lo_score);

        int *samples = new int[2];
        sampler->setRange(0, lo_score.inlier_number);
	    sampler->setSampleSize(2);
	    sampler->generateSample(samples);
	    int reps = 10;
	    for (int i = 0; i < reps; i++) {

	    }
	}*/



    /*
     * We don't need to get inliers in GetModelScore as we use them only once for estimation
     * non minimal model. As result faster way will be implement separate function for getting
     * inliers. Works same as getModelScore, however save inlier's indexes.
     */
    void getInliers (Estimator * const estimator, int points_size, Model * const  model, std::vector<int>& inliers, bool parallel=false) {
        estimator->setModelParametres(model);

	    int num_inliers = 0;
	    for (int point = 0; point < points_size; point++) {
            if (estimator->GetError(point) < model->threshold) {
                inliers[num_inliers] = point;
                num_inliers++;
            }
        }
    }

    /*
     * Compare score of model evaluation
     */
    inline bool IsBetter(const Score * const s1, const Score * const s2) {
        return s1->score < s2->score;
    }
};


#endif //RANSAC_QUALITY_H