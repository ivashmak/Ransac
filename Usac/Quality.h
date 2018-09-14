#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include <omp.h>
#include <thread>
#include "Estimator/Estimator.h"
#include "Model.h"
//#include <opencv2/core/core.hpp>

struct Score {
    int inlier_number;
    float score;
};

static int f()
{
    static int i = 0;
    return i++;
}

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



    inline void GetModelScore(Estimator * const estimator,
                       Model * const model,
                       cv::InputArray input_points,
                       int points_size,
                       Score &score,
                       bool parallel=false) {

        float SS_tot = 0, SS_res = 0;
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
            cv::Point_<float> * points = (cv::Point_<float> * ) input_points.getMat().data;

            std::vector <float> truth (points_size);
            cv::Mat desc;
            model->getDescriptor(desc);
            auto * params = (float * ) desc.data;
            float a = params[0], b = params[1], c = params[2];
            float mean = 0;

            for (int point = 0; point < points_size; point++) {
                if (estimator->GetError(point) < model->threshold) {
                    score.inlier_number++;
                    truth[score.inlier_number-1] = points[point].y;
                    mean += truth[score.inlier_number-1];
                    SS_res += (truth[score.inlier_number-1] - (-c - a*points[point].x)/b) *
                            (truth[score.inlier_number-1] - (-c - a*points[point].x)/b);
                }
            }

            mean = mean / score.inlier_number;

            for (int i = 0; i < score.inlier_number; i++) {
                SS_tot += (truth[i] - mean) * (truth[i] - mean) ;
            }
        }

//        score.score = score.inlier_number;
        score.score = 1 - SS_res/SS_tot;

	}


    void getInliers (Estimator * const estimator, int points_size, Model * const  model, cv::OutputArray inliers, bool parallel=false) {
        estimator->setModelParametres(model);

        std::vector<int> inliers_(points_size);
	    int num_inliers = 0;
	    for (int point = 0; point < points_size; point++) {
            if (estimator->GetError(point) < model->threshold) {
                inliers_[num_inliers] = point;
                num_inliers++;
            }
        }

	    cv::InputArray inlers__ (inliers_);
	    inlers__.copyTo(inliers);
    }

    inline bool IsBetter(const Score * const s1, const Score * const s2) {
        return s1->score < s2->score;
    }


};


#endif //RANSAC_QUALITY_H