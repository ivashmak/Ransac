#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include "Estimator/Estimator.h"
#include "Model.h"

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

    void GetModelScore(Estimator * const estimator,
                       Model * const model,
                       cv::InputArray input_points,
                       bool get_inliers,
                       Score &score,
                       std::vector<int> &inliers) {

        int total_points = input_points.size().width;

	    // useful for big data
        //  #pragma omp parallel for reduction (+:score.inlier_number)
        for (int point = 0; point < total_points; point++) {
            float dist = estimator->GetError(input_points, point, model);

            if (dist < model->threshold) {
                score.inlier_number++;
                if (get_inliers) inliers.push_back(point);
            }
        }
        score.score = score.inlier_number;
    }

    bool IsBetter(const Score * const s1, const Score * const s2) {
        return s1->score < s2->score;
    }
};


#endif //RANSAC_QUALITY_H