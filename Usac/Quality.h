#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include <omp.h>
#include <thread>
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


    inline void GetModelScore(Estimator * const estimator,
                       Model * const model,
                       cv::InputArray input_points,
                       bool get_inliers,
                       Score &score,
                       std::vector<int> &inliers,
                       bool parallel=false) {

        int total_points = input_points.size().width;


        int score_inlier_number = 0;
        float dist;

        if (parallel) {
//            std::cout << "PARALLEL MODE\n";

            int num_threads = omp_get_max_threads();
            int block_size = total_points/num_threads;
            std::vector<std::vector<int>> inliers_per_thread (num_threads);

            #pragma omp parallel for reduction (+:score_inlier_number)
            for (int thread = 0; thread < num_threads; thread++) {
                for (int point = thread*block_size; point < (thread+1)*block_size; point++) {

                    dist = estimator->GetError(point);

                    if (dist < model->threshold) {
                        score_inlier_number++;
                        if (get_inliers) {
                            inliers_per_thread[thread].push_back(point);
                        }
                    }
                }
            }
            for (int thread = 0; thread < num_threads; thread++) {
                inliers.insert(inliers.end(), inliers_per_thread[thread].begin(), inliers_per_thread[thread].end());
            }

            // 2 parallel version, critical is quite expensive for big number of inliers
//            #pragma omp parallel for reduction (+:score_inlier_number)
//            for (int point = 0; point < total_points; point++) {
//                dist = estimator->GetError(point);
//
//                if (dist < model->threshold) {
//                    score_inlier_number++;
//
//                    if (get_inliers) {
//                        #pragma omp critical
//                        inliers.push_back(point);
//                    }
//                }
//            }

        } else {
            for (int point = 0; point < total_points; point++) {
                dist = estimator->GetError(point);

                if (dist < model->threshold) {
                    score_inlier_number++;
                    if (get_inliers) inliers.push_back(point);
                }
            }
        }

        score.inlier_number = score_inlier_number;
        score.score = score.inlier_number;
    }

    bool IsBetter(const Score * const s1, const Score * const s2) {
        return s1->score < s2->score;
    }

};


#endif //RANSAC_QUALITY_H