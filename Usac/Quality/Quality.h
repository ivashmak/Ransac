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

    // priority for inlier number
    inline bool bigger (const Score * const score2) {
        if (inlier_number > score2->inlier_number) return true;
        if (inlier_number == score2->inlier_number) return score > score2->score;
        return false;
    }
    inline bool bigger (const Score& score2) {
        if (inlier_number > score2.inlier_number) return true;
        if (inlier_number == score2.inlier_number) return score > score2.score;
        return false;
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
protected:
    unsigned int points_size;
    float threshold;
    Estimator * estimator;
    bool isinit = false;
public:
    bool isInit () { return isinit; }

    void init (unsigned int points_size_, float theshold_, Estimator * estimator_) {
        points_size = points_size_;
        threshold = theshold_;
        estimator = estimator_;
        isinit = true;
    }

    /*
     * calculating number of inliers under current model.
     * Here score = inlier number.
     * To get real score use getScore
     */
    inline void getNumberInliers (Score * score, Model * model, bool get_inliers=false,
                                  int * inliers= nullptr, bool parallel=false) {
        float threshold = model->threshold;
        estimator->setModelParameters(model->returnDescriptor());

        score->inlier_number = 0;

        if (parallel) {
            int score_inlier_number = 0;
//            std::cout << "PARALLEL MODE\n";

            #pragma omp parallel for reduction (+:score_inlier_number)
            for (int point = 0; point < points_size; point++) {
                if (estimator->GetError(point) < threshold) {
                    score_inlier_number++;
                }
            }

            score->inlier_number = score_inlier_number;

        } else {
            if (get_inliers) {
                for (int point = 0; point < points_size; point++) {
                    if (estimator->GetError(point) < threshold) {
                        inliers[score->inlier_number++] = point;
                    }    
                }
            } else {
                for (int point = 0; point < points_size; point++) {
                    if (estimator->GetError(point) < threshold) {
                        score->inlier_number++;
                    }
                }
            }
       }

        score->score = score->inlier_number;
	}


	virtual void getScore (const float * const points, Score * score, const cv::Mat& model, int * inliers) {
    }

    /*
     * We don't need to get inliers in getNumberInliers as we use them only once for estimation
     * non minimal model. As result faster way will be implement separate function for getting
     * inliers. Works same as getModelScore, however save inlier's indexes.
     */
    void getInliers (const cv::Mat& model, int * inliers) {
        assert(isinit);

        estimator->setModelParameters(model);

	    int num_inliers = 0;
	    for (int point = 0; point < points_size; point++) {
            if (estimator->GetError(point) < threshold) {
                inliers[num_inliers] = point;
                num_inliers++;
            }
        }
    }

    /*
     * Calculate average error for any inliers (e.g. GT inliers or output inliers).
     */
    float getAverageError (const cv::Mat& model,
                           const int * const inliers,
                           int inliers_size) {
        assert(isinit);

        estimator->setModelParameters(model);
        float sum_errors = 0;

        for (int point = 0; point < inliers_size; point++) {
            sum_errors += estimator->GetError(inliers[point]);
        }
        return sum_errors/inliers_size;
    }

};


#endif //RANSAC_QUALITY_H