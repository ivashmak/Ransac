// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include <omp.h>
#include <thread>
#include "../estimator/estimator.hpp"
#include "../model.hpp"
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
    inline void getNumberInliers (Score * score, const cv::Mat& model, float threshold=0, bool get_inliers=false,
                                  int * inliers= nullptr, bool parallel=false) {
        if (threshold == 0) {
            threshold = this->threshold;
        }
        estimator->setModelParameters(model);

        unsigned int inlier_number = 0;

//        if (parallel && !get_inliers) {
//            #pragma omp parallel for reduction (+:inlier_number)
//            for (unsigned int point = 0; point < points_size; point++) {
//                if (estimator->GetError(point) < threshold) {
//                    inlier_number++;
//                }
//            }
//        } else {
            if (get_inliers) {
                for (unsigned int point = 0; point < points_size; point++) {
                    if (estimator->GetError(point) < threshold) {
                        inliers[inlier_number++] = point;
                    }    
                }
            } else {
                for (unsigned int point = 0; point < points_size; point++) {
                    if (estimator->GetError(point) < threshold) {
                        inlier_number++;
                    }
                }
            }
//       }

        score->inlier_number = inlier_number;
        score->score = inlier_number;
    }


	virtual void getScore (const float * const points, Score * score, const cv::Mat& model, int * inliers) {
        std::cout << "NOT IMPEMENTED getScore IN QUALITY\n";
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
	    for (unsigned int point = 0; point < points_size; point++) {
            if (estimator->GetError(point) < threshold) {
                inliers[num_inliers] = point;
                num_inliers++;
            }
        }
    }

    void getInliersFlags (const cv::Mat& model, bool * inlier_flags) {
        assert(isinit);

        estimator->setModelParameters(model);

        for (unsigned int point = 0; point < points_size; point++) {
            inlier_flags[point] = estimator->GetError(point) < threshold;
        }
    }


    static void getInliers (Estimator * estimator, const cv::Mat &model, float threshold, int points_size, std::vector<int> &inliers) {
        estimator->setModelParameters(model);
        inliers.clear();
        for (unsigned int p = 0; p < points_size; p++) {
            if (estimator->GetError(p) < threshold) {
                inliers.push_back(p);
            }
        }
    }



    /*
     * Calculate sum of errors to Ground Truth inliers.
     * And get number of gt inliers.
     */
    static float getErrorGT (Estimator * estimator,
                             Model * model,
                             int points_size,
                             const cv::Mat& gt_model,
                             int * num_gt_inliers) {

        // get inliers of gt model:
        estimator->setModelParameters(gt_model);
        int * inliers = new int [points_size];
        int inliers_size = 0;
        for (unsigned int point = 0; point < points_size; point++) {
            if (estimator->GetError(point) < model->threshold) {
                inliers[inliers_size++] = point;
            }
        }

        if (inliers_size == 0)
            return -1;

        *num_gt_inliers = inliers_size;

        // calculate sum of errors to inliers of gt model
        float sum_errors = 0;
        estimator->setModelParameters(model->returnDescriptor());
        for (unsigned int i = 0; i < inliers_size; i++) {
            sum_errors += estimator->GetError(inliers[i]);
        }
        return sum_errors / inliers_size;
    }

    static float getErrorGT_inl (Estimator * estimator,
                                Model * model,
                                const std::vector<int>& gt_inliers) {

        if (gt_inliers.size() == 0)
            return -1;

        // calculate sum of errors to inliers of gt model
        float sum_errors = 0;
        estimator->setModelParameters(model->returnDescriptor());
        for (unsigned int inl : gt_inliers) {
            sum_errors += estimator->GetError(inl);
        }
        return sum_errors / gt_inliers.size();
    }
};


#endif //RANSAC_QUALITY_H