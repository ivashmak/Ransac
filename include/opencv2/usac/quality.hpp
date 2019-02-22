// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include "estimator.hpp"
#include "model.hpp"
#include "cassert"

namespace cv {namespace usac {
/*
 * Score for model estimation
 */
class Score {
public:
    int inlier_number = 0;
    float score = 0;

    // priority for inlier number
    inline bool bigger(const Score *const score2) {
        if (inlier_number > score2->inlier_number) return true;
        if (inlier_number == score2->inlier_number) return score > score2->score;
        return false;
    }

    inline bool bigger(const Score &score2) {
        if (inlier_number > score2.inlier_number) return true;
        if (inlier_number == score2.inlier_number) return score > score2.score;
        return false;
    }

    void copyFrom(const Score *const score_to_copy) {
        score = score_to_copy->score;
        inlier_number = score_to_copy->inlier_number;
    }
};


class Quality {
protected:
    unsigned int points_size;
    float threshold;
    Estimator *estimator;
    bool isinit = false;
public:
    bool isInit() { return isinit; }

    void init(unsigned int points_size_, float theshold_, Estimator *estimator_) {
        points_size = points_size_;
        threshold = theshold_;
        estimator = estimator_;
        isinit = true;
    }

    /*
     * calculating number of inliers of current model.
     * score is sum of distances to estimated inliers.
     */
    inline void
    getNumberInliers(Score *score, const cv::Mat &model, float threshold = 0, bool get_inliers = false,
                     int *inliers = nullptr, bool parallel = false) {
        if (threshold == 0) {
            threshold = this->threshold;
        }

        estimator->setModelParameters(model);

        unsigned int inlier_number = 0;
        float sum_errors = 0;

        if (parallel && !get_inliers) {
#pragma omp parallel for reduction (+:inlier_number)
            for (unsigned int point = 0; point < points_size; point++) {
                if (estimator->getError(point) < threshold) {
                    inlier_number++;
                }
            }
        } else {
            float err;
            if (get_inliers) {
                for (unsigned int point = 0; point < points_size; point++) {
                    err = estimator->getError(point);
                    if (err < threshold) {
                        inliers[inlier_number++] = point;
                        sum_errors += err;
                    }
                }
            } else {
                for (unsigned int point = 0; point < points_size; point++) {
                    err = estimator->getError(point);
                    if (err < threshold) {
                        inlier_number++;
                        sum_errors += err;
                    }
                }
            }
        }

        score->inlier_number = inlier_number;
        score->score = sum_errors;
    }

    void getInliers(const cv::Mat &model, int *inliers) {
        // Note class Quality should be initialized
        assert(isinit);

        estimator->setModelParameters(model);

        int num_inliers = 0;
        for (unsigned int point = 0; point < points_size; point++) {
            if (estimator->getError(point) < threshold) {
                inliers[num_inliers] = point;
                num_inliers++;
            }
        }
    }

    static void
    getInliers(Estimator *estimator, const cv::Mat &model, float threshold, unsigned int points_size,
               std::vector<int> &inliers) {
        estimator->setModelParameters(model);
        inliers.clear();
        for (unsigned int p = 0; p < points_size; p++) {
            if (estimator->getError(p) < threshold) {
                inliers.push_back(p);
            }
        }
    }

    // Get average error to GT inliers.
    static float getErrorToGTInliers(Estimator *estimator, const cv::Mat &model, const std::vector<int> &gt_inliers) {

        // return -1 (unknown) to avoid division by zero
        if (gt_inliers.size() == 0)
            return -1;

        float sum_errors = 0;
        estimator->setModelParameters(model);
        for (unsigned int inl : gt_inliers) {
            sum_errors += estimator->getError(inl);
        }
        return sum_errors / gt_inliers.size();
    }
};
}}

#endif //RANSAC_QUALITY_H