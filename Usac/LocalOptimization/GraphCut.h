#ifndef USAC_GRAPHCUT_H
#define USAC_GRAPHCUT_H

#include <opencv2/core/mat.hpp>
#include "../Estimator/Estimator.h"
#include "../../include/gco-v3.0/GCoptimization.h"
#include "LocalOptimization.h"

class GraphCut : public LocalOptimization {
public:
    void labeling(const int *const neighbors, Estimator *estimator, Model *model, int *inliers, int points_size);
};

#endif //USAC_GRAPHCUT_H
