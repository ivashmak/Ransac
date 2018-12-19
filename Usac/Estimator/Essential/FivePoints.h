#ifndef USAC_FIVEPOINTSALGORITHM_H
#define USAC_FIVEPOINTSALGORITHM_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>

int FivePointsAlgorithm (const double * const pts, const int * const sample, cv::OutputArray E);



#endif //USAC_FIVEPOINTSALGORITHM_H