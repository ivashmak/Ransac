#ifndef USAC_NAPSACSAMPLER_H
#define USAC_NAPSACSAMPLER_H


//#include <opencv2/flann/miniflann.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>
#include "Sampler.h"

class NapsacSampler : public Sampler {
public:
//    cv::InputArray input_points;
//    NapsacSampler (cv::InputArray points) {
//        input_points = points;
//    }
//
    void getSample (int *points, int npoints, int total_points) {
        cv::Mat queries, indicies, dists;
        int knn = 6;
//        cv::flann::LinearIndexParams flannIndexParams;

//        cv::flann::Index flannIndex (cv::Mat(input_points));
//        flannIndex.
//        flannIndex.knnSearch(queries, indicies, dists, knn);

    }
};


#endif //USAC_NAPSACSAMPLER_H
