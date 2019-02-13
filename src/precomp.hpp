// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef PRECOMP_H
#define PRECOMP_H

// C++
#include <iostream>
#include <cstdio>
#include <string>
#include <cmath>
#include <chrono>
#include <omp.h>
#include <thread>
#include <vector>
#include <cassert>
#include <memory>
#include <cstdlib>
#include <random>
#include <algorithm>
#include <iomanip>

// Eigen (is need only for nanoflann neighbors searching. We should try to get rid of it).
#include <Eigen/Dense>

// Nanoflann (only for nearest neighbors searching, maybe we should try to move nanoflann 
// source code to include folder and don't inlcude whole library)
#include <nanoflann.hpp>

// OpenCV
#include <opencv2/flann/flann.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

//#include "model.hpp"
//#include "sampler/sampler.hpp"
//#include "estimator/estimator.hpp"
//#include "local_optimization/local_optimization.hpp"
//#include "random_generator/random_generator.hpp"

#endif // PRECOMP_H
