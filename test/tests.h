#ifndef TESTS_TESTS_H
#define TESTS_TESTS_H

#include "../helper/drawing/Drawing.h"
#include "../usac/estimator/estimator.hpp"
#include "../usac/quality/quality.hpp"
#include "../usac/ransac/ransac.hpp"
#include "statistical_results.h"
#include "../usac/sampler/prosac_sampler.hpp"
#include "../usac/termination_criteria/prosac_termination_criteria.hpp"
#include "../usac/utils/nearest_neighbors.hpp"
#include "../detector/detector.h"

class Tests {
public:
    void testLineFitting ();
    void testHomographyFitting ();
    void testFundamentalFitting ();
    void testEssentialFitting ();

    void testNeighborsSearchCell ();
    void testNeighborsSearch ();

    void test (cv::Mat points,
                      Model * model,
                      const std::string &img_name,
                      DATASET dataset,
                      bool gt,
                      const std::vector<int>& gt_inliers);


    void detectAndSaveFeatures (const std::vector<std::string>& dataset) {
//        std::string folder = "../dataset/homography/";
        std::string folder = "../dataset/Lebeda/kusvod2/";
//        std::string folder = "../dataset/adelaidermf/";

        for (const std::string &name : dataset) {
            std::cout << name << "\n";
            cv::Mat points;

            cv::Mat image1 = cv::imread (folder+name+"A.png");
            cv::Mat image2 = cv::imread (folder+name+"B.png");

            if (image1.empty()) {
                image1 = cv::imread (folder+name+"A.jpg");
                image2 = cv::imread (folder+name+"B.jpg");
            }

            DetectFeatures(folder+"sift_update/"+name+"_pts.txt", image1, image2, points);
        }
    }

    static std::string sampler2string (SAMPLER sampler) {
        if (sampler == SAMPLER::Prosac) return "prosac";
        if (sampler == SAMPLER::Uniform) return "uniform";
        if (sampler == SAMPLER::Napsac) return "napsac";
        if (sampler == SAMPLER::Evsac) return "evsac";
        return "";
    }

    static std::string estimator2string (ESTIMATOR estimator) {
        if (estimator == ESTIMATOR::Line2d) return "line2d";
        if (estimator == ESTIMATOR::Homography) return "homography";
        if (estimator == ESTIMATOR::Fundamental) return "fundamental";
        if (estimator == ESTIMATOR::Essential) return "essential";
        return "";
    }

    static std::string nearestNeighbors2string (NeighborsSearch nn) {
        if (nn == NeighborsSearch::Grid) return "grid";
        if (nn == NeighborsSearch::Nanoflann) return "nanoflann";
        return "";
    }

    static std::string getComputerInfo () {
        return "RAM 15.6 GB\n"
               "Intel Core i7\n"
               "OS type 64 bit\n"
               "8 CPUs\n";
    }
    /*
     * Display average results such as computational time,
     * number of inliers of N runs of Ransac.
     */
    void getStatisticalResults (const cv::Mat& points,
                                Model * const model,
                                int N,
                                bool GT,
                                const std::vector<int>& gt_inliers, bool get_results,
                                StatisticalResults * statistical_results) {

//        std::cout << "Testing " << estimator2string(model->estimator) << "\n";
//        std::cout << "with " << sampler2string(model->sampler) << " sampler\n";
//        std::cout << "with " << nearestNeighbors2string(model->neighborsType) << " neighbors searching\n";
//        std::cout << "with cell size = " << model->cell_size << "\n";
//        std::cout << "LO " << model->LO << "\n";
//        std::cout << "GC " << model->GraphCutLO << "\n";
//        std::cout << "SPRT " << model->Sprt << "\n";
//        std::cout << N << " times \n";


        long * times = new long[N];
        float * num_inlierss = new float[N];
        float * num_iterss = new float[N];
        float * num_lo_iterss = new float[N];
        float * errorss = new float[N];
        
        long time = 0;
        float num_inliers = 0;
        float errors = 0;
        float num_iters = 0;
        float num_lo_iters = 0;

        int fails_10 = 0;
        int fails_25 = 0;
        int fails_50 = 0;

        cv::Mat neighbors, neighbors_dists;
        std::vector<std::vector<int>> neighbors_v;

        // calculate time of nearest neighbor calculating
        auto begin_time = std::chrono::steady_clock::now();
        if (model->neighborsType == NeighborsSearch::Nanoflann) {
            NearestNeighbors::getNearestNeighbors_nanoflann(points, model->k_nearest_neighbors, neighbors, false, neighbors_dists);
        } else {
            NearestNeighbors::getGridNearestNeighbors(points, model->cell_size, neighbors_v);
        }
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = end_time - begin_time;
        long nn_time = std::chrono::duration_cast<std::chrono::microseconds>(fs).count();

        // Calculate Average of number of inliers, number of iteration,
        // time and average error.
        // If we have GT number of inliers, then find number of fails model.
        for (int i = 0; i < N; i++) {
            Ransac ransac (model, points);
            ransac.run();
            RansacOutput *ransacOutput = ransac.getRansacOutput();

            if (model->sampler == SAMPLER::Napsac || model->lo == LocOpt::GC) {
                times[i] = ransacOutput->getTimeMicroSeconds() + nn_time;
            } else {
                times[i] = ransacOutput->getTimeMicroSeconds();
            }

//            std::cout << "inl " << ransacOutput->getNumberOfInliers() << "\n";

            num_inlierss[i] = ransacOutput->getNumberOfInliers();
            num_iterss[i] = ransacOutput->getNumberOfMainIterations();
            num_lo_iterss[i] = ransacOutput->getLOIters();

            time += times[i];
            num_inliers += ransacOutput->getNumberOfInliers();
            num_iters += ransacOutput->getNumberOfMainIterations();
            num_lo_iters = ransacOutput->getLOIters();

            if (GT) {
                Estimator * estimator;
                initEstimator(estimator, model->estimator, points);
                float error = Quality::getErrorGT_inl(estimator, ransacOutput->getModel(), gt_inliers);

                errors += error;
                errorss[i] = error;

                /*
                 * If ratio of number of inliers and number of
                 * Ground Truth inliers is less than 50% then
                 * it is fail.
                 */
                std::vector<int> est_inliers = ransacOutput->getInliers();
                float matches = 0;
                for (int inl = 0; inl < gt_inliers.size(); inl++) {
                    for (int j = 0; j < num_inlierss[i]; j++) {
                        if (gt_inliers[inl] == est_inliers[j]) {
                            matches++;
                            break;
                        }
                    }
                }
                if (matches / gt_inliers.size() < 0.10) {
                    fails_10++;
                    fails_25++;
                    fails_50++;
                } else if (matches / gt_inliers.size() < 0.25) {
                    fails_25++;
                    fails_50++;
                } else if (matches / gt_inliers.size() < 0.50) {
                    fails_50++;
                }
//            std::cout << "----------------------------------------------------------------\n";
            }
        }
        
        StatisticalResults * results = new StatisticalResults;
        if (GT) {
            results->num_fails_10 = fails_10;
            results->num_fails_25 = fails_25;
            results->num_fails_50 = fails_50;
            results->avg_avg_error = errors/N;
        }

        results->avg_time_mcs = time/N;
        results->avg_num_inliers = num_inliers/N;
        results->avg_num_iters = num_iters/N;
        results->avg_num_lo_iters = num_lo_iters/N;


        long time_ = 0; float iters_ = 0, lo_iters_ = 0, inl_ = 0, err_ = 0;
        // Calculate sum ((xj - x)^2)
        for (int j = 0; j < N; j++) {
            time_ += pow (results->avg_time_mcs - times[j], 2);
            inl_ += pow (results->avg_num_inliers - num_inlierss[j], 2);
            err_ += pow (results->avg_avg_error - errorss[j], 2);
            iters_ += pow (results->avg_num_iters - num_iterss[j], 2);
            lo_iters_ += pow (results->avg_num_lo_iters - num_lo_iterss[j], 2);
        }

        // Calculate standart deviation
        int biased = 1;
        results->std_dev_time_mcs = sqrt (time_/(N-biased));
        results->std_dev_num_inliers = sqrt (inl_/(N-biased));
        results->std_dev_num_iters = sqrt (iters_/(N-biased));
        results->std_dev_num_lo_iters = sqrt (lo_iters_/(N-biased));

        if (GT) {
            results->std_dev_avg_error = sqrt (err_/(N-biased));
        }

        // Sort results for median
        std::sort (times, times + N, [] (long a, long b) { return a < b; });
        std::sort (num_inlierss, num_inlierss + N, [] (float a, float b) { return a < b; });
        std::sort (num_iterss, num_iterss + N, [] (float a, float b) { return a < b; });
        std::sort (num_lo_iterss, num_lo_iterss + N, [] (float a, float b) { return a < b; });
        std::sort (errorss, errorss + N, [] (float a, float b) { return a < b; });

        if (GT) {
            results->worst_case_error = errorss[N-1];
        }
        results->worst_case_num_inliers = num_inlierss[0];

        // Calcualte median of results for N is even
        results->median_time_mcs = (times[N/2-1] + times[N/2])/2;
        results->median_num_inliers = (num_inlierss[N/2-1] + num_inlierss[N/2])/2;
        results->median_num_iters = (num_iterss[N/2-1] + num_iterss[N/2])/2;
        results->median_num_lo_iters = (num_lo_iterss[N/2-1] + num_lo_iterss[N/2])/2;
        if (GT) {
            results->median_avg_error = (errorss[N/2-1] + errorss[N/2])/2;
        }

//        std::cout << results << "\n";

        if (get_results) {
            statistical_results->copyFrom(results);
        }

        delete errorss, num_inlierss, num_iterss, num_lo_iterss, times, results;
    }

    //todo add functions for storeResults () and showResults


//    void storeResults () {
//
//    }

};

#endif //TESTS_TESTS_H