#ifndef TESTS_TESTS_H
#define TESTS_TESTS_H

#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/Estimator.h"
#include "../Usac/Quality/Quality.h"
#include "../Usac/Ransac/Ransac.h"
#include "StatisticalResults.h"
#include "../Usac/Sampler/ProsacSampler.h"
#include "../Usac/TerminationCriteria/ProsacTerminationCriteria.h"
#include "../Usac/Utils/NearestNeighbors.h"


class Tests {
public:
    void testLineFitting ();
    void testHomographyFitting ();
    void testFundamentalFitting ();
    void testEssentialFitting ();

    void initLine2D (Estimator *& estimator, const cv::Mat& points);
    void initHomography (Estimator *& estimator, const cv::Mat& points);
    void initFundamental (Estimator *& estimator, const cv::Mat& points);
    void initEssential (Estimator *& estimator, const cv::Mat& points);
    void initEstimator (Estimator *& estimator, Model * model, const cv::Mat& points);

    void initProsac (Sampler *& sampler, unsigned int sample_number, unsigned int points_size);
    void initUniform (Sampler *& sampler, unsigned int sample_number, unsigned int points_size);
    void initNapsac (Sampler *& sampler, const cv::Mat &neighbors, unsigned int k_nearest_neighbors,
                            unsigned int sample_number);
    void initEvsac (Sampler *& sampler, cv::InputArray points, unsigned int sample_number,
                           unsigned int points_size, unsigned int k_nearest_neighbors);
    void initGraduallyIncreasingSampler (Sampler *& sampler, cv::InputArray points, unsigned int sample_number);

    void initProsacNapsac1 (Sampler *& sampler, Model * model, const cv::Mat &nearest_neighors);
    void initProsacNapsac2 (Sampler *& sampler, Model * model, const cv::Mat &nearest_neighors);

    void initSampler (Sampler *& sampler, Model * model, unsigned int points_size, cv::InputArray points, const cv::Mat& neighbors);

    void test (cv::Mat points,
                      Model * model,
                      const std::string &img_name,
                      bool gt,
                      const std::vector<int>& gt_inliers);

    void getSortedPoints (const cv::Mat& neighbors_dists) {
    }

    std::string sampler2string (SAMPLER sampler) {
        if (sampler == SAMPLER::Prosac) return "prosac";
        if (sampler == SAMPLER::Uniform) return "uniform";
        if (sampler == SAMPLER::Napsac) return "napsac";
        if (sampler == SAMPLER::Evsac) return "evsac";
        return "";
    }

    std::string estimator2string (ESTIMATOR estimator) {
        if (estimator == ESTIMATOR::Line2d) return "line2d";
        if (estimator == ESTIMATOR::Homography) return "homography";
        if (estimator == ESTIMATOR::Fundamental) return "fundamental";
        if (estimator == ESTIMATOR::Essential) return "essential";
        return "";
    }


    std::string getComputerInfo () {
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
        float fails = 0;

        NearestNeighbors nn;
        cv::Mat neighbors, neighbors_dists;

        // calculate time of nearest neighbor calculating
        auto begin_time = std::chrono::steady_clock::now();
        nn.getNearestNeighbors_nanoflann(points, model->k_nearest_neighbors, neighbors, false, neighbors_dists);
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = end_time - begin_time;
        long nn_time = std::chrono::duration_cast<std::chrono::microseconds>(fs).count();

        // Calculate Average of number of inliers, number of iteration,
        // time and average error.
        // If we have GT number of inliers, then find number of fails model.
        for (int i = 0; i < N; i++) {

            // ------------------ init -------------------------------------
            Quality * quality = new Quality;
            Sampler * sampler;
            Estimator * estimator;
            TerminationCriteria * termination_criteria;

            initSampler(sampler, model, points.rows, points, neighbors);
            initEstimator(estimator, model, points);

            if (model->sampler == SAMPLER::Prosac) {
                // re init termination criteria for prosac
                ProsacTerminationCriteria *  prosac_termination_criteria_ = new ProsacTerminationCriteria;
                prosac_termination_criteria_->initProsacTerminationCriteria
                        (((ProsacSampler *) sampler)->getGrowthFunction(), model, points.rows, estimator);

                termination_criteria = prosac_termination_criteria_;
            } else {
                termination_criteria = new StandardTerminationCriteria;
            }
            // -------------- end of initialization -------------------


            Ransac ransac (model, sampler, termination_criteria, quality, estimator);
            ransac.setNeighbors(neighbors);
            ransac.run(points);
            RansacOutput *ransacOutput = ransac.getRansacOutput();

            if (model->sampler == SAMPLER::Napsac || model->GraphCutLO) {
                times[i] = ransacOutput->getTimeMicroSeconds() + nn_time;
            } else {
                times[i] = ransacOutput->getTimeMicroSeconds();
            }

            num_inlierss[i] = ransacOutput->getNumberOfInliers();
            num_iterss[i] = ransacOutput->getNumberOfMainIterations();
            num_lo_iterss[i] = ransacOutput->getLOIters();

            time += times[i];
            num_inliers += ransacOutput->getNumberOfInliers();
            num_iters += ransacOutput->getNumberOfMainIterations();
            num_lo_iters = ransacOutput->getLOIters();

            if (GT) {
                float error = Quality::getErrorGT_inl(estimator, ransacOutput->getModel(), points.rows, &gt_inliers[0],
                                                      gt_inliers.size());

                errors += error;
                errorss[i] = error;

                /*
                 * If ratio of number of inliers and number of
                 * Ground Truth inliers is less than 50% then
                 * it is fail.
                 */
                if (num_inlierss[i] / gt_inliers.size() < 0.5) {
                    fails++;
                    //                    std::cout << "FAIL\n";
                    //                    exit (111);
                }
            }
//            std::cout << "----------------------------------------------------------------\n";
        }
        
        StatisticalResults * results = new StatisticalResults;
        if (GT) {
            results->num_fails = fails;
            results->avg_error = errors/N;
        }

        results->avg_time_mcs = time/N;
        results->avg_num_inliers = num_inliers/N;
        results->avg_num_iters = num_iters/N;
        results->avg_num_lo_iters = num_lo_iters/N;

        long time_ = 0; float iters_ = 0, lo_iters_ = 0, inl_ = 0, err_ = 0;
        // Calculate sum ((xi - x)^2)
        for (int i = 0; i < N; i++) {
            time_ += pow (results->avg_time_mcs - times[i], 2);
            inl_ += pow (results->avg_num_inliers - num_inlierss[i], 2);
            err_ += pow (results->avg_error - errorss[i], 2);
            iters_ += pow (results->avg_num_iters - num_iterss[i], 2);
            lo_iters_ += pow (results->avg_num_lo_iters - num_lo_iterss[i], 2);
        }

        // Calculate standart deviation
        int biased = 1;
        results->std_dev_time_mcs = sqrt (time_/(N-biased));
        results->std_dev_num_inliers = sqrt (inl_/(N-biased));
        results->std_dev_num_iters = sqrt (iters_/(N-biased));
        results->std_dev_num_lo_iters = sqrt (lo_iters_/(N-biased));

        if (GT) {
            results->std_dev_error = sqrt (err_/(N-biased));
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
            results->median_error = (errorss[N/2-1] + errorss[N/2])/2;
        }

//        std::cout << N << " runs of Ransac for " << model->getName() << " with points size " << points.size().width << '\n';
        std::cout << results << "\n";

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