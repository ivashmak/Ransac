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


class Tests {
public:
    void testLineFitting ();
    void testHomographyFitting ();
    void testFundamentalFitting ();
    void testEssentialFitting ();

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

    void test (cv::InputArray points,
               Estimator * estimator,
               Sampler * sampler,
               Model * model,
               Quality * quality,
               TerminationCriteria * termination_criteria,
               const cv::Mat& neighbors,
               const std::string &img_name,
               int GT_num_inliers);


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
                    Estimator *const estimator,
                    Model *const model,
                    Sampler * sampler,
                    TerminationCriteria * termination_criteria,
                    Quality *const quality,
                    const cv::Mat& neighbors,
                    int N,
                    bool GT = false, bool get_results = false,
                    int gt_num_inliers=0, StatisticalResults * statistical_results=0) {



        int bad_models_counter = 0;

        long * times = new long[N];
        float * num_inlierss = new float[N];
        float * num_iterss = new float[N];
        float * avg_errorss = new float[N];
        
        long time = 0;
        float num_inliers = 0;
        float avg_errors = 0;
        int num_iters = 0;
        int fails = 0;

        // Calculate Average of number of inliers, number of iteration,
        // time and average error.
        // If we have GT number of inliers, then find number of fails model.
        for (int i = 0; i < N; i++) {
            // re init sampler
            initSampler(sampler, model, points.rows, points, neighbors);

            if (model->sampler == SAMPLER::Prosac) {
                // re init termination criteria for prosac
                ProsacTerminationCriteria *  prosac_termination_criteria_ = new ProsacTerminationCriteria;
                prosac_termination_criteria_->initProsacTerminationCriteria
                        (((ProsacSampler *) sampler)->getGrowthFunction(), model, points.rows);

                termination_criteria = prosac_termination_criteria_;
            }



            Ransac ransac (model, sampler, termination_criteria, quality, estimator);
            ransac.setNeighbors(neighbors);
            ransac.run(points);
            RansacOutput *ransacOutput = ransac.getRansacOutput();

            times[i] = ransacOutput->getTimeMicroSeconds();
            num_inlierss[i] = ransacOutput->getNumberOfInliers();
            avg_errorss[i] = ransacOutput->getAverageError();
            num_iterss[i] = ransacOutput->getTotalIters();

            time += ransacOutput->getTimeMicroSeconds();
            num_inliers += ransacOutput->getNumberOfInliers();
            avg_errors += ransacOutput->getAverageError();
            num_iters += ransacOutput->getTotalIters();

            if (GT) {
                /* 
                 * If ratio of number of inliers and number of
                 * Ground Truth inliers is less than 50% then
                 * it is fail.
                 */  
                if (num_inlierss[i]/gt_num_inliers < 0.5) {
                    fails++;
                }
            }
        }
        
        StatisticalResults * results = new StatisticalResults;
        if (GT) {
            results->num_fails = fails;
        }

        results->avg_time_mcs = time/N;
        results->avg_num_inliers = (float) num_inliers/N;
        results->avg_avg_error = avg_errors/N;
        results->avg_num_iters = (float) num_iters/N;

        long time_ = 0; float iters_ = 0, inl_ = 0, err_ = 0;
        // Calculate sum ((xi - x)^2)
        for (int i = 0; i < N; i++) {
            time_ += pow (results->avg_time_mcs - times[i], 2);
            inl_ += pow (results->avg_num_inliers - num_inlierss[i], 2);
            err_ += pow (results->avg_avg_error - avg_errorss[i], 2);
            iters_ += pow (results->avg_num_iters - num_iterss[i], 2);
        }

        // Calculate standart deviation
        int biased = 1;
        results->std_dev_time_mcs = sqrt (time_/(N-biased));
        results->std_dev_num_inliers = sqrt ((float) inl_/(N-biased));
        results->std_dev_avg_error = sqrt (err_/(N-biased));
        results->std_dev_num_iters = sqrt ((float) iters_/(N-biased));

        // Sort results for median
        std::sort (times, times + N, [] (long a, long b) { return a < b; });
        std::sort (num_inlierss, num_inlierss + N, [] (float a, float b) { return a < b; });
        std::sort (num_iterss, num_iterss + N, [] (float a, float b) { return a < b; });
        std::sort (avg_errorss, avg_errorss + N, [] (float a, float b) { return a < b; });
        
        // Calcualte median of results for N is even
        results->median_time_mcs = (times[N/2-1] + times[N/2])/2;
        results->median_num_inliers = (num_inlierss[N/2-1] + num_inlierss[N/2])/2;
        results->median_avg_error = (avg_errorss[N/2-1] + avg_errorss[N/2])/2;
        results->median_num_iters = (num_iterss[N/2-1] + num_iterss[N/2])/2;
        

//        std::cout << N << " runs of Ransac for " << model->getName() << " with points size " << points.size().width << '\n';
        std::cout << results << "\n";

        if (get_results) {
            statistical_results->copyFrom(results);
        }

        delete avg_errorss, num_inlierss, num_iterss, times, results;
        //        std::cout << "bad models " << bad_models_counter << '\n';
    }

    //todo add functions for storeResults () and showResults


//    void storeResults () {
//
//    }

};

#endif //TESTS_TESTS_H