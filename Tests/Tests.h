#ifndef TESTS_TESTS_H
#define TESTS_TESTS_H

#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/Estimator.h"
#include "../Usac/Quality/Quality.h"
#include "../Usac/Ransac/Ransac.h"

class StatisticalResults {
public:
    // -1 is not estimated yet
    // std_dev is standart deviation
    // avg is average

    long avg_time_mcs = 0;
    long median_time_mcs = 0;
    long std_dev_time_mcs = 0;

    float avg_num_inliers = 0;
    float median_num_inliers = 0;
    float std_dev_num_inliers = 0;

    float avg_avg_error = -1;
    float median_avg_error = -1;
    float std_dev_avg_error = -1;

    float avg_num_iters = 0;
    float median_num_iters = 0;
    float std_dev_num_iters = 0;

    int num_fails = -1;

    friend std::ostream& operator<< (std::ostream& stream, const StatisticalResults * res) {
        return stream 
        << "Average time (mcs) " << res->avg_time_mcs << "\n"
        << "Standard deviation of time " << res->std_dev_time_mcs << "\n"
        << "Median of time " << res->median_time_mcs << "\n"
        << "-----------------\n"
        << "Average average error " << res->avg_avg_error << "\n"
        << "Standard deviation of average error " << res->std_dev_avg_error << "\n"
        << "Median of average error " << res->median_avg_error << "\n"
        << "-----------------\n"
        << "Average number of inliers " << res->avg_num_inliers << "\n"
        << "Standard deviation of number of inliers " << res->std_dev_num_inliers << "\n"
        << "Median of number of inliers " << res->median_num_inliers << "\n"
        << "-----------------\n"
        << "Average number of iterations " << res->avg_num_iters << "\n"
        << "Standard deviation of number of iterations " << res->std_dev_num_iters << "\n"
        << "Median of number of iterations " << res->median_num_iters << "\n"
        << "-----------------\n"
        << res->num_fails << " failed models\n";
    }
};


class Tests {
public:
    void testLineFitting ();
    void testHomographyFitting ();
    void testFundamentalFitting ();
    void testEssentialFitting ();

    /*
     * Display average results such as computational time,
     * number of inliers of N runs of Ransac.
     */
    void getAverageResults (cv::InputArray points,
                    Estimator *const estimator,
                    Model *const model,
                    Sampler *const sampler,
                    StandardTerminationCriteria * const termination_criteria,
                    Quality *const quality,
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
            Ransac ransac (model, sampler, termination_criteria, quality, estimator);
            ransac.run(points);
            RansacOutput *ransacOutput = ransac.getRansacOutput();

            times[i] = ransacOutput->getTimeMicroSeconds();
            num_inlierss[i] = ransacOutput->getNumberOfInliers();
            avg_errorss[i] = ransacOutput->getAverageError();
            num_iterss[i] = ransacOutput->getNumberOfIterations();

            time += ransacOutput->getTimeMicroSeconds();
            num_inliers += ransacOutput->getNumberOfInliers();
            avg_errors += ransacOutput->getAverageError();
            num_iters += ransacOutput->getNumberOfIterations();

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
        

        std::cout << N << " runs of Ransac for " << model->getName() << " with points size " << points.size().width << '\n';
        std::cout << results << "\n";

        if (get_results) {
            statistical_results = new StatisticalResults(*results);
        }

        delete avg_errorss, num_inlierss, num_iterss, times, results;
        //        std::cout << "bad models " << bad_models_counter << '\n';
    }

    //todo add functions for storeResults () and showResults


   void test (cv::InputArray points,
               Estimator * estimator,
               Sampler * sampler,
               Model * model,
               Quality * quality,
               StandardTerminationCriteria * termination_criteria,
               const std::string &img_name,
               int GT_num_inliers) {

        Drawing drawing;
        Logging logResult;

        Ransac ransac (model, sampler, termination_criteria, quality, estimator);
        ransac.run(points);

        RansacOutput * ransacOutput = ransac.getRansacOutput();

        std::cout << model->getName() << "\n";
        std::cout << "\ttime: ";
        ransacOutput->printTime();
        std::cout << "\titerations: " << ransacOutput->getNumberOfIterations() <<
                  " (" << ((int)ransacOutput->getNumberOfIterations () -(int)ransacOutput->getNumberOfLOIterations ()) << 
                  " + " << ransacOutput->getNumberOfLOIterations () << " (" << ransacOutput->getLORuns() << " lo inner + iterative runs)) \n";
        
        std::cout << "\tpoints under threshold: " << ransacOutput->getNumberOfInliers() << "\n";
        std::cout << "\tAverage error " << ransacOutput->getAverageError() << "\n";

        std::cout << "Best model = ...\n" << ransacOutput->getModel ()->returnDescriptor() << "\n";

        std::cout << "Ground Truth number of inliers for same model parametres is " << GT_num_inliers << "\n";
        
        // save result and compare with last run
        logResult.compare(model, ransacOutput);
        logResult.saveResult(model, ransacOutput);
        std::cout << "-----------------------------------------------------------------------------------------\n";
        

        if (model->estimator == ESTIMATOR::Homography) {
            std::string points_filename = "../dataset/homography/"+img_name+"_pts.txt";
            std::vector<std::string> images_filename;
            images_filename.push_back("../dataset/homography/"+img_name+"A.png");
            images_filename.push_back("../dataset/homography/"+img_name+"B.png");
        
            drawing.drawHomographies(img_name, ransacOutput->getInliers(), ransacOutput->getModel()->returnDescriptor());
        } else 
        if (model->estimator == ESTIMATOR::Fundamental) {
            cv::Mat pts = points.getMat();
            drawing.drawEpipolarLines(img_name, pts.colRange(0,2), pts.colRange(2,4), model->returnDescriptor());
        } else 
        if (model->estimator == ESTIMATOR::Line2d) {
            drawing.draw(ransacOutput->getInliers(), ransacOutput->getModel(), points);
        } else 
        if (model->estimator == ESTIMATOR::Essential) {

        } else {

        }
   
    }

//
//    void storeResults () {
//
//    }
//
//    void showResults (RansacOutput * ransacOutput) {
//
//    }
};

#endif //TESTS_TESTS_H