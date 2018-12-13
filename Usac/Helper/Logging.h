#ifndef USAC_LOGGING_H
#define USAC_LOGGING_H

#include "../Model.h"
#include "../Quality/Quality.h"
#include "../../Tests/StatisticalResults.h"

#include <fstream>

class Logging {
public:

    /*
     * Save results to file
     */
    void saveResult (Model * const model, RansacOutput * const ransacOutput) {
        std::ofstream write_log;
        std::string filename = "../results/" + model->getName() +".txt";
        write_log.open (filename);
        write_log << ransacOutput->getTimeMicroSeconds() <<"\n";
        write_log << (ransacOutput->getNumberOfMainIterations() + ransacOutput->getLOIters()) <<"\n";
        write_log << ransacOutput->getNumberOfInliers() <<"\n";
        write_log.close();
    }

    /*
     * Read results from saved file and compare with current results.
     */
    void compare (Model * const model, RansacOutput * const ransacOutput) {
        std::ifstream read_log;
        std::string filename = "../results/" + model->getName() +".txt";
        read_log.open(filename);
        float time;
        int iters, points_under_treshold;
        read_log >> time;
        read_log >> iters;
        read_log >> points_under_treshold;

        std::cout << "speedup: " << time/ransacOutput->getTimeMicroSeconds() << "\n";
        std::cout << "iterations more on " << ((int)(ransacOutput->getNumberOfMainIterations() + ransacOutput->getLOIters()) - iters) << "\n";
        std::cout << "points under threshold more on " << ((int)ransacOutput->getNumberOfInliers() - points_under_treshold) << "\n";
    }

    void saveResultsCSV (std::ofstream &file, const StatisticalResults * const statistical_results) {
        file << statistical_results->avg_num_inliers << ",";
        file << statistical_results->std_dev_num_inliers << ",";
        file << statistical_results->median_num_inliers << ",";

        file << statistical_results->avg_num_iters << ",";
        file << statistical_results->std_dev_num_iters << ",";
        file << statistical_results->median_num_iters << ",";

        file << statistical_results->avg_num_lo_iters << ",";
        file << statistical_results->std_dev_num_lo_iters << ",";
        file << statistical_results->median_num_lo_iters << ",";

        file << statistical_results->avg_time_mcs << ",";
        file << statistical_results->std_dev_time_mcs << ",";
        file << statistical_results->median_time_mcs << ",";

        file << statistical_results->avg_avg_error << ",";
        file << statistical_results->std_dev_avg_error << ",";
        file << statistical_results->median_avg_error << ",";

        file << statistical_results->worst_case_num_inliers << ",";
        file << statistical_results->worst_case_error << ",";

        file << statistical_results->num_fails_10 << ",";
        file << statistical_results->num_fails_25 << ",";
        file << statistical_results->num_fails_50 << "\n";
    }

    void saveResultsMatlab (std::ofstream &file, const StatisticalResults * const statistical_results) {
        // save results for matlab
        file << statistical_results->avg_num_inliers << ",";
        file << statistical_results->avg_num_iters << ",";
        file << statistical_results->avg_num_lo_iters << ",";
        file << statistical_results->avg_time_mcs << ",";
        file << statistical_results->avg_avg_error << ",";
        file << statistical_results->num_fails_10 << ",";
        file << statistical_results->num_fails_25 << ",";
        file << statistical_results->num_fails_50 << "\n";
    }


};

#endif //USAC_LOGGING_H
