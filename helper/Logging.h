#ifndef USAC_LOGGING_H
#define USAC_LOGGING_H

#include "../usac/model.hpp"
#include "../usac/quality/quality.hpp"
#include "../test/statistical_results.h"
#include "../test/tests.h"

#include <fstream>

class Logging {
public:

    /*
     * Save results to file
     */
    static void saveResult (Model * const model, RansacOutput * const ransacOutput) {
        std::ofstream write_log;
        std::string filename = "../results/" + Tests::sampler2string(model->sampler)+"_"+
                Tests::estimator2string(model->estimator)+".txt";
        write_log.open (filename);
        write_log << ransacOutput->getTimeMicroSeconds() <<"\n";
        write_log << (ransacOutput->getNumberOfMainIterations() + ransacOutput->getLOIters()) <<"\n";
        write_log << ransacOutput->getNumberOfInliers() <<"\n";
        write_log.close();
    }

    /*
     * Read results from saved file and compare with current results.
     */
    static void compare (Model * const model, RansacOutput * const ransacOutput) {
        std::ifstream read_log;
        std::string filename = "../results/" + Tests::sampler2string(model->sampler)+"_"+
                                               Tests::estimator2string(model->estimator) +".txt";
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

    static void saveHeadOfCSV (std::ofstream &file, Model * model, int N_runs) {
        file << Tests::getComputerInfo();
        file << Tests::sampler2string(model->sampler)+"_"+
                Tests::estimator2string(model->estimator) << "\n";
        file << "Runs for each image = " << N_runs << "\n";
        file << "Threshold for each image = " << model->threshold << "\n";
        file << "Desired probability for each image = " << model->desired_prob << "\n";
        file << "Inner Iterative LO = " << (model->lo == LocOpt ::InItLORsc) << "\n";
        file << "Inner Iterative Fxing LO = " << (model->lo == LocOpt ::InItFLORsc) << "\n";
        file << "Graph Cut LO = " << (model->lo == LocOpt ::GC) << "\n";
        file << "SPRT = " << (bool) model->sprt << "\n\n\n";

        file << "Filename,GT Inl,Avg num inl/gt,Std dev num inl,Med num inl,"
                         "Avg num iters,Std dev num iters,Med num iters,"
                         "Avg num LO iters,Std dev num LO iters,Med num LO iters,"
                         "Avg time (mcs),Std dev time,Med time,"
                         "Avg err,Std dev err,Med err,"
                         "Worst case num Inl,Worst case Err,"
                         "Num fails (<10%),Num fails (<25%),Num fails (<50%)\n";

    }

    static void saveResultsCSV (std::ofstream &file, const StatisticalResults * const statistical_results) {
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

    static void saveResultsMatlab (std::ofstream &file, const StatisticalResults * const statistical_results) {
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
