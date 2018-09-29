#ifndef USAC_LOGGING_H
#define USAC_LOGGING_H

#include "../Model.h"
#include "../Quality.h"

#include <fstream>

class Logging {
public:

    /*
     * Save results to file
     */
    void saveResult (Model * const model, Quality * const quality) {
        std::ofstream write_log;
        std::string filename = "../results/" + model->model_name +".txt";
        write_log.open (filename);
        write_log << quality->getComputationTime() <<"\n";
        write_log << quality->getIterations() <<"\n";
        write_log << quality->getNumberOfPointsUnderThreshold() <<"\n";
        write_log.close();
    }

    /*
     * Read results from saved file and compare with current results.
     */
    void compare (Model * const model, Quality * const quality) {
        std::ifstream read_log;
        std::string filename = "../results/" + model->model_name +".txt";
        read_log.open(filename);
        float time;
        int iters, points_under_treshold;
        read_log >> time;
        read_log >> iters;
        read_log >> points_under_treshold;

        std::cout << "speedup: " << time/quality->getComputationTime() << "\n";
        std::cout << "iterations more on " << (quality->getIterations() - iters) << "\n";
        std::cout << "points under threshold more on " << (quality->getNumberOfPointsUnderThreshold() - points_under_treshold) << "\n";
    }
};

#endif //USAC_LOGGING_H
