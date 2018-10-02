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
    void saveResult (const Model * const model, RansacOutput * const ransacOutput) {
        std::ofstream write_log;
        std::string filename = "../results/" + model->model_name +".txt";
        write_log.open (filename);
        write_log << ransacOutput->getTimeMicroSeconds() <<"\n";
        write_log << ransacOutput->getNumberOfIterations() <<"\n";
        write_log << ransacOutput->getNumberOfInliers() <<"\n";
        write_log.close();
    }

    /*
     * Read results from saved file and compare with current results.
     */
    void compare (const Model * const model, RansacOutput * const ransacOutput) {
        std::ifstream read_log;
        std::string filename = "../results/" + model->model_name +".txt";
        read_log.open(filename);
        float time;
        int iters, points_under_treshold;
        read_log >> time;
        read_log >> iters;
        read_log >> points_under_treshold;

        std::cout << "speedup: " << time/ransacOutput->getTimeMicroSeconds() << "\n";
        std::cout << "iterations more on " << ((int)ransacOutput->getNumberOfIterations() - iters) << "\n";
        std::cout << "points under threshold more on " << ((int)ransacOutput->getNumberOfInliers() - points_under_treshold) << "\n";
    }
};

#endif //USAC_LOGGING_H
