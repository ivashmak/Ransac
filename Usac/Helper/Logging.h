#ifndef USAC_LOGGING_H
#define USAC_LOGGING_H

#include "../Model.h"
#include "../Quality.h"

#include <fstream>

class Logging {
public:
    void saveResult (Model * const model, Quality * const quality) {
        std::ofstream write_log;
        std::string filename = "../res/" + model->model_name +".txt";
        write_log.open (filename);
        write_log << quality->getComputationTime() <<"\n";
        write_log << quality->getIterations() <<"\n";
        write_log << quality->getNumberOfPointsUnderThreshold() <<"\n";
        write_log.close();
    }

    void compare (Model * const model, Quality * const quality) {
        std::ifstream read_log;
        std::string filename = "../res/" + model->model_name +".txt";
        read_log.open(filename);
        float time;
        int iters, points_under_treshold;
        read_log >> time;
        read_log >> iters;
        read_log >> points_under_treshold;

        std::cout << "speedup: " << time/quality->getComputationTime() << "\n";

        if (iters > quality->getIterations()) {
            std::cout << "iterations less on " << (iters - quality->getIterations()) << "\n";
        } else {
            std::cout << "iterations more on " << (-iters + quality->getIterations()) << "\n";
        }

        if (points_under_treshold > quality->getNumberOfPointsUnderThreshold()) {
            std::cout << "points under threshold less on " << (points_under_treshold - quality->getNumberOfPointsUnderThreshold()) << "\n";
        } else {
            std::cout << "points under threshold more on " << (-points_under_treshold + quality->getNumberOfPointsUnderThreshold()) << "\n";
        }


    }
};

#endif //USAC_LOGGING_H
