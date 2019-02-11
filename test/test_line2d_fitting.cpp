#include <cstdlib>
#include "tests.h"

#include "../generator/generator.h"
#include "../usac/estimator/line2d_estimator.hpp"
#include "../usac/ransac/ransac.hpp"

#include "../helper/Logging.h"

#include "../detector/Reader.h"

#include "../usac/sampler/sampler.hpp"
#include "../usac/sampler/napsac_sampler.hpp"
#include "../usac/sampler/prosac_sampler.hpp"
#include "../usac/sampler/evsac_sampler.hpp"
#include "../usac/sampler/uniform_sampler.hpp"
#include "../usac/sampler/prosac_simple_sampler.hpp"
#include "../usac/utils/nearest_neighbors.hpp"
#include "../usac/sampler/prosac_sampler.hpp"
#include "../usac/utils/utils.hpp"

void store_results_line2d ();

void Tests::testLineFitting() {
    DATASET dataset = DATASET ::Syntectic;

    // old ------------------------------
    // change false to true to reset time for random points generator
    // get number of ground truth inliers too.
//    std::string img_name = "../dataset/image1";
//    std::vector<cv::Point_<float>> points;
//    generate(points, false, true, gt_model);
    // -------------------------------------

    // another generated data
    std::string img_name = "w=1200_h=1200_n=3.000000_I=200_N=10200";
    ImageData img_data (dataset, img_name);
    cv::Mat gt_model = img_data.getModel();
    cv::Mat pts = img_data.getPoints();
    cv::Mat sorted_pts = img_data.getSortedPoints();
    std::vector<int> gt_inliers = img_data.getGTInliers(5 /* threshold */);
    std::vector<int> gt_sorted_inliers = img_data.getGTInliersSorted(5 /* threshold */);

//    std::cout << "generated points\n";

    unsigned int knn = 13;
    Model * model;
    float threshold = 8, confidence = 0.99;

    // ---------------- uniform -------------------
//     model = new Model (threshold, 2, confidence, knn, ESTIMATOR::Line2d, SAMPLER::Uniform);
//------------------------------------------

    // --------------  prosac ---------------------
    model = new Model (threshold, 2, confidence, knn, ESTIMATOR::Line2d, SAMPLER::Prosac);
     // ------------------------------------------------

    // ---------------- napsac -------------------------------
    // model = new Model (threshold, 2, confidence, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
    // ---------------------------------------------------------------------

    // ----------------- evsac ------------------------------
//    model = new Model (threshold, 2, confidence, knn, ESTIMATOR::Line2d, SAMPLER::Evsac);
// ------------------------------------------------------------

    // ------------------ Progressive Napsac ----------------------
//    model = new Model (threshold, 2, confidence, knn, ESTIMATOR::Line2d, SAMPLER::ProgressiveNAPSAC);
// --------------------------------------------------------

     model->lo = LocOpt ::NullLO;
     model->setSprt(0);
     model->setNeighborsType(NeighborsSearch::Nanoflann);

//    test (pts, model, img_name, dataset, true, gt_inliers);
    test (sorted_pts, model, img_name, dataset, true, gt_sorted_inliers);

//     getStatisticalResults(pts, model, 1000, true, gt_inliers, false, nullptr);
//     getStatisticalResults(sorted_pts, model, 1000, true, gt_sorted_inliers, false, nullptr);

//   store_results_line2d();
}

void store_results_line2d () {
    DATASET dataset = DATASET ::Syntectic;
    std::vector<std::string> points_filename = Dataset::getDataset(dataset);

    std::vector<cv::Mat_<float>> points;
    std::vector<cv::Mat_<float>> sorted_points;
    std::vector<std::vector<int>> gt_inliers;
    std::vector<std::vector<int>> gt_sorted_inliers;

    float confidence = 0.95;
    float threshold = 8;
    int N_runs = 50;
    int knn = 8;

    for (const std::string &img_name : points_filename) {
        ImageData img_data (dataset, img_name);
        cv::Mat sorted_pts = img_data.getSortedPoints();
        cv::Mat pts = img_data.getPoints();

        // save points and gt
        gt_inliers.push_back(img_data.getGTInliers(threshold));
        gt_sorted_inliers.push_back(img_data.getGTInliers(threshold));
        points.push_back(pts);
        sorted_points.push_back(sorted_pts);
    }
    std::cout << "Got data files\n";

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
    samplers.push_back(SAMPLER::Prosac);
    samplers.push_back(SAMPLER::Napsac);

    std::vector<LocOpt > loc_opts;
    loc_opts.push_back(LocOpt::InItLORsc);
    bool sprt = 0;

    for (SAMPLER smplr : samplers) {
        for (auto loc_opt : loc_opts) {
            std::ofstream results_total;
            std::ofstream results_matlab;
            std::string name = "../results/line2d/" + Tests::sampler2string(smplr);
            if (loc_opt == LocOpt::InItLORsc) name += "_lo";
            if (loc_opt == LocOpt::InItFLORsc) name += "_flo";
            if (loc_opt == LocOpt::GC) name += "_gc";
            if (sprt) name += "_sprt";

            std::string mfname = name+"_m.csv";
            std::string fname = name+".csv";

            results_matlab.open(mfname);
            results_total.open(fname);

            Model * model = new Model(threshold, 2, confidence, knn, ESTIMATOR::Line2d, smplr);
            model->lo = loc_opt;
            model->setSprt(sprt);

            results_total << Tests::getComputerInfo();
            results_total << Tests::sampler2string(model->sampler)+"_"+Tests::estimator2string(model->estimator) << "\n";
            results_total << "Runs for each image = " << N_runs << "\n";
            results_total << "Threshold for each image = " << model->threshold << "\n";
            results_total << "Desired probability for each image = " << model->desired_prob << "\n";
            results_total << "LO = " << (bool) model->lo << "\n";
            results_total << "SPRT = " << (bool) model->sprt << "\n\n\n";

            results_total << "Filename,GT Inl,Avg num inl/gt,Std dev num inl,Med num inl,"
                             "Avg num iters,Std dev num iters,Med num iters,"
                             "Avg num LO iters,Std dev num LO iters,Med num LO iters,"
                             "Avg time (mcs),Std dev time,Med time,"
                             "Avg err,Std dev err,Med err,"
                             "Worst case num Inl,Worst case Err,"
                             "Num fails (<10%),Num fails (<25%),Num fails (<50%)\n";

            std::cout << Tests::sampler2string(smplr) << "\n";
            std::cout << "LO  " << loc_opt << "\n";
            
            for (int img = 0; img < points_filename.size(); img++) {
                std::cout << points_filename[img] << "\n";

                StatisticalResults *statistical_results = new StatisticalResults;
                if (smplr == SAMPLER::Prosac) {
                    Tests::getStatisticalResults(sorted_points[img], model, N_runs,
                                                true, gt_inliers[img], true, statistical_results);
                } else {
                    Tests::getStatisticalResults(points[img], model, N_runs,
                                                true, gt_inliers[img], true, statistical_results);
                }

                // save to csv file
                // save to csv file
                results_total << points_filename[img] << ",";
                results_total << gt_inliers[img].size() << ",";
                Logging::saveResultsCSV(results_total, statistical_results);

                // save results for matlab
                results_matlab << points_filename[img] << ",";
                Logging::saveResultsMatlab(results_matlab, statistical_results);

                img++;
            }

            results_total.close();
            results_matlab.close();
        }
    }
}