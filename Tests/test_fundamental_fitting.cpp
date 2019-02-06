#include "tests.h"

#include "../Detector/Reader.h"
#include "../helper/drawing/Drawing.h"
#include "../helper/Logging.h"
#include "../usac/estimator/fundamental_estimator.hpp"
#include "../usac/sampler/uniform_sampler.hpp"
#include "../dataset/Dataset.h"
#include "../usac/utils/nearest_neighbors.hpp"
#include "../usac/utils/utils.hpp"

void storeResultsFundamental ();
void getGTInliersFromGTModelFundamental (const std::string& filename, const cv::Mat& points, float threshold, std::vector<int> &gt_inliers);

void Tests::testFundamentalFitting() {
//    detectAndSaveFeatures(getAdelaidermfDataset());
//    detectAndSaveFeatures(getKusvod2Dataset());
//    exit (0);

    DATASET dataset = DATASET::Kusvod2_SIFT;
    std::string img_name = "box";
    cv::Mat_<float> sorted_points, points;

    ImageData gt_data (dataset, img_name);
    points = gt_data.getPoints();
    sorted_points = gt_data.getSortedPoints();

//    densitySort (points, 4, sorted_points);

    std::cout << "points size = " << points.rows << "\n";

    float threshold = 2;
    float confidence = 0.95;
    int knn = 8;

    std::vector<int> gt_inliers = gt_data.getGTInliers(threshold);
    std::vector<int> gt_sorted_inliers = gt_data.getGTInliersSorted(threshold);

//    for (int i = 0; i < gt_inliers.size(); i++) {
//        std::cout << gt_inliers[i] << " " << gt_sorted_inliers[i] << "\n";
//    }

    Model * model;

    // -------------------------- uniform -------------------------------------
//    model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Uniform);
    // ------------------------------------------------------------------------

    // -------------------------- Prosac -------------------------------------
    model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Prosac);
    // ------------------------------------------------------------------------

    model->lo = LocOpt ::NullLO;
    model->setSprt(0);
    model->setCellSize(50);
    model->setNeighborsType(NeighborsSearch::Nanoflann);
    model->ResetRandomGenerator(true);

//    test (points, model, img_name, dataset, true, gt_inliers);
    test (sorted_points, model, img_name, dataset, true, gt_sorted_inliers);

//    getStatisticalResults(points, model, 500, true, gt_inliers, false, nullptr);
//    getStatisticalResults(sorted_points, model, 500, true, gt_sorted_inliers, false, nullptr);

//     storeResultsFundamental ();
}
 
/*
 * Store results from dataset to csv file.
 */

void storeResultsFundamental () {
    DATASET dataset = DATASET ::Adelaidermf_SIFT; // Homogr, Kusvod2, Adelaidrmf, EVD
    std::vector<std::string> points_filename = Dataset::getDataset(dataset);
    int num_images = points_filename.size();
    std::cout << "number of images " << num_images << "\n";

    Tests tests;
    Logging log;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<cv::Mat_<float>> sorted_points_imgs;
    std::vector<std::vector<int>> gt_inliers;
    std::vector<std::vector<int>> gt_inliers_sorted;

    int N_runs = 100;
    int knn = 8;
    float threshold = 2;
    float confidence = 0.95;

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";

        cv::Mat_<float> points, sorted_points;
        ImageData gt_data (dataset, img_name);
        points = gt_data.getPoints();
        sorted_points = gt_data.getSortedPoints();

        gt_inliers.push_back(gt_data.getGTInliers(threshold));
        gt_inliers_sorted.push_back(gt_data.getGTInliersSorted(threshold));
        points_imgs.push_back(points);
        sorted_points_imgs.push_back(sorted_points);

        std::cout << "inliers size " << gt_inliers[gt_inliers.size()-1].size() << "\n";
        std::cout << "sorted inliers size " << gt_inliers_sorted[gt_inliers.size()-1].size() << "\n";
    }

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
//    samplers.push_back(SAMPLER::Prosac);

    std::vector<LocOpt > loc_opts;
    loc_opts.push_back(LocOpt::GC);

    NeighborsSearch neighborsSearch = NeighborsSearch ::Grid;
    int cell_size = 50;

    long mean_time = 0;
    long mean_error = 0;
    bool sprt = 0;

    for (SAMPLER smplr : samplers) {
        for (auto loc_opt : loc_opts) {
            std::ofstream results_total;
            std::ofstream results_matlab;
            std::string name = "../results/EVD/";
            name += Tests::sampler2string(smplr);
            if (loc_opt == LocOpt::InItRsc) name += "_lo";
            if (loc_opt == LocOpt::GC) name += "_gc";
            if (sprt) name += "_sprt";

            name += "_"+Tests::nearestNeighbors2string(neighborsSearch) + "_c_sz_"+std::to_string(cell_size);

            std::string mfname = name+"_m.csv";
            std::string fname = name+".csv";

            Model *model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, smplr);
            model->lo = loc_opt;
            model->setSprt(sprt);
            model->setNeighborsType(neighborsSearch);
            model->setCellSize(cell_size);

            results_matlab.open (mfname);
            results_total.open (fname);

            log.saveHeadOfCSV (results_total, model, N_runs);

            int img = 0;
            for (const std::string &img_name : points_filename) {

//                std::cout << img_name << "\n";

                StatisticalResults * statistical_results = new StatisticalResults;
                if (smplr == SAMPLER::Prosac) {
                    tests.getStatisticalResults(sorted_points_imgs[img], model, N_runs,
                                                true, gt_inliers_sorted[img], true, statistical_results);
                } else {
                    tests.getStatisticalResults(points_imgs[img], model, N_runs,
                                                true, gt_inliers[img], true, statistical_results);
                }

//                // save to csv file
//                results_total << img_name << ",";
//                results_total << gt_inliers[img].size() << ",";
//                log.saveResultsCSV(results_total, statistical_results);
//
//                // save results for matlab
//                results_matlab << img_name << ",";
//                log.saveResultsMatlab(results_matlab, statistical_results);
//                std::cout << statistical_results->avg_time_mcs;
//                std::cout << " +- " << statistical_results->std_dev_time_mcs << "\n";
//                std::cout << statistical_results->avg_avg_error;
//                std::cout << " +- " << statistical_results->std_dev_avg_error << "\n";
//                std::cout << "- - - - - - - - - - - - - - - - - -\n";

                std::cout << statistical_results->avg_num_lo_iters << " ";
                std::cout << statistical_results->avg_avg_error << " ";
                std::cout << statistical_results->worst_case_error << " ";
                std::cout << statistical_results->avg_time_mcs << " ";
                std::cout << statistical_results->avg_num_iters << " ";
                std::cout << statistical_results->num_fails_50 << "\n";

//                mean_time += statistical_results->avg_time_mcs;
//                mean_error += statistical_results->avg_avg_error;

                img++;
            }
//            std::cout << "------------------------------------------------\n";

//            results_total.close();
//            results_matlab.close();
        }
    }

//    std::cout << "mean time " << (mean_time / num_images) << "\n";
//    std::cout << "mean error " << (mean_error / num_images) << "\n";

}



void getGTInliersFromGTModelFundamental (const std::string& filename, const cv::Mat& points, float threshold, std::vector<int> &gt_inliers) {
    cv::Mat gt_model;
    Reader::getMatrix3x3(filename, gt_model);
//    std::cout << gt_model << "\n";

    Estimator * estimator = new FundamentalEstimator (points);
    std::vector<int> inliers2;

    Quality::getInliers(estimator, gt_model, threshold, points.rows, gt_inliers);
    Quality::getInliers(estimator, gt_model.t(), threshold, points.rows, inliers2);

    if (inliers2.size() > gt_inliers.size()) {
        gt_inliers = inliers2;
    }
}