#include "tests.h"
#include "../detector/Reader.h"
#include "../usac/utils/utils.hpp"
#include "../helper/Logging.h"
#include "../usac/estimator/homography_estimator.hpp"
#include "../dataset/Dataset.h"

void storeResultsHomography ();

void Tests::testHomographyFitting() {
    float threshold = 2;

//    detectAndSaveFeatures(getHomographyDatasetPoints());
//    exit (0);

    DATASET dataset = DATASET::Homogr_SIFT;
    std::string img_name = "adam";

    ImageData gt_data (dataset, img_name);

    cv::Mat points = gt_data.getPoints();
    cv::Mat sorted_points = gt_data.getSortedPoints();

    std::vector<int> gt_inliers = gt_data.getGTInliers(threshold);
    std::vector<int> gt_sorted_inliers = gt_data.getGTInliersSorted(threshold);


    unsigned int points_size = (unsigned int) points.rows;
    std::cout << "points size " << points_size << "\n";
    std::cout << "sorted points size " << sorted_points.rows << "\n";

    unsigned int knn = 7;
    float confidence = 0.95;

    std::cout << "gt inliers " << gt_inliers.size() << "\n";
    std::cout << "gt inliers sorted " << gt_sorted_inliers.size() << "\n";

    Model * model;

//     ---------------------- uniform ----------------------------------
   model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Uniform);
//     --------------------------------------------------------------

//     ---------------------- napsac ----------------------------------
//    model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Napsac);
//     --------------------------------------------------------------

// ------------------ prosac ---------------------
//     model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Prosac);
//     -------------------------------------------------


     model->lo = LocOpt ::GC;
     model->setSprt(0);
     model->setCellSize(50);
     model->setNeighborsType(NeighborsSearch::Grid);
     model->ResetRandomGenerator(false);

     test (points, model, img_name, dataset, true, gt_inliers);
//     test (sorted_points, model, img_name, dataset, true, gt_sorted_inliers);

//    getStatisticalResults(points, model, 10, true, gt_inliers, false, nullptr);
//    getStatisticalResults(sorted_points, model, 100, true, gt_sorted_inliers, false, nullptr);

//    storeResultsHomography();
}


/*
 * Store results from dataset to csv file.
 */
void storeResultsHomography () {
    DATASET dataset = DATASET ::Homogr_SIFT; // Homogr, EVD
    std::vector<std::string> points_filename = Dataset::getDataset(dataset);
    int num_images = points_filename.size();
    std::cout << "number of images " << num_images << "\n";

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<cv::Mat_<float>> sorted_points_imgs;
    std::vector<std::vector<int>> gt_inliers;
    std::vector<std::vector<int>> gt_inliers_sorted;

    int N_runs = 100;
    unsigned int knn = 5;
    float threshold = 2, confidence = 0.95;

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";

        ImageData gt_data (dataset, img_name);
        cv::Mat points = gt_data.getPoints();
        cv::Mat sorted_points = gt_data.getSortedPoints();

        gt_inliers.push_back(gt_data.getGTInliers(threshold));
        gt_inliers_sorted.push_back(gt_data.getGTInliersSorted(threshold));
        points_imgs.emplace_back(points);
        sorted_points_imgs.emplace_back(sorted_points);

        std::cout << "inliers size " << gt_inliers[gt_inliers.size()-1].size() << "\n";
        std::cout << "sorted inliers size " << gt_inliers_sorted[gt_inliers.size()-1].size() << "\n";
    }

//    exit (0);

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
//    samplers.push_back(SAMPLER::Prosac);

    std::vector<LocOpt > loc_opts;
    loc_opts.push_back(LocOpt::GC);
    bool sprt = 0;

    NeighborsSearch neighborsSearch = NeighborsSearch ::Grid;
    int cell_size = 50;

    long mean_time = 0;
    long mean_error = 0;

    for (SAMPLER smplr : samplers) {
        for (auto loc_opt : loc_opts) {
            std::ofstream results_total;
            std::ofstream results_matlab;
            std::string name = "../results/EVD/";
            name += Tests::sampler2string(smplr);
            if (loc_opt == LocOpt::InItLORsc) name += "_lo";
            if (loc_opt == LocOpt::InItFLORsc) name += "_flo";
            if (loc_opt == LocOpt::GC) name += "_gc";
            if (sprt) name += "_sprt";
            name += "_"+Tests::nearestNeighbors2string(neighborsSearch) + "_c_sz_"+std::to_string(cell_size);

            std::string mfname = name+"_m.csv";
            std::string fname = name+".csv";

            Model *model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, smplr);
            model->lo = loc_opt;
            model->setSprt(sprt);
            model->setNeighborsType(neighborsSearch);
            model->setCellSize(cell_size);

            results_matlab.open (mfname);
            results_total.open (fname);

            Logging::saveHeadOfCSV (results_total, model, N_runs);

            int img = 0;
            for (const std::string &img_name : points_filename) {

//                std::cout << img_name << "\n";

                StatisticalResults * statistical_results = new StatisticalResults;
                if (smplr == SAMPLER::Prosac) {
                    Tests::getStatisticalResults(sorted_points_imgs[img], model, N_runs,
                                                true, gt_inliers_sorted[img], true, statistical_results);
                } else {
                    Tests::getStatisticalResults(points_imgs[img], model, N_runs,
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
                std::cout << statistical_results->avg_num_lo_iters << " ";
                std::cout << statistical_results->avg_avg_error << " ";
                std::cout << statistical_results->worst_case_error << " ";
                std::cout << statistical_results->avg_time_mcs << " ";
                std::cout << statistical_results->avg_num_iters << " ";
                std::cout << statistical_results->num_fails_50 << "\n";


//                std::cout << " +- " << statistical_results->std_dev_avg_error << "\n";
//                std::cout << "- - - - - - - - - - - - - - - - - -\n";

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