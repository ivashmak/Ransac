#include "tests.h"
#include "../helper/Logging.h"

void storeResultsEssential ();
void detectFeaturesMVS () {
    cv::Mat P1, P2;
    float threshold = 1;

    for (int i = 1; i < 6; i++) {
        std::vector<std::string> images = Dataset::getStrechaCastleDenseImages(i);
        int img_id = 0;
        for (int img1 = 0; img1 < images.size(); img1++) {
            for (int img2 = img1+1; img2 < images.size(); img2++) {
                std::cout << "pair " << images[img1] << " " << images[img2] << "\n";
                unsigned long name_idx = images[img1].find_last_of('_');
                std::string filename = images[img1].substr(0, name_idx);
                std::string folder = "../dataset/MVS/"+filename+"/";

                Reader::readProjectionMatrix(P1, folder+images[img1]+".P");
                Reader::readProjectionMatrix(P2, folder+images[img2]+".P");

                bool ok = detectFeaturesForEssentialMatrix(P1, P2, folder+images[img1], folder+images[img2],
                                                 "../dataset/MVS/"+filename+"_"+std::to_string(img_id), threshold);
                if (! ok) {
                    std::cout << "CONTINUE\n";
                    continue;
                }

                // copy images
                std::ifstream  src1(folder+images[img1], std::ios::binary);
                std::ofstream  dst1("../dataset/MVS/"+filename+"_"+std::to_string(img_id)+"A.png",   std::ios::binary);
                dst1 << src1.rdbuf();

                std::ifstream  src2(folder+images[img2], std::ios::binary);
                std::ofstream  dst2("../dataset/MVS/"+filename+"_"+std::to_string(img_id)+"B.png",   std::ios::binary);
                dst2 << src2.rdbuf();

                img_id++;
            }
        }
    }
}

void Tests::testEssentialFitting() {
//    detectFeaturesMVS();
//    exit (0);

    DATASET dataset = DATASET::Strecha;
    std::string img_name = "fountain_dense_25";

    float threshold = 2, confidence = 0.95;
    unsigned int knn = 6;

    ImageData img_data (dataset, img_name);
    cv::Mat points = img_data.getPoints();
    cv::Mat sorted_points = img_data.getPoints();
    std::vector<int> gt_inliers = img_data.getGTInliers(threshold);
    std::vector<int> gt_sorted_inliers = img_data.getGTInliersSorted(threshold);

    Model * model;

    // -------------------------- uniform -------------------------------------
//    model = new Model (threshold, 5, confidence, knn, ESTIMATOR::Essential, SAMPLER::Uniform);
    // ------------------------------------------------------------------------

    // -------------------------- Prosac -------------------------------------
    model = new Model (threshold, 5, confidence, knn, ESTIMATOR::Essential, SAMPLER::Prosac);
    // ------------------------------------------------------------------------

    model->lo = LocOpt ::NullLO;
    model->setSprt(0);
    model->setCellSize(50);
    model->setNeighborsType(NeighborsSearch::Grid);
    model->ResetRandomGenerator(true);

    if (model->sampler == SAMPLER::Prosac)
        test (sorted_points, model, img_name, dataset, true, gt_sorted_inliers);
    else
        test (points, model, img_name, dataset, true, gt_inliers);
}


void storeResultsEssential () {
    DATASET dataset = DATASET ::Strecha;
    std::vector<std::string> points_filename = Dataset::getDataset(dataset);
    int num_images = points_filename.size();
    std::cout << "number of images " << num_images << "\n";

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<cv::Mat_<float>> sorted_points_imgs;
    std::vector<std::vector<int>> gt_inliers;
    std::vector<std::vector<int>> gt_inliers_sorted;

    int N_runs = 100;
    unsigned int knn = 5;
    float threshold = 2;
    float confidence = 0.95;

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
    loc_opts.push_back(LocOpt::InItLORsc);
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

            Model *model = new Model (threshold, 5, confidence, knn, ESTIMATOR::Essential, smplr);
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