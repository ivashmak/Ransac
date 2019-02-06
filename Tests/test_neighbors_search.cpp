#include "tests.h"
#include "../dataset/Dataset.h"

void Tests::testNeighborsSearchCell () {
//    std::vector<std::string> points_filename = Dataset::getHomographyDatasetPoints();
    std::vector<std::string> points_filename = Dataset::getAdelaidermfDataset();

    int num_images = points_filename.size();

    Tests tests;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<std::vector<int>> gt_inliers;

    int N_runs = 200;
    int knn = 5;
    float threshold = 2;
    float confidence = 0.95;

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";
        cv::Mat_<float> points;

//        ImageData gt_data (DATASET::Homogr, img_name);
        ImageData gt_data (DATASET::Adelaidermf, img_name);
        points = gt_data.getPoints();

        gt_inliers.push_back(gt_data.getGTInliers(threshold));
        std::cout << "inliers size " << gt_inliers[gt_inliers.size()-1].size() << "\n";
        points_imgs.push_back(points);
    }

    std::cout << N_runs <<" for each " << num_images  << "\n";
    std::cout << "cell size | total avg time | total avg error | total avg num inliers\n";

    for (int cell_size = 25; cell_size <= 100; cell_size += 5) {
//        Model *model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Uniform);
        Model *model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Uniform);
        model->lo = LocOpt ::GC;
        model->setNeighborsType(NeighborsSearch::Grid);
        model->setCellSize(cell_size);
        model->ResetRandomGenerator(true);

        int img = 0;
        float avg_error = 0;
        float avg_num_inliers = 0;
        long avg_time = 0;

        for (const std::string &img_name : points_filename) {

//            std::cout << img_name << "\n";

            StatisticalResults * statistical_results = new StatisticalResults;
                tests.getStatisticalResults(points_imgs[img], model, N_runs,
                                            true, gt_inliers[img], true, statistical_results);

            avg_error += statistical_results->avg_avg_error;
            avg_num_inliers += statistical_results->avg_num_inliers;
            avg_time += statistical_results->avg_time_mcs;

            img++;
        }
        std::cout << cell_size << " " <<  (avg_time / num_images) << " " << (avg_error / num_images) << " " << (avg_num_inliers / num_images) << "\n";
    }
}


void Tests::testNeighborsSearch () {
    DATASET dataset = DATASET ::EVD; // Homogr, Kusvod2, Adelaidrmf, EVD
    std::vector<std::string> points_filename = Dataset::getDataset(dataset);

    int num_images = points_filename.size();
    std::cout << "number of images " << num_images << "\n";

    Tests tests;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<std::vector<int>> gt_inliers;

    int N_runs = 100;
    int knn = 5;
    float threshold = 2;
    float confidence = 0.95;
    std::vector<NeighborsSearch> neighbors_searches;
    neighbors_searches.push_back(NeighborsSearch::Nanoflann);
    neighbors_searches.push_back(NeighborsSearch::Grid);

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";
        cv::Mat_<float> points;

        ImageData gt_data (dataset, img_name);

        points = gt_data.getPoints();

        gt_inliers.push_back(gt_data.getGTInliers(threshold));
        std::cout << "inliers size " << gt_inliers[gt_inliers.size()-1].size() << "\n";
        points_imgs.push_back(points);
    }


    std::cout << N_runs <<" for each " << num_images  << "\n";

    std::vector<std::vector<float>> res (2);

    int ns = 0;
    for (NeighborsSearch neighborsSearch : neighbors_searches) {
        std::cout << nearestNeighbors2string (neighborsSearch) << "\n";

        Model *model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Uniform);
//        Model *model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Uniform);
        model->lo = LocOpt ::GC;
        model->setNeighborsType(neighborsSearch);
        model->setCellSize(50);
        model->ResetRandomGenerator(true);

        int img = 0;
        float avg_error = 0;
        float worst_case_err = 0;
        float avg_time = 0;

        for (const std::string &img_name : points_filename) {

            std::cout << img_name << "\n";

            StatisticalResults * statistical_results = new StatisticalResults;
            tests.getStatisticalResults(points_imgs[img], model, N_runs,
                                        true, gt_inliers[img], true, statistical_results);

            avg_error += statistical_results->avg_avg_error;
            worst_case_err += statistical_results->worst_case_error;
            avg_time += statistical_results->avg_time_mcs;

            res[ns].push_back(statistical_results->avg_avg_error);
            res[ns].push_back(statistical_results->worst_case_error);
            res[ns].push_back(statistical_results->avg_time_mcs);
            std::cout << statistical_results->avg_avg_error << " ";
            std::cout << statistical_results->worst_case_error << " ";
            std::cout << statistical_results->avg_time_mcs << "\n";

            img++;
        }
        float mean_time = (avg_time / num_images);
        float mean_err = (avg_error / num_images);
        float mean_err_wc = (worst_case_err / num_images);
        std::cout << mean_time  << " " << mean_err << " " << mean_err_wc << "\n";
        std::cout << "--------------------------------------------\n";
        ns++;
    }

    int time_is_better = 0;
    int err_is_better = 0;
    int err_wc_is_better = 0;

    for (int j = 0; j < num_images; j++) {
//        std::cout << res[1][3*j] << " vs " << res[0][3*j] << "\n";
//        std::cout << res[1][3*j+1] << " vs " << res[0][3*j+1] << "\n";
//        std::cout << res[1][3*j+2] << " vs " << res[0][3*j+2] << "\n";
//        std::cout << "- - - - - - -\n";
        if (res[1][3*j] <= res[0][3*j]) {
            err_is_better++;
        }
        if (res[1][3*j+1] <= res[0][3*j+1]) {
            err_wc_is_better++;
        }
        if (res[1][3*j+2] <= res[0][3*j+2]) {
            time_is_better++;
        }
    }
    std::cout << "error is better " << err_is_better << "\n";
    std::cout << "error wc is better " << err_wc_is_better << "\n";
    std::cout << "time is better " << time_is_better << "\n";

}
