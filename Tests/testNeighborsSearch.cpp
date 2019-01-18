#include "Tests.h"
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
        model->setGraphCutLO(1);
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
//    std::vector<std::string> points_filename = Dataset::getHomographyDatasetPoints();
//    std::vector<std::string> points_filename = Dataset::getEVDDataset();
//    std::vector<std::string> points_filename = Dataset::getAdelaidermfDataset();
    std::vector<std::string> points_filename = Dataset::getKusvod2Dataset();



    int num_images = points_filename.size();

    Tests tests;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<std::vector<int>> gt_inliers;

    int N_runs = 200;
    int knn = 5;
    float threshold = 2;
    float confidence = 0.95;
    std::vector<NeighborsSearch> neighbors_searches;
    neighbors_searches.push_back(NeighborsSearch::Nanoflann);
    neighbors_searches.push_back(NeighborsSearch::Grid);

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";
        cv::Mat_<float> points;

//        ImageData gt_data (DATASET::Homogr, img_name);
//        ImageData gt_data (DATASET::EVD, img_name);
//        ImageData gt_data (DATASET::Adelaidermf, img_name);
        ImageData gt_data (DATASET::Kusvod2, img_name);

        points = gt_data.getPoints();

        gt_inliers.push_back(gt_data.getGTInliers(threshold));
        std::cout << "inliers size " << gt_inliers[gt_inliers.size()-1].size() << "\n";
        points_imgs.push_back(points);
    }

    exit (0);

    std::cout << N_runs <<" for each " << num_images  << "\n";
    std::cout << "cell size | total avg time | total avg error | total avg num inliers\n";

    for (NeighborsSearch neighborsSearch : neighbors_searches) {
        Model *model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Uniform);
//        Model *model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Uniform);
        model->setGraphCutLO(1);
        model->setNeighborsType(neighborsSearch);
        model->setCellSize(50);
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
        std::cout << (avg_time / num_images) << " " << (avg_error / num_images) << " " << (avg_num_inliers / num_images) << "\n";
    }
}
