#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Ransac/Ransac.h"

#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"

#include "../Detector/ReadPoints.h"

#include "../Usac/Sampler/Sampler.h"
#include "../Usac/Sampler/NapsacSampler.h"
#include "../Usac/Sampler/GradualNapsacSampler.h"
#include "../Usac/Sampler/EvsacSampler.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Sampler/ProsacSimpleSampler.h"
#include "../Usac/Utils/NearestNeighbors.h"
#include "../Usac/TerminationCriteria/ProsacTerminationCriteria.h"
#include "../Usac/Sampler/ProsacSampler.h"

void store_results_line2d ();
int getGTInliers (const cv::Mat& points, const cv::Mat& gt_model, float threshold);

void Tests::testLineFitting() {
     std::string img_name = "../dataset/image1";
     std::vector<cv::Point_<float>> points;
     cv::Mat_<float> gt_model;

    // change false to true to reset time for random points generator
    // get number of ground truth inliers too.
     generate(points, false, true, gt_model);
    
    // another generated data 
//    std::string img_name = "../dataset/line2d/w=1000_h=1000_n=3.000000_I=500_N=10500";
//    std::ifstream read_data_file;
//    read_data_file.open (img_name+".txt");
//    int width, height, noise, N;
//    read_data_file >> width;
//    read_data_file >> height;
//    read_data_file >> noise;
//    read_data_file >> gt_inliers;
//    read_data_file >> N;
//    std::vector<cv::Point_<float>> points;
//    float x, y;
//    for (int p = 0; p < N; p++) {
//        read_data_file >> x >> y;
//        cv::Point_<float> pt (x, y);
//        points.push_back (pt);
//    }
    // 

    std::cout << "generated points\n";


    unsigned int points_size = points.size();

    int knn = 13;
    cv::Mat_<float> pts = cv::Mat (points);

    cv::Mat_<float> neighbors, neighbors_dists;
    NearestNeighbors nn;
    nn.getNearestNeighbors_nanoflann(pts, knn, neighbors, true, neighbors_dists);
    std::vector<int> sorted_idx (points_size);
    std::iota(sorted_idx.begin(), sorted_idx.end(), 0);

    /*
     * Prosac quality sort.
     * Sorting by sum of distances of the (3) nearest neighbors.
     */
    float sum1, sum2;
    int idxa, idxb;
    float * neighbors_dists_ptr = (float *) neighbors_dists.data;
    std::sort(sorted_idx.begin(), sorted_idx.end(), [&] (int a, int b) {
        sum1 = 0, sum2 = 0;
        idxa = knn*a, idxb = knn*b;
        for (int i = 0; i < 3; i++) {
            sum1 += neighbors_dists_ptr[idxa + i];
            sum2 += neighbors_dists_ptr[idxb + i];
        }
        return sum1 < sum2;
    });

    cv::Mat_<float> sorted_points;
    for (int i = 0; i < points_size; i++) {
        sorted_points.push_back(points[sorted_idx[i]]);
    }

    cv::Mat_<float> sorted_pts = cv::Mat (sorted_points);

    Model * model;

    // ---------------- uniform -------------------
//     model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Uniform);
//
//     model->setStandardRansacLO(0);
//     model->setGraphCutLO(0);
//     model->setSprtLO(0);
//
//     initUniform(sampler, model->sample_number, points_size);
//     test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, img_name, gt_inliers);
    //------------------------------------------



    // --------------  prosac ---------------------
    model = new Model (10, 4, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Prosac);
    model->setStandardRansacLO(0);
    model->setGraphCutLO(0);
    model->setSprtLO(0);

    test (sorted_pts, model, img_name, true, gt_model);
     // ------------------------------------------------





    // ---------------- napsac -------------------------------
    // model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
    // model->setSprtLO(1);
    // model->setGraphCutLO(1);
    // model->setStandardRansacLO(1);

//     test (pts, model, img_name, true, gt_model);
    // ---------------------------------------------------------------------




    // ----------------- evsac ------------------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Evsac);
//
//     test (pts, model, img_name, true, gt_model);
    // ------------------------------------------------------------





    // ------------------ gradually increasing ransac ----------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::GradualNapsac);
//
//     test (pts, model, img_name, true, gt_model);
    // --------------------------------------------------------


//     getStatisticalResults(pts, model, 1000, true, false, gt_model, nullptr);


//   store_results_line2d();
}

// problem for 111
void store_results_line2d () {

    std::vector<std::string> dataset;
    std::vector<cv::Mat_<float>> dataset_points;
    std::vector<cv::Mat_<float>> gt_models;

    std::ifstream read_dataset;
    read_dataset.open("../dataset/line2d/dataset.txt");
    std::string data_file;
    while (read_dataset >> data_file) {
        std::cout << data_file << "\n";
        std::ifstream read_data_file;
        read_data_file.open ("../dataset/line2d/"+data_file+".txt");
        if (!read_data_file.is_open()) {
            std::cout << "file not open\n";
            exit (0);
        }
        int width, height, noise, N;

        read_data_file >> width;
        read_data_file >> height;
        read_data_file >> noise;
        float a, b, c;
        read_data_file >> a;
        read_data_file >> b;
        read_data_file >> c;

        cv::Mat gt_model;
        gt_model = (cv::Mat_<float> (1,3) << a, b, c);
        gt_models.push_back(gt_model);

        read_data_file >> N;
        cv::Mat_<float> points(N, 2);
        float *pts = (float *) points.data;
        float x, y;
        for (int p = 0; p < N; p++) {
//            std::cout << p << "\n";
            read_data_file >> x >> y;
            pts[2*p] = x;
            pts[2*p+1] = y;
        }
        dataset_points.push_back(points);
        dataset.push_back(data_file);
    }
    std::cout << "Got data files\n";

    Tests tests;

    int N_runs = 50;
    NearestNeighbors nn;

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
    samplers.push_back(SAMPLER::Prosac);
    samplers.push_back(SAMPLER::Napsac);

    int lo_combinations = 5;
    bool lo[lo_combinations][3] = {
            {0, 0, 0},
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
            {1, 1, 1},
    };

    for (SAMPLER smplr : samplers) {
        for (int l = 0; l < lo_combinations; l++) {
            std::ofstream results_total;
            std::ofstream results_matlab;
            std::string mfname = "../results/line2d/" + tests.sampler2string(smplr) + "_" +
                                 std::to_string(lo[l][0]) + std::to_string(lo[l][1]) + std::to_string(lo[l][2]) +
                                 "_m.csv";
            std::string fname = "../results/line2d/" + tests.sampler2string(smplr) + "_" +
                                std::to_string(lo[l][0]) + std::to_string(lo[l][1]) + std::to_string(lo[l][2]) + ".csv";

            results_matlab.open(mfname);
            results_total.open(fname);
            int knn = 3;

            Model *model = new Model(10, 2, 0.99, knn, ESTIMATOR::Line2d, smplr);
            model->setStandardRansacLO(lo[l][0]);
            model->setGraphCutLO(lo[l][1]);
            model->setSprtLO(lo[l][2]);

            results_total << tests.getComputerInfo();
            results_total << model->getName() << "\n";
            results_total << "Runs for each image = " << N_runs << "\n";
            results_total << "Threshold for each image = " << model->threshold << "\n";
            results_total << "Desired probability for each image = " << model->desired_prob << "\n";
            results_total << "Standard LO = " << (bool) model->LO << "\n";
            results_total << "Graph Cut LO = " << (bool) model->GraphCutLO << "\n";
            results_total << "SPRT = " << (bool) model->SprtLO << "\n\n\n";

            results_total << "Filename,Avg num inl/gt,Std dev num inl,Med num inl,"
                             "Avg num iters,Std dev num iters,Med num iters,"
                             "Avg time (mcs),Std dev time,Med time,"
                             "Num fails\n";
            int img = 0;

            std::cout << tests.sampler2string(smplr) << "\n";
            std::cout << lo[l][0] << " " << lo[l][1] << " " << lo[l][2] << "\n";
            
            for (auto &points : dataset_points) {

                std::cout << dataset[img] << "\n";
                unsigned int points_size = points.rows;
                cv::Mat_<float> neighbors, neighbors_dists;
                nn.getNearestNeighbors_nanoflann(points, knn, neighbors, true, neighbors_dists);
                std::vector<int> sorted_idx(points_size);
                std::iota(sorted_idx.begin(), sorted_idx.end(), 0);

                cv::Mat_<float> sorted_points;
                if (smplr == SAMPLER::Prosac) {
                    float sum1, sum2;
                    int idxa, idxb;
                    float *neighbors_dists_ptr = (float *) neighbors_dists.data;
                    std::sort(sorted_idx.begin(), sorted_idx.end(), [&](int a, int b) {
                        sum1 = 0, sum2 = 0;
                        idxa = knn * a, idxb = knn * b;
                        for (int i = 0; i < 3; i++) {
                            sum1 += neighbors_dists_ptr[idxa + i];
                            sum2 += neighbors_dists_ptr[idxb + i];
                        }
                        return sum1 < sum2;
                    });

                    for (int i = 0; i < points_size; i++) {
                        sorted_points.push_back(points.row(sorted_idx[i]));
                    }
                    sorted_points.copyTo(points);
                }

                int gt_inliers = getGTInliers (points, gt_models[img], model->threshold);

                StatisticalResults *statistical_results = new StatisticalResults;
                tests.getStatisticalResults(points, model, N_runs, true, true, gt_models[img], statistical_results);

                // save to csv file
                results_total << dataset[img] << ",";
                results_total << statistical_results->avg_num_inliers << " / " << gt_inliers << ",";
                results_total << statistical_results->std_dev_num_inliers << ",";
                results_total << statistical_results->median_num_inliers << ",";

                results_total << statistical_results->avg_num_iters << ",";
                results_total << statistical_results->std_dev_num_iters << ",";
                results_total << statistical_results->median_num_iters << ",";

                results_total << statistical_results->avg_time_mcs << ",";
                results_total << statistical_results->std_dev_time_mcs << ",";
                results_total << statistical_results->median_time_mcs << ",";

                results_total << statistical_results->num_fails << "\n";

                results_matlab << dataset[img] << ",";
                results_matlab << statistical_results->avg_num_inliers << ",";
                results_matlab << statistical_results->std_dev_num_inliers << ",";

                results_matlab << statistical_results->avg_num_iters << ",";
                results_matlab << statistical_results->std_dev_num_iters << ",";

                results_matlab << statistical_results->avg_time_mcs << ",";
                results_matlab << statistical_results->std_dev_time_mcs << ",";

                results_matlab << statistical_results->num_fails << "\n";
                img++;
            }

            results_total.close();
            results_matlab.close();
        }
    }
}

int getGTInliers (const cv::Mat& points, const cv::Mat& gt_model, float threshold) {
    Estimator * estimator = new Line2DEstimator(points);
    int inliers_size = 0;
    for (int i = 0; i < points.rows; i++) {
        inliers_size += (estimator->GetError(i) < threshold);
    }
    return inliers_size;
}