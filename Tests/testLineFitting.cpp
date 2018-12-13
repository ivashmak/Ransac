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
#include "../Usac/Utils/Utils.h"

void store_results_line2d ();
void getGTInliers (const cv::Mat& points, const cv::Mat& gt_model, float threshold, std::vector<int> &gt_inliers);

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
    std::vector<cv::Mat_<float>> dataset_sorted_points;
    std::vector<std::vector<int>> gt_inliers;

    Logging log;
    Tests tests;
    float confidence = 0.95;
    float threshold = 8;
    int N_runs = 50;
    int knn = 3;


    std::ifstream read_dataset;
    std::string data_file;
    read_dataset.open("../dataset/line2d/dataset.txt");
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

        read_data_file >> N;
        cv::Mat_<float> points(N, 2);
        float *pts = (float *) points.data;
        float x, y;
        for (int p = 0; p < N; p++) {
            read_data_file >> x >> y;
            pts[2*p] = x;
            pts[2*p+1] = y;
        }

        cv::Mat_<float> sorted_points;
        densitySort (points, 3, sorted_points);

        std::vector<int> gt_inliers_;
        getGTInliers (points, gt_model, threshold, gt_inliers_);

        // save points and gt
        gt_inliers.push_back(gt_inliers_);
        dataset_points.push_back(points);
        dataset_sorted_points.push_back(sorted_points);
        dataset.push_back(data_file);
    }
    std::cout << "Got data files\n";

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

            Model *model = new Model(threshold, 2, confidence, knn, ESTIMATOR::Line2d, smplr);
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
            results_total << "SPRT = " << (bool) model->Sprt << "\n\n\n";

            results_total << "Filename,GT Inl,Avg num inl/gt,Std dev num inl,Med num inl,"
                             "Avg num iters,Std dev num iters,Med num iters,"
                             "Avg num LO iters,Std dev num LO iters,Med num LO iters,"
                             "Avg time (mcs),Std dev time,Med time,"
                             "Avg err,Std dev err,Med err,"
                             "Worst case num Inl,Worst case Err,"
                             "Num fails (<10%),Num fails (<25%),Num fails (<50%)\n";

            std::cout << tests.sampler2string(smplr) << "\n";
            std::cout << lo[l][0] << " " << lo[l][1] << " " << lo[l][2] << "\n";
            
            for (int img = 0; img < dataset.size(); img++) {
                std::cout << dataset[img] << "\n";

                StatisticalResults *statistical_results = new StatisticalResults;
                if (smplr == SAMPLER::Prosac) {
                    tests.getStatisticalResults(dataset_sorted_points[img], model, N_runs,
                                                true, gt_inliers[img], true, statistical_results);
                } else {
                    tests.getStatisticalResults(dataset_points[img], model, N_runs,
                                                true, gt_inliers[img], true, statistical_results);
                }

                // save to csv file
                // save to csv file
                results_total << dataset[img] << ",";
                results_total << gt_inliers[img].size() << ",";
                log.saveResultsCSV(results_total, statistical_results);

                // save results for matlab
                results_matlab << dataset[img] << ",";
                log.saveResultsMatlab(results_matlab, statistical_results);

                img++;
            }

            results_total.close();
            results_matlab.close();
        }
    }
}

void getGTInliers (const cv::Mat& points, const cv::Mat& gt_model, float threshold, std::vector<int> &gt_inliers) {
    Estimator * estimator = new Line2DEstimator(points);
    for (int i = 0; i < points.rows; i++) {
        if (estimator->GetError(i) < threshold) {
            gt_inliers.push_back(i);
        }
    }
}