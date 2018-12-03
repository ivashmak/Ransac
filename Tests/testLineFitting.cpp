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

void Tests::testLineFitting() {
    // std::string img_name = "../dataset/image1";
    // std::vector<cv::Point_<float>> points;
    int gt_inliers;
    // change false to true to reset time for random points generator
    // get number of ground truth inliers too.
    // generate(points, false, true, &gt_inliers);
    
    // another generated data 
    std::string img_name = "../dataset/line2d/w=1000_h=1000_n=3.000000_I=500_N=10500";
    std::ifstream read_data_file;
    read_data_file.open (img_name+".txt");
    int width, height, noise, N;
    read_data_file >> width;
    read_data_file >> height;
    read_data_file >> noise;
    read_data_file >> gt_inliers;
    read_data_file >> N;
    std::vector<cv::Point_<float>> points;
    float x, y;
    for (int p = 0; p < N; p++) {
        read_data_file >> x >> y;
        cv::Point_<float> pt (x, y);
        points.push_back (pt);
    }
    // 

    std::cout << "generated points\n";


    unsigned int points_size = points.size();

    int knn = 3;
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

    std::vector<cv::Point_<float>> sorted_points;
    for (int i = 0; i < points_size; i++) {
        sorted_points.push_back(points[sorted_idx[i]]);
    }


    // Main Ransac components
    Estimator *estimator = new Line2DEstimator (points);
    TerminationCriteria * termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;
    Model * model;
    Sampler * sampler;
    //



    // ---------------- uniform -------------------
    // model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Uniform);

    // model->setStandardRansacLO(1);
    // model->setGraphCutLO(1);
    // model->setSprtLO(1);

    // initUniform(sampler, model->sample_number, points_size);
    // test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, img_name, gt_inliers);
    //------------------------------------------



    // --------------  prosac ---------------------
    // model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Prosac);
    // model->setStandardRansacLO(1);
    // model->setGraphCutLO(1);
    // model->setSprtLO(1);
    
    // initProsac(sampler, model->sample_number, points.size());
    // ProsacSampler *prosac_sampler_ = (ProsacSampler *) sampler;

    // ProsacTerminationCriteria * prosac_termination_criteria_ = new ProsacTerminationCriteria;
    // prosac_termination_criteria_->initProsacTerminationCriteria (prosac_sampler_->getGrowthFunction(),
    //                                            model, points_size);

    // TerminationCriteria * prosac_termination_criteria = prosac_termination_criteria_;
    // cv::Mat sorted_pts (sorted_points);
    // estimator = new Line2DEstimator (sorted_points);

    // test (sorted_pts, estimator, sampler, model, quality, prosac_termination_criteria, neighbors, img_name, gt_inliers);

    // // switch to unsorted points back (not necessary, just for testing)
    // estimator = new Line2DEstimator (points);
    // ------------------------------------------------





    // ---------------- napsac -------------------------------
    // model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
    // model->setSprtLO(1);
    // model->setGraphCutLO(1);
    // model->setStandardRansacLO(1);

    // initNapsac(sampler, neighbors, model->k_nearest_neighbors, model->sample_number);
    // test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, img_name, gt_inliers);
    // ---------------------------------------------------------------------




    // ----------------- evsac ------------------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Evsac);
//    sampler = new EvsacSampler(points, points.size(), evsac_model->k_nearest_neighbors, evsac_model->sample_number);
//
//     test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, img_name, gt_inliers);
    // ------------------------------------------------------------





    // ------------------ gradually increasing ransac ----------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::GradualNapsac);
//    sampler = new GradualNapsacSampler(points, gradual_napsac_model->sample_number);
//
//     test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, img_name, gt_inliers);
    // --------------------------------------------------------



//     getStatisticalResults(pts, estimator, model, sampler, termination_criteria, quality, neighbors,
//                           1000, true, false, gt_inliers);



   store_results_line2d();
}

// problem for 111
void store_results_line2d () {

    std::vector<std::string> dataset;
    std::vector<cv::Mat_<float>> dataset_points;
    std::vector<int> gt_inliers;

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
        int width, height, inliers, noise, N;
        read_data_file >> width;
        read_data_file >> height;
        read_data_file >> noise;
        read_data_file >> inliers;
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
        gt_inliers.push_back(inliers);
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
                }

                TerminationCriteria *termination_criteria = new StandardTerminationCriteria;
                Quality *quality = new Quality;
                Estimator *estimator;
                Sampler *sampler;

                tests.initSampler(sampler, model, points.rows, points, neighbors);

                ProsacTerminationCriteria * prosac_termination_criteria_ = new ProsacTerminationCriteria;
                    
                if (smplr == SAMPLER::Prosac) {
                    estimator = new Line2DEstimator(sorted_points);
                    ProsacSampler *prosac_sampler_ = (ProsacSampler *) sampler;

                    prosac_termination_criteria_->initProsacTerminationCriteria (prosac_sampler_->getGrowthFunction(),
                                                               model, points_size);

                    termination_criteria = prosac_termination_criteria_;                    
                } else {
                    estimator = new Line2DEstimator(points);
                }
                

                StatisticalResults *statistical_results = new StatisticalResults;
                tests.getStatisticalResults(points, estimator, model, sampler, termination_criteria,
                                            quality, neighbors, N_runs, true, true, gt_inliers[img], statistical_results);

                // save to csv file
                results_total << dataset[img] << ",";
                results_total << statistical_results->avg_num_inliers << " / " << gt_inliers[img] << ",";
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
