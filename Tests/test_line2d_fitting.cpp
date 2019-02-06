#include <cstdlib>
#include "tests.h"

#include "../Generator/generator.h"
#include "../usac/estimator/line2d_estimator.hpp"
#include "../usac/ransac/ransac.hpp"

#include "../helper/drawing/Drawing.h"
#include "../helper/Logging.h"

#include "../Detector/Reader.h"

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
void getGTInliers (const cv::Mat& points, const cv::Mat& gt_model, float threshold, std::vector<int> &gt_inliers);

void Tests::testLineFitting() {
    DATASET dataset = DATASET ::Syntectic;
     std::string img_name;
     img_name = "../dataset/image1";
     std::vector<cv::Point_<float>> points;
     cv::Mat_<float> gt_model;

    // change false to true to reset time for random points generator
    // get number of ground truth inliers too.
     generate(points, false, true, gt_model);

    // another generated data
//    img_name = "../dataset/line2d/w=1000_h=1000_n=3.000000_I=500_N=10500";
//    std::ifstream read_data_file;
//    read_data_file.open (img_name+".txt");
//    int width, height, noise, N;
//    float a,b,c;
//    read_data_file >> width;
//    read_data_file >> height;
//    read_data_file >> noise;
//    read_data_file >> a;
//    read_data_file >> b;
//    read_data_file >> c;
//    gt_model = (cv::Mat_<float>(1,3) << a,b,c);
//    read_data_file >> N;
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
    float threshold = 10;
    std::vector<int> gt_inliers;
    getGTInliers (pts, gt_model, threshold, gt_inliers);


    // ---------------- uniform -------------------
//     model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Uniform);
//------------------------------------------


    // --------------  prosac ---------------------
//    model = new Model (threshold, 4, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Prosac);
     // ------------------------------------------------


    // ---------------- napsac -------------------------------
    // model = new Model (threshold, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
    // ---------------------------------------------------------------------


    // ----------------- evsac ------------------------------
//    model = new Model (threshold, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Evsac);
// ------------------------------------------------------------


    // ------------------ Progressive Napsac ----------------------
    model = new Model (threshold, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::ProgressiveNAPSAC);
// --------------------------------------------------------

     model->lo = LocOpt ::NullLO;
     model->setSprt(0);
     model->setNeighborsType(NeighborsSearch::Nanoflann);

    test (pts, model, img_name, dataset, true, gt_inliers);
//    test (sorted_pts, model, img_name, dataset, true, gt_inliers);

//     getStatisticalResults(pts, model, 1000, true, false, gt_model, nullptr);

//   store_results_line2d();
}

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

    std::vector<LocOpt > loc_opts;
    loc_opts.push_back(LocOpt::InItRsc);
    bool sprt = 0;

    for (SAMPLER smplr : samplers) {
        for (auto loc_opt : loc_opts) {
            std::ofstream results_total;
            std::ofstream results_matlab;
            std::string name = "../results/line2d/" + Tests::sampler2string(smplr);
            if (loc_opt == LocOpt::InItRsc) name += "_lo";
            if (loc_opt == LocOpt::GC) name += "_gc";
            if (sprt) name += "_sprt";

            std::string mfname = name+"_m.csv";
            std::string fname = name+".csv";

            results_matlab.open(mfname);
            results_total.open(fname);

            Model *model = new Model(threshold, 2, confidence, knn, ESTIMATOR::Line2d, smplr);
            model->lo = loc_opt;
            model->setSprt(sprt);

            results_total << tests.getComputerInfo();
            results_total << model->getName() << "\n";
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

            std::cout << tests.sampler2string(smplr) << "\n";
            std::cout << "LO  " << loc_opt << "\n";
            
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