#include "Evsac.h"
#include "Helper/Drawing.h"
// https://github.com/feixh/PnPRANAAC/blob/a20df02db963128d4796f29dca35ed4becfddfbe/include/theia/solvers/evsac_sampler.h



void fitGEV (cv::InputArray points, float *mu, float *sigma, float *epsilon) {

}

// fit a two-parameter Gamma distribution to the data identified as correct to estimate Fc.
void fitGamma (cv::InputArray points, float *alpha, float *beta) {
    // https://stats.stackexchange.com/questions/342639/how-to-find-alpha-and-beta-from-a-gamma-distribution
    cv::Scalar mean, standard_deviation;
    cv::meanStdDev(points, mean, standard_deviation);

    float mean_v = (float) mean.val[0];
    float variance = (float) pow(standard_deviation.val[0], 2);

    *alpha = (float) (pow(mean_v, 2) / variance);
    *beta = mean_v/variance;
}

// require: image feature correspondences. (n image points sets)
// require: k nearest neighbor matching scores sorted in an ascending order for every i-th correspondence.

void Evsac::run (cv::InputArray input_points1, cv::InputArray input_points2, Estimator *estimator2d) {
    int knn = 10;

    int total_points = input_points1.getMat().rows;
    std::cout << "total points = " << total_points << "\n\n";

    cv::Mat points1 = cv::Mat(total_points, 2, CV_32F, input_points1.getMat().data);
    cv::Mat points2 = cv::Mat(total_points, 2, CV_32F, input_points2.getMat().data);

    std::cout << "total points = " << total_points << "\n\n";


    int * init_point = new int[1];
    sampler->getSample(init_point, 1, total_points);

    cv::Point_<float> initial_point = points1.at<cv::Point_<float>>(init_point[0]);
    cv::Point_<float> corr_initial_point = points2.at<cv::Point_<float>>(init_point[0]);

    /*
    cv::Mat_<float> H = cv::findHomography(points1, points2);
    cv::Mat initial_point3d (1, 3, CV_32FC1);
    initial_point3d.at<float>(0,0) = initial_point.x;
    initial_point3d.at<float>(0,1) = initial_point.y;
    initial_point3d.at<float>(0,2) = 1;
    cv::Mat_<float> corr_initial_point3d = initial_point3d * H;
    corr_initial_point3d.at<float>(0,2) = 1;
    cv::Point_<float> corr_initial_point;
    corr_initial_point.x = corr_initial_point3d.at<float>(0,0);
    corr_initial_point.y = corr_initial_point3d.at<float>(0,1);
*/
    std::cout << "initial_point = " << initial_point << "\n\n";
    std::cout << "corr_initial_point = " << corr_initial_point << "\n\n";

    cv::Mat query, dists, indicies1, indicies2;
    cv::flann::LinearIndexParams flannIndexParams;

    cv::flann::Index flannIndex1 (cv::Mat(points1).reshape(1), flannIndexParams);
    query = cv::Mat(1, 2, CV_32F, &initial_point);
    flannIndex1.knnSearch(query, indicies1, dists, knn);
    std::cout << "indicies1 = " << indicies1 << '\n';

    cv::flann::Index flannIndex2 (cv::Mat(points1).reshape(1), flannIndexParams);
    query = cv::Mat (1, 2, CV_32F, &corr_initial_point);
    flannIndex2.knnSearch(query, indicies2, dists, knn);
    std::cout << "indicies2 = " << indicies2 << '\n';


    // Our algorithm begins by computing the distributions for correct and incorrect matches for
    // the data provided. In order to start the process, we need a correct-match predictor to
    // preliminarily label each match as correct or incorrect (e.g., Lowe's ratio [11] or MR-Rayleigh [8]).
    std::vector<std::vector<cv::DMatch>> matches;
    cv::BFMatcher matcher;

//    matcher.knnMatch(points1, points2, matches, 2);  // Find two nearest matches
    matcher.knnMatch(points1, points2, matches, 2);  // Find two nearest matches

    std::vector<cv::DMatch> good_matches;
    std::vector<cv::DMatch> match1;
    std::vector<cv::DMatch> match2;
    for (int i = 0; i < matches.size(); ++i) {
        match1.push_back(matches[i][0]);
        match2.push_back(matches[i][1]);

        const float ratio = 0.8; // As in Lowe's paper; can be tuned
        if (matches[i][0].distance < ratio * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }


    // Then fit a two-parameter Gamma distribution to the data identified as correct to estimate Fc.
    std::gamma_distribution();


    // helper
    Drawing draw;
    cv::Mat image1 = cv::imread("../images/img1.png");
    cv::Mat image2 = cv::imread("../images/img2.png");

    cv::Mat img_matches1 = cv::imread("../images/img1.png");
    cv::Mat img_matches2 = cv::imread("../images/img2.png");

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    for (int i = 0; i < total_points; i++) {
        keypoints1.push_back(cv::KeyPoint(points1.at<float>(i, 0), points1.at<float>(i, 1), 1));
        keypoints2.push_back(cv::KeyPoint(points2.at<float>(i, 0), points1.at<float>(i, 1), 1));
    }

    cv::drawMatches(image1, keypoints1, image2, keypoints2, match1, img_matches1);
    cv::drawMatches(image1, keypoints1, image2, keypoints2, match2, img_matches2);

    imshow("matches1", img_matches1);
    imshow("matches1", img_matches2);

    std::vector<int> v_inds1, v_inds2;
    for (int i = 0; i < knn; i++) {
        v_inds1.push_back(indicies1.at<int>(i));
        v_inds2.push_back(indicies2.at<int>(i));
    }
    draw.showInliers(input_points1, v_inds1, image2);
    draw.showInliers(input_points2, v_inds2, image1);

    circle(image2, initial_point, 3, cv::Scalar(255, 0, 0), -1);
    circle(image1, corr_initial_point, 3, cv::Scalar(255, 0, 0), -1);

    imshow("knn1", image1);
    imshow("knn2", image2);
    cv::waitKey (0);
    // end helper
}