#include "Evsac.h"
#include "Helper/Drawing.h"

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

    cv::flann::Index flannIndex2 (cv::Mat(points1).reshape(1), flannIndexParams);
    query = cv::Mat (1, 2, CV_32F, &corr_initial_point);
    flannIndex2.knnSearch(query, indicies2, dists, knn);


    std::vector<std::vector<cv::DMatch>> matches;
    cv::BFMatcher matcher;
//    matcher.knn
//    matcher.knnMatch(descriptors_1, descriptors_2, matches, 2);  // Find two nearest matches
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < matches.size(); ++i)
    {
        const float ratio = 0.8; // As in Lowe's paper; can be tuned
        if (matches[i][0].distance < ratio * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }

    // helper
    cv::Mat image1 = cv::imread("../images/img1.png");
    cv::Mat image2 = cv::imread("../images/img2.png");

    std::vector<int> v_inds1, v_inds2;
    for (int i = 0; i < knn; i++) {
        v_inds1.push_back(indicies1.at<int>(i));
        v_inds2.push_back(indicies2.at<int>(i));
    }
    Drawing draw;
    draw.showInliers(input_points1, v_inds1, image2);
    draw.showInliers(input_points2, v_inds2, image1);

    circle(image2, initial_point, 3, cv::Scalar(255, 0, 0), -1);
    circle(image1, corr_initial_point, 3, cv::Scalar(255, 0, 0), -1);

    imshow("knn1", image1);
    imshow("knn2", image2);
    cv::waitKey (0);
    // end helper
}