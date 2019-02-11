#include "detector.h"
#include "Reader.h"
#include "../dataset/Dataset.h"
#include "../usac/estimator/estimator.hpp"
#include "../usac/estimator/fundamental_estimator.hpp"
#include "../usac/utils/utils.hpp"
#include "../usac/estimator/essential_estimator.hpp"
#include "../usac/quality/quality.hpp"

void DetectFeatures(const std::string &name, const cv::Mat &image1, const cv::Mat &image2, cv::Mat &points)
{

    if (Reader::LoadPointsFromFile(points, name.c_str())) {
        printf("Match number: %d\n", points.rows);
        return;
    }

//    printf("Detect SIFT features\n");
//    std::cout << "Detect SURF features\n";
    cv::Mat descriptors1, descriptors2;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

    cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
//    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();

    detector->detect(image1, keypoints1);
    detector->compute(image1, keypoints1, descriptors1);
    printf("Features found in the first image: %d\n", static_cast<int>(keypoints1.size()));

    detector->detect(image2, keypoints2);
    detector->compute(image2, keypoints2, descriptors2);
    printf("Features found in the second image: %d\n", static_cast<int>(keypoints2.size()));

    std::vector<std::vector< cv::DMatch >> matches_vector;
    cv::FlannBasedMatcher matcher(new cv::flann::KDTreeIndexParams(5), new cv::flann::SearchParams(32));
    matcher.knnMatch(descriptors1, descriptors2, matches_vector, 2);

    std::vector<cv::Point2f> src_points, dst_points;
    std::vector<std::vector< cv::DMatch >> good_matches_vector;

    std::ofstream save_score;
    std::string img_name = name.substr (0, name.length()-7); // name = dir/*_pts.txt
    save_score.open(img_name+"score.txt");

    for (auto m : matches_vector)
    {
        if (m.size() == 2 && m[0].distance < m[1].distance * 0.7) // 0.8
        {
            auto& kp1 = keypoints1[m[0].queryIdx];
            auto& kp2 = keypoints2[m[0].trainIdx];
            src_points.push_back(kp1.pt);
            dst_points.push_back(kp2.pt);
            good_matches_vector.push_back(m);
            // save score
            save_score << (m[0].distance / m[1].distance) << "\n";
        }
    }
    save_score.close();

    points = cv::Mat(static_cast<int>(src_points.size()), 4, CV_32F);
    float *points_ptr = reinterpret_cast<float*>(points.data);

    for (int i = 0; i < src_points.size(); ++i)
    {
        *(points_ptr++) = src_points[i].x;
        *(points_ptr++) = src_points[i].y;
        *(points_ptr++) = dst_points[i].x;
        *(points_ptr++) = dst_points[i].y;
    }


    Reader::SavePointsToFile(points, name.c_str(), NULL);
    printf("Match number: %d\n", static_cast<int>(dst_points.size()));


    // ---------------------------------------------------------------------------------------------

    // sort points by ratio of distances in ascending order.
    std::sort (good_matches_vector.begin(), good_matches_vector.end(), [&] (const std::vector<cv::DMatch>& m1, const std::vector<cv::DMatch>& m2) {
        return (m1[0].distance / (m1[1].distance)) < (m2[0].distance / (m2[1].distance));
    });


    std::vector<cv::Point2f> src_points_sort, dst_points_sort;
    for (auto m : good_matches_vector) {
        auto& kp1 = keypoints1[m[0].queryIdx];
        auto& kp2 = keypoints2[m[0].trainIdx];
        src_points_sort.push_back(kp1.pt);
        dst_points_sort.push_back(kp2.pt);
    }
    cv::Mat sorted_points = cv::Mat(static_cast<int>(src_points_sort.size()), 4, CV_32F);
    float *sorted_points_ptr = reinterpret_cast<float*>(sorted_points.data);

    for (int i = 0; i < src_points.size(); ++i) {
        *(sorted_points_ptr++) = src_points_sort[i].x;
        *(sorted_points_ptr++) = src_points_sort[i].y;
        *(sorted_points_ptr++) = dst_points_sort[i].x;
        *(sorted_points_ptr++) = dst_points_sort[i].y;
    }
    std::string sort_points_name = img_name+"spts.txt";
    Reader::SavePointsToFile(sorted_points, sort_points_name.c_str(), NULL);
}


/*
 * Get points from image with name "filename".
 * Using different key points detection.
 *
 * Example of usage
 * std::vector<cv::KeyPoint> points = detect("data/image1.jpg", "sift");
 *
 */
std::vector<cv::KeyPoint> detect(std::string filename, std::string detector_name) {
	cv::Mat img = imread (filename, cv::IMREAD_GRAYSCALE);
	
	if (!img.data) {
		std::cout<< "Error in read\n";
		exit (1);
	}

	std::vector<cv::KeyPoint> keypoints;
	
	if (detector_name.compare("surf") == 0) {
		cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create ();
		detector->detect (img, keypoints);

	} else if (detector_name.compare("orb") == 0) {
		cv::Ptr<cv::ORB> detector = cv::ORB::create ();
		detector->detect (img, keypoints);

	} else if (detector_name.compare("sift") == 0) {
		cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create ();
		detector->detect (img, keypoints);

	} else if (detector_name.compare("corner") == 0) {
		cv::Mat src_gray;
		cv::Mat src = cv::imread(filename, 1 );
  		cv::cvtColor( src, src_gray, cv::COLOR_BGR2GRAY);
	  	cv::Mat dst, dst_norm, dst_norm_scaled;
	  	dst = cv::Mat::zeros( src.size(), CV_32FC1 );
		
		int blockSize = 2;
	  	int apertureSize = 3;
	  	double k = 0.3;
		cornerHarris( src_gray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );

	    normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
     	convertScaleAbs( dst_norm, dst_norm_scaled );
	  	for (int j = 0; j < dst_norm.rows; j++) { 
	     	for( int i = 0; i < dst_norm.cols; i++ ) {
	            if ((int) dst_norm.at<float>(j,i) <= 43 ) {// thresh
	               	cv::KeyPoint kp (cv::Point2f(i,j), -1, 0, 0, -1);
	               	keypoints.push_back(kp);
	            }
	        }
	    }
	    
	} else {
		std::cout << "Not found detector\n";
		exit (1);
	}

	cv::Mat img_keypoints;
	drawKeypoints (img, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  	imwrite("res/points_detection/"+detector_name+".jpg", img_keypoints);
  	// imshow (detector_name, img_keypoints);	
  	// cv::waitKey(0);
	
	return keypoints;
}


bool detectFeaturesForEssentialMatrix (const cv::Mat &P1, const cv::Mat &P2, const std::string &img1_name,
                                       const std::string &img2_name, const std::string &out_name, float threshold) {
    cv::Mat K1, R1, t1;
    cv::decomposeProjectionMatrix(P1, K1, R1, t1);
    cv::Mat K2, R2, t2;
    cv::decomposeProjectionMatrix(P2, K2, R2, t2);

    cv::Mat F_gt;
    FundamentalEstimator::getFundamentalFromProjection(P1, P2, F_gt);

    cv::Mat image1 = cv::imread(img1_name);
    cv::Mat image2 = cv::imread(img2_name);
    if (image1.empty() || image2.empty()) {
        std::cout << "wrong direction to images for essential dataset\n";
        exit (0);
    }
    cv::Mat points;
    // Detect and match features
    DetectFeatures(out_name+"_pts.txt", image1, image2, points);

    std::vector<int> gt_inliers;
    Estimator * fundamental = new FundamentalEstimator (points);
    Quality::getInliers(fundamental, F_gt, threshold, points.rows, gt_inliers);

    if (gt_inliers.size() < 20) {
        std::remove((out_name+"_pts.txt").c_str());
        std::remove((out_name+"_spts.txt").c_str());
        std::remove((out_name+"_score.txt").c_str());
        std::cout << "not enough inliers\n";
        return false;
    }

    std::ofstream save_inliers;
    save_inliers.open(out_name+"_inl.txt");
    save_inliers << gt_inliers.size() << "\n";
    for (const int &inl : gt_inliers) {
        save_inliers << inl << "\n";
    }

    return true;
}