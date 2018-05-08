#include "detector.h"

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
  		cv::cvtColor( src, src_gray, CV_BGR2GRAY );
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
  	// cv::waitKey(0);
	
	return keypoints;
}
