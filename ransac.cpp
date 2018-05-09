#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

#include "Generator/generator.h"
#include "Detector/detector.h"

// using namespace cv;
// using namespace std;
// using namespace cv::xfeatures2d;

// struct Point {
// 	float x;
// 	float y;
// };
// Point *points = new Point[keypoints.size()];
	
struct Line{
	float x;
	float y;
};

void line_func (float x1, float x2, float y1, float y2) {
	float k = (x2-x1)/(y2-y1);
	float b = (x2*y1- x1*y2)/(x2-x1);
}


void draw_function (float k, float b, float max_dimen, cv::Scalar color, cv::Mat img);
bool fit_point (float x, float y, float k, float b1, float b2);

int main () {
	srand (time(NULL));

	generate();
	std::cout << "generated image\n";

	cv::Mat img = cv::imread("data/image1.jpg");
	int width = img.cols;
	int height = img.rows;

	std::vector<cv::KeyPoint> keypoints = detect("data/image1.jpg", "sift");
	std::cout << "read keypoints\n";

	int iter = 0;
	int max_iters = 3;
	float threshold = 50;
	Line best_line;
	int max_quantity_in_polygon = 0;

	while (iter < max_iters) {
		int r1 = rand() % keypoints.size();
		int r2 = rand() % keypoints.size();
		
		std::cout << r1 << " "<< r2 << '\n';

		float x1 = keypoints[r1].pt.x;
		float y1 = keypoints[r1].pt.y;

		float x2 = keypoints[r2].pt.x;
		float y2 = keypoints[r2].pt.y;
		
		int quantity_in_polygon = 0;

		float k = (x2-x1)/(y2-y1);
		float b = (y1*x1- y1*x2)/(y2-y1) + x1;

		std::vector<cv::Point> fit_points;
		
		for (int k = 0; k < keypoints.size(); k++) {
			if (fit_point(keypoints[k].pt.x, keypoints[k].pt.y, k, b-threshold, b+threshold)) {
				fit_points.push_back(cv::Point(keypoints[k].pt.y, keypoints[k].pt.x));
			}
		}

		
		img = cv::imread("data/image1.jpg");
		draw_function (k, b, std::max(width, height), cv::Scalar(255,0,0), img);
		draw_function (k, b+threshold, std::max(width, height), cv::Scalar(0,0,255), img);
		draw_function (k, b-threshold, std::max(width, height), cv::Scalar(0,0,255), img);		
		cv::line (img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(110, 220, 0), 2); //thickness
		

		// cv::Mat img_keypoints;
		// drawKeypoints (img, line_points, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  		
		// imshow("Image",img_keypoints);
		imshow("Image",img);
  		

  		cv::waitKey( 0 );

		iter++;	
	}

	// for (int i = 0; i < keypoints.size(); i++) {
	// 	std::cout << keypoints[i].pt.x<<" "<<keypoints[i].pt.y << '\n';
	// }	

	return 0;
}

void draw_function (float k, float b, float max_dimen, cv::Scalar color, cv::Mat img) {
	float corner_y1 = max_dimen;
	float corner_x1 = k*max_dimen+b;
	
	float corner_y2 = -max_dimen;
	float corner_x2 = k*(-max_dimen)+b;
	cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8 );
}

bool fit_point (float x, float y, float k, float b1, float b2) {
	// if () {

	// }
	return true;
}