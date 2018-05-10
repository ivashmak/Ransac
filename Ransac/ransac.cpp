#include "ransac.h"

Line ransac (std::vector<cv::KeyPoint> keypoints) {
	int max_iters = 200;
	float threshold = 10;
	Line best_line;
	int max_quantity_in_polygon = 0;

	float x1, y1, x2, y2, k, b;
	int r1, r2, quantity_in_polygon;
	for (int iter = 0; iter < max_iters; iter++) {
		r1 = rand() % keypoints.size();
		r2 = rand() % keypoints.size();

		x1 = keypoints[r1].pt.x;
		y1 = keypoints[r1].pt.y;

		x2 = keypoints[r2].pt.x;
		y2 = keypoints[r2].pt.y;
		
		k = (x2-x1)/(y2-y1);
		b = (y1*x1- y1*x2)/(y2-y1) + x1;

		// std::vector<cv::KeyPoint> fit_points;
		quantity_in_polygon = 0;

		#pragma omp parallel for reduction (+:quantity_in_polygon)
		for (int kp = 0; kp < keypoints.size(); kp++) {
			if (fit_point(keypoints[kp].pt.x, keypoints[kp].pt.y, k, b-threshold, b+threshold)) {
				// fit_points.push_back(cv::KeyPoint(keypoints[kp].pt.x, keypoints[kp].pt.y, -1, 0, 0, -1));
				quantity_in_polygon++;
			}
		}
		
		if (max_quantity_in_polygon < quantity_in_polygon) {
			max_quantity_in_polygon = quantity_in_polygon;
			best_line.x1 = x1;
			best_line.x2 = x2;
			best_line.y1 = y1;
			best_line.y2 = y2;	 
		}

		// std::cout << "keypoints.size() "<< keypoints.size() << " vs fit_points.size() " << fit_points.size() << '\n';
		
		// img = cv::imread("data/image1.jpg");
		// draw_function (k, b, std::max(width, height), cv::Scalar(255,0,0), img);
		// draw_function (k, b+threshold, std::max(width, height), cv::Scalar(0,0,255), img);
		// draw_function (k, b-threshold, std::max(width, height), cv::Scalar(0,0,255), img);		
		// cv::line (img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(110, 220, 0), 2); //thickness
		
		// cv::Mat img_keypoints;
		// drawKeypoints (img, fit_points, img_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT);
  		
		// imshow("Image",img_keypoints);
		// imshow("Image",img);
  		
  		// cv::waitKey( 0 );
	}

	return best_line;
}	

bool fit_point (float x, float y, float k, float b1, float b2) {
	if ((x >= k*y + b1) && (x <= k*y + b2))  {
		return true;
	}
	return false;
}

void draw_function (float k, float b, float max_dimen, cv::Scalar color, cv::Mat img) {
	float corner_y1 = max_dimen;
	float corner_x1 = k*max_dimen+b;
	
	float corner_y2 = -max_dimen;
	float corner_x2 = k*(-max_dimen)+b;
	cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8 );
}

