#include "NaiveRansac.h"

Line NaiveRansac::getBestLineFit (std::vector<cv::KeyPoint> keypoints) {
	auto begin_time = std::chrono::steady_clock::now();
	
	threshold = 10; // parent varible
	
	Line best_line;

	float x1, y1, x2, y2, k, b;
	int r1, r2;
	
	float quantity_in_polygon;
	float total_points = (float) keypoints.size();
	float inlier_points = 2;
	
	float desired_p = 0.99;

	float iter = 0;
	float number_of_points_in_sample = 2;
	float max_iters = log(1-desired_p)/log(1-pow(inlier_points/total_points, number_of_points_in_sample));
		
	while (iter < max_iters) {		
		r1 = rand() % keypoints.size();
		r2 = rand() % keypoints.size();

		x1 = keypoints[r1].pt.x;
		y1 = keypoints[r1].pt.y;

		x2 = keypoints[r2].pt.x;
		y2 = keypoints[r2].pt.y;
		
		k = (x2-x1)/(y2-y1);
		b = (y1*x1- y1*x2)/(y2-y1) + x1;

		quantity_in_polygon = 0.0;

		#pragma omp parallel for reduction (+:quantity_in_polygon)
		for (int kp = 0; kp < keypoints.size(); kp++) {
			if (fit_point(keypoints[kp].pt.x, keypoints[kp].pt.y, k, b-threshold, b+threshold)) {
				// fit_points.push_back(cv::KeyPoint(keypoints[kp].pt.x, keypoints[kp].pt.y, -1, 0, 0, -1));
				quantity_in_polygon++;
			}
		}
		
		if (inlier_points < quantity_in_polygon) {
			inlier_points = quantity_in_polygon;
			best_line.x1 = x1;
			best_line.x2 = x2;
			best_line.y1 = y1;
			best_line.y2 = y2;	 

			max_iters = log(1-desired_p)/log(1-pow(inlier_points/total_points, number_of_points_in_sample));
		}
		iter++;

		total_iterations++;  // parent varible
	}

	auto end_time = std::chrono::steady_clock::now();
	fsec fs = end_time - begin_time;
	total_time = std::chrono::duration_cast<ms>(fs);

	return best_line;
}	


void NaiveRansac::showResult (cv::Mat image) {
	int width = image.cols;
	int height = image.rows;

	float k = (best_line.x2 - best_line.x1)/(best_line.y2 - best_line.y1);
	float b = (best_line.y1*best_line.x1 - best_line.y1*best_line.x2)/(best_line.y2-best_line.y1) + best_line.x1;

	image = cv::imread("data/image1.jpg");
			
	draw_function (k, b, std::max(width, height), cv::Scalar(255,0,0), image);
	imshow("Best Line",image);
	cv::waitKey (0);
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