#include "NaiveRansac.h"

Line NaiveRansac::getBestLineFit () {

	auto begin_time = std::chrono::steady_clock::now();

	float iters = 0;
	float max_iters = model->max_iterations;

	float inlier_points = 2;
	float quantity_in_polygon;

	float dist;

	std::pair<cv::Point2f, cv::Point2f> random_points;
	cv::Point2f p1, p2;

	while (iters < max_iters) {
		random_points = sampler->getRandomTwoPoints();
		p1 = random_points.first;
		p2 = random_points.second;

		quantity_in_polygon = 0;

		// useful for big data
		// #pragma omp parallel for reduction (+:quantity_in_polygon)
		for (int kp = 0; kp < total_points; kp++) {
			dist = abs((p2.y-p1.y)*points[kp].x - (p2.x-p1.x)*points[kp].y + p2.x*p1.y - p2.y*p1.x)/sqrt(pow(p2.y-p1.y,2)+pow(p2.x-p1.x,2));

			if (dist < model->threshold) {
				quantity_in_polygon++;
			}
		}
		
		if (inlier_points < quantity_in_polygon) {
			inlier_points = quantity_in_polygon;
			best_line.p1 = p1;
			best_line.p2 = p2;
			max_iters = termination_criteria->getUpBoundIterations(inlier_points, total_points);
			quality->points_under_threshold = inlier_points;
		}
		iters++;
	}

	quality->total_iterations = iters;
	auto end_time = std::chrono::steady_clock::now();
	std::chrono::duration<float> fs = end_time - begin_time;
	quality->total_time = std::chrono::duration_cast<std::chrono::milliseconds>(fs);	
	return best_line;
}	


bool fit_point_between_lines (float x, float y, float k, float b1, float b2) {
	if ((x >= k*y + b1) && (x <= k*y + b2))  {
		return true;
	}
	return false;
}
