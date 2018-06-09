#include "Line2DEstimator.h"

void Line2DEstimator::EstimateModel(cv::InputArray input_points, cv::OutputArray &line, int *sample, int sample_number, Model &model) {
	
	points = (cv::Point_<float> *) input_points.getMat().data;
	
	std::vector<cv::Point_<float>> best_line(SampleNumber());

	auto begin_time = std::chrono::steady_clock::now();

	float iters = 0;
	float max_iters = model.max_iterations;

	float inlier_points = 2;
	float quantity_in_polygon;

	float dist;

	while (iters < max_iters) {

		std::vector<cv::Point_<float>> random_points;	
		for (int rp = 0; rp < SampleNumber(); rp++) {
			cv::Point_<float> pt = sampler->getRandomPoint();
			random_points.push_back(pt);
		}

		quantity_in_polygon = 0;

		// useful for big data
		// #pragma omp parallel for reduction (+:quantity_in_polygon)
		for (int kp = 0; kp < total_points; kp++) {
			dist = GetError2(random_points, kp);
			
			if (dist < model.threshold) {
				quantity_in_polygon++;
			}
		}
		
		if (inlier_points < quantity_in_polygon) {
			inlier_points = quantity_in_polygon;

			cv::OutputArray best_line(random_points);
			best_line.copyTo(line);

			max_iters = termination_criteria->getUpBoundIterations(inlier_points, total_points);
			quality->points_under_threshold = inlier_points;
		}
		iters++;
	}

	quality->total_iterations = iters;
	auto end_time = std::chrono::steady_clock::now();
	std::chrono::duration<float> fs = end_time - begin_time;
	quality->total_time = std::chrono::duration_cast<std::chrono::milliseconds>(fs);	

}	


bool fit_point_between_lines (float x, float y, float k, float b1, float b2) {
	if ((x >= k*y + b1) && (x <= k*y + b2))  {
		return true;
	}
	return false;
}
