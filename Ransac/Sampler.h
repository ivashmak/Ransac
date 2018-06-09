#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

class Sampler {
public:
	int total_points;
	cv::Mat mat_points;
	cv::Point_<float> *points;		
public:
	Sampler (cv::InputArray points) {
		CV_Assert(!points.empty());
		
		srand (time(NULL));
		
		this->total_points = points.size().width;
		this->mat_points = points.getMat();
		this->points = (cv::Point_<float> *) mat_points.data;
	}

	cv::Point_<float> getRandomPoint () {
		
		int p = rand() % total_points;
		return cv::Point_<float>(points[p].x, points[p].y);
	}

	std::pair<cv::Point_<float>, cv::Point_<float>> getRandomTwoPoints () {
		int r1 = rand() % total_points;
		int r2 = rand() % total_points;
		
		return std::make_pair(cv::Point_<float>(points[r1].x, points[r1].y), cv::Point_<float>(points[r2].x, points[r2].y));
	}
};

#endif //RANSAC_SAMPLER_H