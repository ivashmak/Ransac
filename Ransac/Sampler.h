#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

class Sampler {
public:
	int total_points;
	cv::Point2f *points;
public:
	Sampler (cv::InputArray points) {
		CV_Assert(!points.empty());
	
		srand (time(NULL));
		
		this->total_points = points.size().width;
		this->points = (cv::Point2f *) points.getMat().data;
	}

	std::pair<cv::Point2f, cv::Point2f> getRandomTwoPoints () {
		int r1 = rand() % total_points;
		int r2 = rand() % total_points;

		// if (r1 == r2) return getRandomTwoPoints();

		return std::make_pair(cv::Point2f(points[r1].x, points[r1].y), cv::Point2f(points[r2].x, points[r2].y));
	}
};

#endif //RANSAC_SAMPLER_H