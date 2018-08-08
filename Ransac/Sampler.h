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
 
    void getRandomPoints (cv::OutputArray &rpoints, int npoints) {
        int point;
        std::vector<cv::Point_<float>> vpoints;
        for (int i = 0; i < npoints; i++) {
            point = rand() % total_points;
            vpoints.push_back(cv::Point_<float>(points[point].x, points[point].y));
        }
        cv::OutputArray temp (vpoints);
        temp.copyTo(rpoints);
    }

};

#endif //RANSAC_SAMPLER_H