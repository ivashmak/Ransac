#ifndef RANSAC_QUALITY_H
#define RANSAC_QUALITY_H

#include "Line.h"
#include "Model.h"

class Quality {
public:
    int total_iterations = 0;
	std::chrono::milliseconds total_time;
	int points_under_threshold = 0;
public:
	int getComputationTime (void) {
		return total_time.count();
    }

	int getIterations (void) {
		return total_iterations;
	}

	int getNumberOfPointsUnderThreshold () {
		return points_under_threshold;
	}


	void showResult (Model& model, cv::InputArray ps, cv::InputArray line) {
		cv::Point_<float> *lpoints = (cv::Point_<float> *) line.getMat().data;
		cv::Point_<float> p1 = lpoints[0];
		cv::Point_<float> p2 = lpoints[1];

		cv::Mat image = cv::imread("data/image1.jpg");

		CV_Assert(image.depth() == CV_8U);
		CV_Assert(!ps.empty());

		int width = image.cols;
		int height = image.rows;

		float k = (p2.x - p1.x)/(p2.y - p1.y);
		float b = (p1.y*p1.x - p1.y*p2.x)/(p2.y-p1.y) + p1.x;

		draw_function (k, b-sqrt(pow(model.threshold,2)*pow(k,2)+pow(model.threshold,2)), std::max(width, height), cv::Scalar(0,255,0), image);
		draw_function (k, b, std::max(width, height), cv::Scalar(255,0,0), image);
		draw_function (k, b+sqrt(pow(model.threshold,2)*pow(k,2)+pow(model.threshold,2)), std::max(width, height), cv::Scalar(0,255,0), image);
		
		cv::Point_<float> *points = (cv::Point_<float> *) ps.getMat().data;
		int total_points = ps.size().width;
		float dist;

		for (int kp = 0; kp < total_points; kp++) {
			dist = abs((p2.y-p1.y)*points[kp].x - 
				(p2.x-p1.x)*points[kp].y + 
				p2.x*p1.y - p2.y*p1.x)/sqrt(pow(p2.y-p1.y,2)+pow(p2.x-p1.x,2));
			
			if (dist < model.threshold) {
		        circle(image, points[kp], 3, cv::Scalar(0, 0, 255), -1);
			}
		}

		imshow("Best Line", image);
		cv::waitKey (0);
	}

	void draw_function (float k, float b, float max_dimen, cv::Scalar color, cv::Mat img) {
		float corner_y1 = max_dimen;
		float corner_x1 = k*max_dimen+b;
		
		float corner_y2 = -max_dimen;
		float corner_x2 = k*(-max_dimen)+b;
		cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
	}

};




#endif //RANSAC_QUALITY_H