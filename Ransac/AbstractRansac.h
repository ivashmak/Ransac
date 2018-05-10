#ifndef RANSAC_ABSTRACT_RANSAC_H
#define RANSAC_ABSTRACT_RANSAC_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>


struct Line{
    float x1;
    float y1;
    float x2;
    float y2;   
};

typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

class Ransac {
    public:
    	cv::Mat image;
  		std::vector<cv::KeyPoint> keypoints;
  		float threshold;
  		Line best_line;
  		int total_iterations = 0;
  		ms total_time;

  	public:

    	// Ransac ();
    	// Ransac (std::vector<cv::KeyPoint> keypoints) {
    	// 	this->keypoints = keypoints;
     //    }

    	// Ransac (cv::Mat image, std::vector<cv::KeyPoint> keypoints) {
    	// 	this->image = image;
    	// 	this->keypoints = keypoints;
    	// }
    	
     //    Ransac (cv::Mat image, std::vector<cv::KeyPoint> keypoints, float threshold) {
     //        this->image = image;
     //        this->keypoints = keypoints;
     //        this->threshold = threshold;
     //    }
    	
     //    ~Ransac ();
    	
    	void setThreshold (float threshold) {
    		this->threshold = threshold;
    	} 

    	void setKeypoints (std::vector<cv::KeyPoint> keypoints) {
    		this->keypoints = keypoints;
    	}

    	// virtual Line getBestLineFit (void) = 0; // abstract functions
  		virtual Line getBestLineFit (std::vector<cv::KeyPoint> keypoints) = 0;

  		int getComputationTime (void) {
  			return total_time.count();
        }

  		int getIterations (void) {
  			return total_iterations;
  		}

  		// virtual void showResult (void) = 0;
  		virtual void showResult (cv::Mat image) = 0;

        
        void exitOnFail(void) {
            std::cout << "Something wrong in Ransac!\n";
        }

        void exitOnFail(std::string message) {
            std::cout << message + '\n';
        }

        void exitOnSuccess(void) {
            std::cout << "Ransac finished successfuly!\n";
        }

        void exitOnSuccess(std::string message) {
            std::cout << message + '\n';
        }

  		// virtual void saveOutput (void) = 0;
};

#endif //RANSAC_ABSTRACT_RANSAC_H