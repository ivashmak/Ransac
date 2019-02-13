#ifndef RANSAC_DRAWING_H
#define RANSAC_DRAWING_H

#include "../../test/tests.h"
#include "../../dataset/GetImage.h"

class Tests;
class Drawing {
public:
    static void draw (Model * model, DATASET dataset, const std::string &img_name) {
        if (model->estimator == ESTIMATOR::Line2d) {
            draw_line(model, dataset, img_name);

        } else if (model->estimator == ESTIMATOR::Essential) {
            drawEpipolarLines(model, dataset, img_name);

        } else if (model->estimator == ESTIMATOR::Fundamental) {
            drawEpipolarLines(model, dataset, img_name);

        } else if (model->estimator == ESTIMATOR::Homography) {
            drawHomography(model, dataset, img_name);

        }
    }
    /*
     * w ~ original width;  h ~ original height; S original square;
     * constraint 1: new square is S' = w' * h' == 480000
     * constraint 2: ratio is the same
     *      S'
     * h' = -
     *      w'
     *
     * w'  w             wh'   wS'                  wS'
     * - = -   =>  w' = ---  = --   =>  w' = sqrt (----)
     * h'  h             h     hw'                  h
     *
     *                                              hS'
     *                                  h' = sqrt (----)
     *                                              w
     */
    static void drawing_resize (cv::Mat &image) {
        float nS = 900000; // 600 x 800
        cv::resize(image, image, cv::Size(sqrt ((image.cols * nS)/image.rows), sqrt ((image.rows * nS)/image.cols)));
    }

    /*
     * Show inliers for image
     * @input_points         all point
     * @input_inliers_idxes  indexes of inliers
     * @image                image
     */
    static void showInliers (cv::InputArray input_points, cv::InputArray input_inliers_idxes, cv::Mat &image) {
        int *inliers_idxes = (int *) input_inliers_idxes.getMat().data;
        cv::Point_<float> *points = (cv::Point_<float> *) input_points.getMat().data;

        int inliers_size = input_inliers_idxes.size().width;
        for (int i = 0; i < inliers_size; i++) {
            circle(image, points[inliers_idxes[i]], 3, cv::Scalar(20, 90, 250), -1);
        }
    }

    // x = ky + b
    static void draw_line_ky_b (float k, float b, const cv::Scalar &color, cv::Mat img) {
        int max_dimen = std::max (img.cols, img.rows);
        float corner_y1 = max_dimen;
        float corner_x1 = k*corner_y1 + b;

        float corner_y2 = -max_dimen;
        float corner_x2 = k*corner_y2 + b;
        cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
    }
    
    // y = kx + b
    static void draw_line_kx_b (float k, float b, const cv::Scalar &color, cv::Mat img) {
        int max_dimen = std::max (img.cols, img.rows);
        float corner_x1 = max_dimen;
        float corner_y1 = k*corner_x1 + b;

        float corner_x2 = -max_dimen;
        float corner_y2 = k*corner_x2 + b;
        cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
    }
    
    // ax + by + c = 0
    // y = (-ax - c)/b
    // x = (-by - c)/a 
    static void draw_line_abc (float a, float b, float c, cv::Scalar color, cv::Mat img) {
        int max_dimen = std::max (img.cols, img.rows);
        float corner_x1, corner_x2, corner_y1, corner_y2;
        if (b == 0) {
            corner_y1 = max_dimen;
            corner_y2 = -max_dimen;   
            
            corner_x1 = (-b*corner_y1 + c)/a;
            corner_x2 = (-b*corner_y2 + c)/a;
        } else {
            corner_x1 = max_dimen;
            corner_x2 = -max_dimen;
            
            corner_y1 = (-a*corner_x1 + c)/b;
            corner_y2 = (-a*corner_x2 + c)/b;
        }
        
        cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
    }

    static void draw_line_model (Model * const model, cv::Scalar color, cv::Mat img, bool threshold) {
        cv::Mat desc = model->returnDescriptor();
        auto * params = reinterpret_cast<float *>(desc.data);
        // // ax + by + c = 0
        // // y = kx + l, k = -a*x/b, l = -c/b
        float l = -params[2]/params[1]; 
        float k = -params[0]/params[1]; 

        draw_line_kx_b (k, l, color, img);
        
        if (threshold) {
            draw_line_kx_b (k, l+sqrt(pow(model->threshold,2)*pow(k,2)+pow(model->threshold,2)), cv::Scalar(0,0,255), img);
            draw_line_kx_b (k, l-sqrt(pow(model->threshold,2)*pow(k,2)+pow(model->threshold,2)), cv::Scalar(0,0,255), img);
        }
    }

    /*
     * Read image.
     * Show inliers of best ransac model.
     * Show inliers of non minimal best model.
     * To show threshold lines change false to true.
     */
    static void draw_line (Model * const model, DATASET dataset, const std::string &img_name) {
        ImageData imageData(dataset, img_name);
        cv::Mat points = imageData.getPoints();
        cv::Mat image = imageData.getImage1();
        Line2DEstimator est (points);
        std::vector<int> inliers;
        Quality::getInliers(&est, model->returnDescriptor(), model->threshold, points.rows, inliers);
        showInliers(points, inliers, image);
        draw_line_model(model, cv::Scalar(255, 0, 0), image, true);

        drawing_resize(image);
        imshow("Inliers", image);
//        std::string filename = "../results/linefitting_"+Tests::sampler2string(model->sampler)+".jpg";
        std::string filename = "../results/linefitting_sampler.jpg";
        cv::imwrite(filename, image);
        cv::waitKey (0);
    }

    /*
     * Draw epipolar lines by Fundamental Matrix
     */
    static void drawEpipolarLines (Model * model, DATASET dataset, const std::string& img_name);


    static void drawHomography (Model *model, DATASET dataset, const std::string& img_name) {
        ImageData gt_data (dataset, img_name);

        cv::Mat points1 = gt_data.getPoints1();
        cv::Mat points2 = gt_data.getPoints2();
        cv::hconcat(points1, cv::Mat_<float>::ones(points1.rows, 1), points1);
        cv::hconcat(points2, cv::Mat_<float>::ones(points1.rows, 1), points2);

        cv::Mat img1 = gt_data.getImage1();
        cv::Mat img2 = gt_data.getImage2();

        cv::Mat img1_inl = img1.clone();
        cv::Mat img2_inl = img2.clone();

        cv::Mat gt_img1_inl = img1.clone();
        cv::Mat gt_img2_inl = img2.clone();

        cv::Mat opencv_img1_inl = img1.clone();
        cv::Mat opencv_img2_inl = img2.clone();


        // draw matches
//        cv::Mat img_matches, gt_img_matches;
//        drawMatches(img_matches, img1, img2, points1, points2, inliers, inliers_size);
//        drawMatches(gt_img_matches, img1, img2, points1, points2, &gt_inliers[0], gt_inliers.size());
//        cv::vconcat(img_matches, gt_img_matches, img_matches);
        // ----------------------


        cv::Mat H_opencv = cv::Mat_<float>(cv::findHomography(points1, points2));
        cv::Mat_<float> H_gt = gt_data.getModel();

//        getMatrix3x3 ("../dataset/EVD/h/"+img_name+".txt", H_gt);

        drawErrors(img1_inl, img2_inl, points1, points2, model->returnDescriptor());

        drawErrors(gt_img1_inl, gt_img2_inl, points1, points2, H_gt.inv());
//        drawErrors(gt_img1_inl, gt_img2_inl, points1, points2, H_gt);

        drawErrors(opencv_img1_inl, opencv_img2_inl, points1, points2, H_opencv);

        cv::hconcat(img1_inl, img2_inl, img1_inl);
        cv::hconcat(gt_img1_inl, gt_img2_inl, gt_img1_inl);
        cv::hconcat(opencv_img1_inl, opencv_img2_inl, opencv_img1_inl);

        cv::vconcat(img1_inl, gt_img1_inl, img1_inl);
//        cv::vconcat(img1_inl, opencv_img1_inl, img1_inl);

//        cv::resize(img2_inl, img2_inl, cv::Size (0.75 * img2_inl.cols, 0.5 * img2_inl.rows));
        drawing_resize(img1_inl);
        cv::imshow ("estimated points on correspondence images vs grand truth estimated points vs opencv", img1_inl);
//        cv::imwrite("../results/homography/"+img_name+".png", img1_inl);

        // draw panorama
        cv::Mat panorama_opencv, panorama_gt, panorama;
        std::vector<cv::Mat> images;
        images.push_back(img1); images.push_back(img2);
        drawPanorama(images, panorama, model->returnDescriptor().inv());
        drawPanorama(images, panorama_gt, H_gt);
        drawPanorama(images, panorama_opencv, H_opencv.inv());

//        drawing_resize(img_matches);

        cv::vconcat(panorama, panorama_gt, panorama);
//        cv::vconcat(panorama, panorama_opencv, panorama);

        drawing_resize(panorama);
//        cv::imshow("panorama", panorama);
        // -------------------------------------

//        cv::imshow("imgs ", img_matches);

        cv::waitKey(0);
    }

    static void drawPanorama (const std::vector<cv::Mat>& imgs, cv::Mat& panorama, const cv::Mat& H);

    static void drawMatches (cv::Mat& img_matches, const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& points1, const cv::Mat& points2, const int * const inliers, int inliers_size) {
        std::vector< cv::DMatch > good_matches;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        mat2keypoint(points1, keypoints1);
        mat2keypoint(points2, keypoints2);

        for (int i = 0; i < inliers_size; i++) {
            cv::DMatch match (inliers[i], inliers[i], 0);
            good_matches.push_back(match);
        }

        cv::drawMatches (img1, keypoints1, img2, keypoints2,
                         good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                         std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    }

    static void drawErrors (cv::Mat &img1_error, cv::Mat &img2_error, const cv::Mat& points1, const cv::Mat& points2, const cv::Mat& H) {
        cv::Mat points1_t, points2_t;
        cv::transpose(points1, points1_t);
        cv::transpose(points2, points2_t);

        cv::Mat est_pts_on_corr2 = H * points1_t;
        cv::Mat est_pts_on_corr1 = H.inv() * points2_t;

        cv::divide(est_pts_on_corr1.row(0), est_pts_on_corr1.row(2), est_pts_on_corr1.row(0), 1);
        cv::divide(est_pts_on_corr1.row(1), est_pts_on_corr1.row(2), est_pts_on_corr1.row(1), 1);
        cv::divide(est_pts_on_corr2.row(0), est_pts_on_corr2.row(2), est_pts_on_corr2.row(0), 1);
        cv::divide(est_pts_on_corr2.row(1), est_pts_on_corr2.row(2), est_pts_on_corr2.row(1), 1);

        for (int i = 0; i < points1.rows; i++) {
            cv::Scalar color = cv::Scalar(random()%255, random ()%255, random()%255);

            cv::line(img1_error, cv::Point_<float> (points1.at<float>(i, 0), points1.at<float>(i, 1)),
                                 cv::Point_<float> (est_pts_on_corr1.at<float>(0, i), est_pts_on_corr1.at<float>(1, i)), color, 2);

            cv::line(img2_error, cv::Point_<float> (points2.at<float>(i, 0), points2.at<float>(i, 1)),
                                 cv::Point_<float> (est_pts_on_corr2.at<float>(0, i), est_pts_on_corr2.at<float>(1, i)), color, 2);

            cv::circle (img1_error, cv::Point_<float>(points1.at<float>(i, 0), points1.at<float>(i, 1)), 3, cv::Scalar(0, 255, 0), -1);
            cv::circle (img2_error, cv::Point_<float>(points2.at<float>(i, 0), points2.at<float>(i, 1)), 3, cv::Scalar(0, 255, 0), -1);

            cv::circle (img1_error, cv::Point_<float>(est_pts_on_corr1.at<float>(0, i), est_pts_on_corr1.at<float>(1, i)), 3, cv::Scalar(0, 0, 255), -1);
            cv::circle (img2_error, cv::Point_<float>(est_pts_on_corr2.at<float>(0, i), est_pts_on_corr2.at<float>(1, i)), 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    static void mat2keypoint(const cv::Mat& points, std::vector<cv::KeyPoint>& keypoints) {
        for (int i = 0; i < points.rows; i++) {
            cv::KeyPoint kp(points.at<float>(i,0), points.at<float>(i,1), 1);
            keypoints.push_back(kp);
        }
    }

    static void mat2point (const cv::Mat& points, std::vector<cv::Point_<float>>& vpoints) {
        for (int i = 0; i < points.rows; i++) {
            cv::Point_<float> p (points.at<float>(i,0), points.at<float>(i,1));
            vpoints.push_back(p);
        }
    }

};


#endif //RANSAC_DRAWING_H
