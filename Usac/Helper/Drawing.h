#ifndef RANSAC_DRAWING_H
#define RANSAC_DRAWING_H

#include "../Estimator/Estimator.h"
#include "../Ransac/Ransac.h"
#include "../../Detector/ReadPoints.h"

class Drawing {
public:

    void showInliers (cv::InputArray input_points, cv::InputArray input_inliers_idxes, cv::Mat image) {
        int *inliers_idxes = (int *) input_inliers_idxes.getMat().data;
        cv::Point_<float> *points = (cv::Point_<float> *) input_points.getMat().data;

        int inliers_size = input_inliers_idxes.size().width;
        for (int i = 0; i < inliers_size; i++) {
            circle(image, points[inliers_idxes[i]], 3, cv::Scalar(20, 90, 250), -1);
        }
    }

    void draw_line (float k, float b, cv::Scalar color, cv::Mat img) {
        int max_dimen = std::max (img.cols, img.rows);
        float corner_y1 = max_dimen;
        float corner_x1 = k*max_dimen + b;

        float corner_y2 = -max_dimen;
        float corner_x2 = k*(-max_dimen) + b;
        cv::line (img, cv::Point(corner_x1, corner_y1), cv::Point(corner_x2, corner_y2), color,  2, 8);
    }

    void draw_model (Model * const model, float max_dimen, cv::Scalar color, cv::Mat img, bool threshold) {
        cv::Mat desc;
        model->getDescriptor(desc);
        auto * params = reinterpret_cast<float *>(desc.data);
        std::cout <<"model: a = "<< params[0] << " b = " << params[1] << " c = " <<params[2] << '\n';
        float b = -params[2]/params[0];
        float k = -params[1]/params[0];

        draw_line(k, b, color, img);

        if (threshold) {
            draw_line (k, b+sqrt(pow(model->threshold,2)*pow(k,2)+pow(model->threshold,2)), cv::Scalar(0,0,255), img);
            draw_line (k, b-sqrt(pow(model->threshold,2)*pow(k,2)+pow(model->threshold,2)), cv::Scalar(0,0,255), img);
        }

    }

    /*
     * Read image.
     * Show inliers of best ransac model.
     * Show inliers of non minimal best model.
     * To show threshold lines change false to true.
     */
    void draw (cv::InputArray inliers, Model * const best_model, Model * const non_minimal_model, cv::InputArray points) {
        cv::Mat image  = cv::imread("../images/image1.jpg");
        showInliers(points, inliers, image);
        draw_model(best_model, std::max (image.cols, image.rows), cv::Scalar(255, 0, 0), image, false);
        draw_model(non_minimal_model, std::max (image.cols, image.rows), cv::Scalar(0, 255, 0), image, false);
        imshow("Inliers", image);
        std::string filename = "../res/linefitting_"+best_model->model_name+".jpg";
        cv::imwrite(filename, image);
        cv::waitKey (0);
    }

    // DLT

    /*
     * Draw epipolar lines by Fundamental Matrix
     */
    void drawEpipolarLines (cv::InputArray points1, cv::InputArray points2, const cv::Mat& F) {

        cv::Mat img1 = cv::imread("../images/img1.png"), img2 = cv::imread("../images/img2.png");

        cv::Mat pts1 = points1.getMat(), pts2 = points2.getMat();
        cv::hconcat(pts1, cv::Mat_<float>::ones (pts1.rows, 1), pts1);
        cv::hconcat(pts2, cv::Mat_<float>::ones (pts2.rows, 1), pts2);

        cv::Mat lines1, lines2;
        cv::computeCorrespondEpilines(pts1, 1, F, lines1);
        cv::computeCorrespondEpilines(pts2, 2, F, lines2);

        int c = img1.cols, r = img1.rows;
        float x0, y0_img1, x1, y1_img1, r0_img1, r1_img1, r2_img1;
        float  y0_img2, y1_img2, r0_img2, r1_img2, r2_img2;
        for (int i = 0; i < points1.rows(); i++) {
            r0_img1 = lines1.at<float>(i, 0);
            r1_img1 = lines1.at<float>(i, 1);
            r2_img1 = lines1.at<float>(i, 2);

            r0_img2 = lines2.at<float>(i, 0);
            r1_img2 = lines2.at<float>(i, 1);
            r2_img2 = lines2.at<float>(i, 2);

            x0 = 0;
            y0_img1 = -r2_img1/r1_img1;
            y0_img2 = -r2_img2/r1_img2;
            x1 = c;
            y1_img1 = -(r2_img1 + r0_img1*c)/r1_img1;
            y1_img2 = -(r2_img2 + r0_img2*c)/r1_img2;

            cv::Scalar color = cv::Scalar(rand()%255, rand ()%255, rand()%255);
            cv::line(img1, cv::Point_<float> (x0, y0_img1), cv::Point_<float> (x1, y1_img1), color);
            cv::line(img2, cv::Point_<float> (x0, y0_img2), cv::Point_<float> (x1, y1_img2), color);
            circle (img1, cv::Point_<float> (pts1.at<float>(i, 0), pts1.at<float>(i,1)), 3, color, -1);
            circle (img2, cv::Point_<float> (pts2.at<float>(i, 0), pts2.at<float>(i,1)), 3, color, -1);
        }

        imshow("Epipolar lines using Fundamental matrix 1", img1);
        imshow("Epipolar lines using Fundamental matrix 2", img2);
        cv::waitKey (0);
    }


    void drawHomographies (std::vector<std::string> images_filename, const std::string &points_filename, cv::InputArray in_inliers,
                           const cv::Mat &H) {
        int * inliers =  (int *) in_inliers.getMat().data;
        int inliers_size = in_inliers.size().width;

        std::vector<int> gt_inliers;
        cv::Mat points1, points2;
        read_points(points1, points2, points_filename);
        getInliers(points_filename, gt_inliers);

        std::cout << "gt inliers " << gt_inliers.size() << '\n';
        std::cout << "inliers_size " << inliers_size << '\n';

        cv::Mat img1 = cv::imread(images_filename[0]);
        cv::Mat img2 = cv::imread(images_filename[1]);

        std::vector< cv::DMatch > good_matches, gt_good_matches;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        for (int i = 0; i < points1.rows; i++) {
            cv::KeyPoint kp1(points1.at<float>(i,0), points1.at<float>(i,1), 1);
            keypoints1.push_back(kp1);
            cv::KeyPoint kp2(points2.at<float>(i,0), points2.at<float>(i,1), 1);
            keypoints2.push_back(kp2);

        }

        for (int i = 0; i < inliers_size; i++) {
            cv::DMatch match (inliers[i], inliers[i], 0);
            good_matches.push_back(match);
        }
        for (int i = 0; i < gt_inliers.size(); i++) {
            cv::DMatch match (gt_inliers[i], gt_inliers[i], 0);
            gt_good_matches.push_back(match);
        }

        cv::Mat img_matches, gt_img_matches;
        cv::drawMatches (img1, keypoints1, img2, keypoints2,
                         good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                         std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::drawMatches (img1, keypoints1, img2, keypoints2,
                         gt_good_matches, gt_img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                         std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        cv::Mat imgs;

        cv::vconcat(img_matches, gt_img_matches, imgs);

        cv::Mat H_gt;
        getH (points_filename.substr(0, points_filename.find('_'))+"_model.txt", H_gt);

        cv::Mat panorama_gt, panorama;

        cv::warpPerspective(img2, panorama_gt, H_gt, cv::Size(2*img2.cols, img2.rows));
        cv::warpPerspective(img2, panorama, H, cv::Size(2*img2.cols, img2.rows));

        cv::Mat warp_img1, warp_img2, gt_warp_img1, gt_warp_img2;

        cv::warpPerspective(img1, warp_img1, H, cv::Size(img1.cols, img1.rows));
        cv::warpPerspective(img2, warp_img2, H, cv::Size(img2.cols, img2.rows));

        cv::warpPerspective(img1, gt_warp_img1, H_gt, cv::Size(img1.cols, img1.rows));
        cv::warpPerspective(img2, gt_warp_img2, H_gt, cv::Size(img2.cols, img2.rows));

        cv::Mat warp_imgs;
        cv::hconcat(warp_img1, warp_img2, warp_img1);
        cv::hconcat(gt_warp_img1, gt_warp_img2, gt_warp_img1);
        cv::vconcat(warp_img1, gt_warp_img1, warp_imgs);

//        cv::imshow ("gt perp img 1", gt_warp_img1);
//        cv::imshow ("gt perp img 2", gt_warp_img2);
//        cv::imshow ("perp img 1", warp_img1);
//        cv::imshow ("perp img 2", warp_img2);


        cv::Mat half_gt(panorama_gt, cv::Rect(0,0,img1.cols,img1.rows));
        cv::Mat half(panorama, cv::Rect(0,0,img1.cols,img1.rows));

        img1.copyTo(half_gt);
        img1.copyTo(half);

        cv::resize(panorama_gt, panorama_gt, cv::Size (0.5 * imgs.cols, 0.5 * imgs.rows));
        cv::resize(panorama, panorama, cv::Size (0.5 * imgs.cols, 0.5 * imgs.rows));

//        cv::imshow( "Grand Truth panorama", panorama_gt);
//        cv::imshow( "panorama", panorama);

        cv::resize(imgs, imgs, cv::Size (0.7 * imgs.cols, 0.7 * imgs.rows));
        cv::resize(warp_imgs, warp_imgs, cv::Size (0.7 * imgs.cols, 0.7 * imgs.rows));

        imshow("imgs ", imgs);
        imshow("warp imgs ", warp_imgs);

        cv::waitKey(0);

    }
};


#endif //RANSAC_DRAWING_H
