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
        std::string filename = "../results/linefitting_"+best_model->model_name+".jpg";
        cv::imwrite(filename, image);
        cv::waitKey (0);
    }

    // DLT

    /*
     * Draw epipolar lines by Fundamental Matrix
     */
    void drawEpipolarLines (const std::vector<std::string>& images_filename, cv::InputArray points1, cv::InputArray points2, const cv::Mat& F) {
        cv::Mat pts1 = points1.getMat(), pts2 = points2.getMat();

        cv::hconcat(pts1, cv::Mat_<float>::ones (pts1.rows, 1), pts1);
        cv::hconcat(pts2, cv::Mat_<float>::ones (pts2.rows, 1), pts2);

        /*
         * For every point in one of the two images of a stereo pair,
         * the function finds the equation of the corresponding epipolar
         * line in the other image.
         */
        cv::Mat lines1, lines2;
        cv::computeCorrespondEpilines(pts2, 2, F, lines1);
        cv::computeCorrespondEpilines(pts1, 1, F, lines2);

        cv::Mat img1 = cv::imread(images_filename[0]),
                img2 = cv::imread(images_filename[1]),
                img3 = cv::imread(images_filename[0]),
                img4 = cv::imread(images_filename[1]);

        drawEpipolarLines_ (img1, img2, lines1, pts1, pts2);
        drawEpipolarLines_ (img4, img3, lines2, pts2, pts1);

        cv::hconcat(img1, img4, img1);

        cv::resize(img1, img1, cv::Size(0.8*img1.cols, img1.rows));

        imshow("Epipolar lines using Fundamental matrix 1", img1);

        cv::waitKey (0);
    }

    void drawEpipolarLines_ (cv::Mat &img1, cv::Mat& img2, const cv::Mat& lines, const cv::Mat& pts1, const cv::Mat& pts2) {
        int c = img1.cols, r = img1.rows;
        float x0, y0, x1, y1;
        float  r0, r1, r2;
        for (int i = 0; i < pts1.rows; i++) {
            r0 = lines.at<float>(i, 0);
            r1 = lines.at<float>(i, 1);
            r2 = lines.at<float>(i, 2);

            x0 = 0;
            x1 = c;

            y0 = -r2/r1;
            y1 = -(r2 + r0*c)/r1;

            cv::Scalar color = cv::Scalar(random()%255, random ()%255, random ()%255);
            cv::line(img1, cv::Point_<float> (x0, y0), cv::Point_<float> (x1, y1), color);
            cv::circle (img1, cv::Point_<float> (pts1.at<float>(i, 0), pts1.at<float>(i,1)), 3, color, -1);
            cv::circle (img2, cv::Point_<float> (pts2.at<float>(i, 0), pts2.at<float>(i,1)), 3, color, -1);
        }
    }


    void drawHomographies (const std::vector<std::string>& images_filename, const std::string &points_filename, cv::InputArray in_inliers,
                           const cv::Mat &H) {
        int * inliers =  (int *) in_inliers.getMat().data;
        int inliers_size = in_inliers.size().width;

        std::vector<int> gt_inliers;
        cv::Mat points1, points2;
        read_points(points1, points2, points_filename);

        if (points1.cols == 2) {
            cv::Mat ones = cv::Mat_<float>::ones(points1.rows, 1);
            cv::hconcat(points1, ones, points1);
            cv::hconcat(points2, ones, points2);
        }

        getInliers(points_filename, gt_inliers);

        std::cout << "gt inliers " << gt_inliers.size() << '\n';
        std::cout << "inliers_size " << inliers_size << '\n';

        cv::Mat img1 = cv::imread(images_filename[0]);
        cv::Mat img2 = cv::imread(images_filename[1]);

        cv::Mat img1_inl = cv::imread (images_filename[0]);
        cv::Mat img2_inl = cv::imread (images_filename[1]);

        cv::Mat gt_img1_inl = cv::imread (images_filename[0]);
        cv::Mat gt_img2_inl = cv::imread (images_filename[1]);

        cv::Mat opencv_img1_inl = cv::imread (images_filename[0]);
        cv::Mat opencv_img2_inl = cv::imread (images_filename[1]);

        // draw matches
        cv::Mat img_matches, gt_img_matches;
        drawMatches(img_matches, img1, img2, points1, points2, inliers, inliers_size);
        drawMatches(gt_img_matches, img1, img2, points1, points2, &gt_inliers[0], gt_inliers.size());
        cv::vconcat(img_matches, gt_img_matches, img_matches);
        // ----------------------


        cv::Mat H_opencv = cv::Mat_<float>(cv::findHomography(points1, points2));
        cv::Mat H_gt;
        getH (points_filename.substr(0, points_filename.find('_'))+"_model.txt", H_gt);

        drawErrors(img1_inl, img2_inl, points1, points2, H);
        drawErrors(gt_img1_inl, gt_img2_inl, points1, points2, H_gt.inv());
        drawErrors(opencv_img1_inl, opencv_img2_inl, points1, points2, H_opencv);

        cv::vconcat(img2_inl, gt_img2_inl, img2_inl);
        cv::vconcat(img2_inl, opencv_img2_inl, img2_inl);
        cv::vconcat(img1_inl, gt_img1_inl, img1_inl);
        cv::vconcat(img1_inl, opencv_img1_inl, img1_inl);
        cv::hconcat(img1_inl, img2_inl, img2_inl);

        cv::resize(img2_inl, img2_inl, cv::Size (0.75 * img2_inl.cols, 0.5 * img2_inl.rows));
        cv::imshow ("estimated points on correspondence images vs grand truth estimated points vs opencv", img2_inl);

        // draw panorama
        cv::Mat panorama_opencv, panorama_gt, panorama;
        std::vector<cv::Mat> images;
        images.push_back(img1); images.push_back(img2);
        drawPanorama(images, panorama, H.inv());
        drawPanorama(images, panorama_gt, H_gt);
        drawPanorama(images, panorama_opencv, H_opencv.inv());

        cv::resize(img_matches, img_matches, cv::Size (0.7 * img_matches.cols, 0.7 * img_matches.rows));

        cv::vconcat(panorama, panorama_gt, panorama);
        cv::vconcat(panorama, panorama_opencv, panorama);

        cv::resize(panorama, panorama, cv::Size ( img_matches.cols, 1 * img_matches.rows));
        cv::imshow("panorama", panorama);
        // -------------------------------------

        cv::imshow("imgs ", img_matches);

        cv::waitKey(0);
    }

    void drawPanorama (const std::vector<cv::Mat>& imgs, cv::Mat& panorama, const cv::Mat& H);

    void drawMatches (cv::Mat& img_matches, const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& points1, const cv::Mat& points2, const int * const inliers, int inliers_size) {
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

    void drawErrors (cv::Mat &img1_error, cv::Mat &img2_error, const cv::Mat& points1, const cv::Mat& points2, const cv::Mat& H) {
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

            cv::circle (img1_error, cv::Point_<float>(points1.at<float>(i, 0), points1.at<float>(i, 1)), 3, cv::Scalar(255, 255, 0), -1);
            cv::circle (img2_error, cv::Point_<float>(points2.at<float>(i, 0), points2.at<float>(i, 1)), 3, cv::Scalar(255, 255, 0), -1);

            cv::circle (img1_error, cv::Point_<float>(est_pts_on_corr1.at<float>(0, i), est_pts_on_corr1.at<float>(1, i)), 3, cv::Scalar(255, 0, 0), -1);
            cv::circle (img2_error, cv::Point_<float>(est_pts_on_corr2.at<float>(0, i), est_pts_on_corr2.at<float>(1, i)), 3, cv::Scalar(255, 0, 0), -1);
        }
    }

    void mat2keypoint(const cv::Mat& points, std::vector<cv::KeyPoint>& keypoints) {
        for (int i = 0; i < points.rows; i++) {
            cv::KeyPoint kp(points.at<float>(i,0), points.at<float>(i,1), 1);
            keypoints.push_back(kp);
        }
    }

    void mat2point (const cv::Mat& points, std::vector<cv::Point_<float>>& vpoints) {
        for (int i = 0; i < points.rows; i++) {
            cv::Point_<float> p (points.at<float>(i,0), points.at<float>(i,1));
            vpoints.push_back(p);
        }
    }
};


#endif //RANSAC_DRAWING_H
