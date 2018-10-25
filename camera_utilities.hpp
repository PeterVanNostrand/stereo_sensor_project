#pragma once

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace camera_utilities {
    const double default_mono_viewer_params[10] = {0.05,1,0.9,2,0.08,3,0,-0.7,-1.8,500};
    const double default_mono_orb_params[5] = {1000,1.2,8,20,7};
    const double default_stereo_viewer_params[10] = {0.6,2,1,2,0.7,3,0,-100,-0.1,2000};
    const double default_stereo_orb_params[5] = {2000,1.2,8,12,7};

    // helper function swap defined for cv::Mat pointers
    void swap(cv::Mat  **a, cv::Mat **b);

    class Monocular_Camera {
        public:
            //Constructor for camera with known intrinsic properties (matrix, dist coeffs, etc)
            Monocular_Camera(int cam_num, std::string prop_path);
            //Constructor for camera with unknown intrinsic properties (matrix, dist_coeffs, etc)
            //These values are found using chessboard image calibration
            Monocular_Camera(int cam_num, std::string prop_path, cv::Size frame_size, int frame_rate, cv::Size chessboard_dim);
            void get_frame_undistorted(cv::Mat **out_frame);
            void get_frame(cv::Mat **out_frame);
            void create_orbslam_settings(std::string yaml_path, const double orb_params[5]=default_mono_orb_params, const double viewer_params[10]=default_mono_viewer_params);
            void start(bool rectify_images);
            void stop();
            void init_frame(cv::Mat **frame);
        private:
            // camera object and properties
            cv::VideoCapture camera;
            cv::Size frame_size;
            int frame_rate;
            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;
            // Pointers to frames
            cv::Mat *latest_frame, *latest_frame_rectified, *temp_frame, *temp_frame_rectified;
            // Threading fields
            std::thread this_thread;
            std::mutex thread_status;
            bool shutdown;
            bool has_updated;

            void calibrate_camera(cv::Size chessboard_dim);
            void load_camera_properties(std::string prop_path);
            void save_camera_properties(std::string prop_path);
            void run_thread(bool rectify_images);
            void update_frame();
            void update_frame_rectified();
    };

    class Stereo_Camera{
        public:
            Stereo_Camera(int left_cam_num, int right_cam_num, std::string prop_path);
            Stereo_Camera(int left_cam_num, int right_cam_num, std::string prop_path, cv::Size frame_size, int frame_rate, double baseline_dist,  cv::Size chessboard_dim);
            void get_stereo_frame(cv::Mat **left_frame_out, cv::Mat **right_frame_out);
            void get_stereo_frame_rectified(cv::Mat **left_frame_out, cv::Mat **right_frame_out);
            void create_orbslam_settings(std::string yaml_path, const double orb_params[5]=default_stereo_orb_params, const double viewer_params[10]=default_stereo_viewer_params, int ThDepth=40);
            void create_disparity_matcher(int num_disparities=0, int block_size=21, int prefilter_cap=31, int prefilter_size=9, int prefilter_type=0, int texture_threshold=10, int uniqueness_ratio=15);
            void get_disparity(cv::Mat& out_disparity);
            void start(bool rectify_images);
            void stop();
            void init_frames(cv::Mat **left, cv::Mat **right);
        private:
            // Camera objects
            cv::VideoCapture left_camera, right_camera;
            // Camera properties
            cv::Size frame_size;
            int frame_rate;
            cv::Mat left_camera_matrix, right_camera_matrix;
            cv::Mat left_dist_coeffs, right_dist_coeffs;            
            cv::Mat R, T; //relative Rotation and Translation between two cameras
            cv::Mat C0M0, C0M1; //cam 0 map 0, cam 0 map 1 -- 0=left 1=right
            cv::Mat C1M0, C1M1; //cam 1 map 0, cam 1 map 1 -- 0=left 1=right
            double baseline_dist; //horizontal distance between camera centers in meters
            cv::Ptr<cv::StereoBM> stereo_matcher;
            // Pointers to frames
            cv::Mat *latest_frame_left, *latest_frame_right, *latest_frame_left_rectified, *latest_frame_right_rectified;
            // Threading fields
            std::thread this_thread;
            std::mutex thread_status;
            bool shutdown;
            bool has_updated;
             cv::Mat *temp_frame_left, *temp_frame_right, *temp_frame_left_rectified, *temp_frame_right_rectified;

            void calibrate_stereo_camera(cv::Size chessboard_dim);
            void load_camera_properties(std::string prop_path);
            void save_stereo_camera_properties(std::string prop_path);
            void run_thread(bool rectify_images);
            void update_stereo_frame();
            void update_stereo_frame_rectified();
    };
}
