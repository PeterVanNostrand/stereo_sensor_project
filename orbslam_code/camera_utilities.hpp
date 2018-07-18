#pragma once

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace camera_utilities {
    const double default_mono_viewer_params[10] = {0.05,1,0.9,2,0.08,3,0,-0.7,-1.8,500};
    const double default_mono_orb_params[5] = {1000,1.2,8,20,7};
    const double default_stereo_viewer_params[10] = {0.6,2,1,2,0.7,3,0,-100,-0.1,2000};
    const double default_stereo_orb_params[5] = {2000,1.2,8,12,7};
    class Monocular_Camera {
        public:
            //Constructor for camera with known intrinsic properties (matrix, dist coeffs, etc)
            Monocular_Camera(int cam_num, std::string prop_path){
                camera = cv::VideoCapture(cam_num);
                load_camera_properties(prop_path);
            }
            //Constructor for camera with unknown intrinsic properties (matrix, dist_coeffs, etc)
            //These values are found using chessboard image calibration
            Monocular_Camera(int cam_num, std::string prop_path, cv::Size frame_size, int frame_rate, cv::Size chessboard_dim){
                camera = cv::VideoCapture(cam_num);
                this->frame_size = frame_size;
                this->frame_rate = frame_rate;
                calibrate_camera(chessboard_dim);
                save_camera_properties(prop_path);
            }
            void get_frame_undistorted(cv::Mat& out_frame){
                camera.grab();
                cv::Mat frame;
                camera.retrieve(frame, 0);
                cv::undistort(frame, out_frame, camera_matrix, dist_coeffs, camera_matrix);
            }
            void get_frame(cv::Mat& out_frame){
                camera.grab();
                camera.retrieve(out_frame, 0);
            }
            void create_orbslam_settings(std::string yaml_path, const double orb_params[5]=default_mono_orb_params, const double viewer_params[10]=default_mono_viewer_params){
                std::fstream yaml(yaml_path, std::fstream::out);
                if(!yaml.is_open()) throw std::runtime_error("UNABLE TO CREATE SETTINGS FILE");
                //start of file contents
                yaml << "%YAML:1.0" << std::endl;
                yaml << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;
                yaml << "# Camera Parameters for Monocular Camera" << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;
                yaml << std::endl;
                yaml << "# Camera calibration and distortion parameters (OpenCV) " << std::endl;
                yaml << "Camera.fx: " << camera_matrix.at<double>(0,0) << std::endl;
                yaml << "Camera.fy: " << camera_matrix.at<double>(1,1) << std::endl;
                yaml << "Camera.cx: " << camera_matrix.at<double>(0,2) << std::endl;
                yaml << "Camera.cy: " << camera_matrix.at<double>(1,2) << std::endl;
                yaml << std::endl;
                yaml << "Camera.k1: " << dist_coeffs.at<double>(0,0) << std::endl;
                yaml << "Camera.k2: " << dist_coeffs.at<double>(0,1) << std::endl;
                yaml << "Camera.p1: " << dist_coeffs.at<double>(0,2) << std::endl;
                yaml << "Camera.p2: " << dist_coeffs.at<double>(0,3) << std::endl;
                yaml << "Camera.k3: " << dist_coeffs.at<double>(0,4) << std::endl;
                yaml << std::endl;
                yaml << "Camera.fps: " << frame_rate << std::endl;
                yaml << std::endl;
                yaml << "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)" << std::endl;
                yaml << "Camera.RGB: 0" << std::endl; //OpenCV always uses BGR
                yaml << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;   
                yaml << "# ORB Parameters" << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Number of features per image" << std::endl;
                yaml << "ORBextractor.nFeatures: " << orb_params[0] << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Scale factor between levels in the scale pyramid" << std::endl;
                yaml << "ORBextractor.scaleFactor: " << orb_params[1] << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Number of levels in the scale pyramid" << std::endl;
                yaml << "ORBextractor.nLevels: " << orb_params[2] << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Fast threshold" << std::endl;
                yaml << "# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response." << std::endl;
                yaml << "# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST" << std::endl;
                yaml << "# You can lower these values if your images have low contrast" << std::endl;			
                yaml << "ORBextractor.iniThFAST: " << orb_params[3] << std::endl;
                yaml << "ORBextractor.minThFAST: " << orb_params[4] << std::endl;
                yaml << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;	
                yaml << "# Viewer Parameters" << std::endl;	
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;	
                yaml << "Viewer.KeyFrameSize: " << viewer_params[0] << std::endl;
                yaml << "Viewer.KeyFrameLineWidth: " << viewer_params[1] << std::endl;
                yaml << "Viewer.GraphLineWidth: " << viewer_params[2] << std::endl;
                yaml << "Viewer.PointSize: " << viewer_params[3] << std::endl;
                yaml << "Viewer.CameraSize: " << viewer_params[4] << std::endl;
                yaml << "Viewer.CameraLineWidth: " << viewer_params[5] << std::endl;
                yaml << "Viewer.ViewpointX: " << viewer_params[6] << std::endl;
                yaml << "Viewer.ViewpointY: " << viewer_params[7] << std::endl;
                yaml << "Viewer.ViewpointZ: " << viewer_params[8] << std::endl;
                yaml << "Viewer.ViewpointF: " << viewer_params[9] << std::endl;
                //end of file contents
                yaml.close();
            }
        private:
            cv::VideoCapture camera;
            cv::Size frame_size;
            int frame_rate;
            cv::Mat camera_matrix;
            cv::Mat dist_coeffs;         
            void calibrate_camera(cv::Size chessboard_dim){
                camera.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
                camera.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
                camera.set(cv::CAP_PROP_FPS, frame_rate);
                
                std::vector<std::vector<cv::Point2f>> imgpoints; //stores 2D points for location of chessboard corners found in image
                std::vector<std::vector<cv::Point3f>> objpoints; //stores 3D points for location of chessboard corners in world frame
                { //Block for gathering 2D->3D correspondence from chessboard images
                    //Creating appropriate 3D coordinate plane for chessboard size
                    std::vector<cv::Point3f> real_corners;
                    for(int j=0; j<chessboard_dim.height; ++j)
                        for(int i=0; i<chessboard_dim.width; ++i)
                            real_corners.push_back(cv::Point3f(i,j,0)); //Maps corners to x,y,z plane with z=0 being chessboard surface
                    
                    cv::namedWindow("Camera Feed");
                    int fail_count = 0;
                    cv::Mat frame_color;
                    cv::Mat frame_gray;
                    while(fail_count < 10){
                        if(!camera.grab()){ //failed to get frame
                            ++fail_count;
                            continue;
                        }
                        camera.retrieve(frame_color, 0);
                        cv::imshow("Camera Feed", frame_color);
                        int key = cv::waitKey(1);
                        if(key%256 == 27) //ESC was pressed, stop taking frames
                            break;
                        if(key%256 == 32){ //SPACE was presed, process the frames
                            std::vector<cv::Point2f> frame_corners;
                            cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);
                            if(!cv::findChessboardCorners(frame_gray, chessboard_dim, frame_corners, 0)) //no chesssboard was found
                                continue; //jump to the next loop
                            cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,20,0.03);
                            cv::cornerSubPix(frame_gray, frame_corners, {5,5}, {-1,-1}, termcrit); //refine the location of the chessboard corners
                            imgpoints.push_back(frame_corners); //save 2D points
                            objpoints.push_back(real_corners); //save 3D mapping of 2D points
                            cv::drawChessboardCorners(frame_color, chessboard_dim, frame_corners, true);
                            cv::imshow("Camera Feed", frame_color);
                            cv::waitKey(500);
                        }
                    }
                    cv::destroyAllWindows();
                    if(fail_count>=10) throw std::runtime_error( "UNABLE TO GET IMAGES FROM CAMERA" );
                }

                //Use objpoints and imgpoints to create camera_matrix and dist_coeffs
                //These are used to undistort later images and create a 2D->3D mapping 
		        std::vector<cv::Mat> rvecs;
		        std::vector<cv::Mat> tvecs;
                cv::calibrateCamera(objpoints, imgpoints, chessboard_dim, camera_matrix, dist_coeffs, rvecs, tvecs, 0);
            }    
            void load_camera_properties(std::string prop_path){
                cv::FileStorage fs(prop_path, cv::FileStorage::READ);
                if (!fs.isOpened()) throw std::runtime_error("UNABLE TO OPEN PROPERTIES FILE");
                fs["frame_size"] >> frame_size;
                fs["frame_rate"] >> frame_rate;
                fs["camera_matrix"] >> camera_matrix;
                fs["dist_coeffs"] >> dist_coeffs;
                fs.release();
                camera.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
                camera.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
                camera.set(cv::CAP_PROP_FPS, frame_rate);
            }
            void save_camera_properties(std::string prop_path){
                cv::FileStorage fs(prop_path, cv::FileStorage::WRITE);
                if (!fs.isOpened())  throw std::runtime_error("UNABLE TO CREATE PROPERTIES FILE");
                fs << "frame_size" << frame_size;
                fs << "frame_rate" << frame_rate;
                fs << "camera_matrix" << camera_matrix;
                fs << "dist_coeffs" << dist_coeffs;
                fs.release();
            }
    };

    class Stereo_Camera{
        public:
            Stereo_Camera(int left_cam_num, int right_cam_num, std::string prop_path){
                left_camera = cv::VideoCapture(left_cam_num);
                right_camera = cv::VideoCapture(right_cam_num);
                load_camera_properties(prop_path);
            }
            Stereo_Camera(int left_cam_num, int right_cam_num, std::string prop_path, cv::Size frame_size, int frame_rate, double baseline_dist,  cv::Size chessboard_dim){
                left_camera = cv::VideoCapture(left_cam_num);
                right_camera = cv::VideoCapture(right_cam_num);
                this->frame_size = frame_size;
                this->frame_rate = frame_rate;
                this->baseline_dist = baseline_dist;
                calibrate_stereo_camera(chessboard_dim);
                save_stereo_camera_properties(prop_path);
            }
            void get_stereo_frame(cv::Mat (&out_frames)[2]){
                    left_camera.grab();
                    right_camera.grab();
                    cv::Mat left_frame, right_frame;
                    left_camera.retrieve(out_frames[0], 0);
                    right_camera.retrieve(out_frames[1], 0);
            }
            void get_stereo_frame_rectified(cv::Mat (&out_frames)[2]){
                left_camera.grab();
                right_camera.grab();
                cv::Mat left_frame, right_frame;
                left_camera.retrieve(left_frame, 0);
                right_camera.retrieve(right_frame, 0);
                cv::remap(left_frame, out_frames[0], C0M0, C0M1, cv::INTER_LINEAR);
                cv::remap(right_frame, out_frames[1], C1M0, C1M1, cv::INTER_LINEAR);
            }
            void create_orbslam_settings(std::string yaml_path, const double orb_params[5]=default_stereo_orb_params, const double viewer_params[10]=default_stereo_viewer_params, int ThDepth=40){
                 std::fstream yaml(yaml_path, std::fstream::out);
                if(!yaml.is_open()) throw std::runtime_error("UNABLE TO CREATE SETTINGS FILE");
                //start of file contents
                yaml << "%YAML:1.0" << std::endl;
                yaml << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;
                yaml << "# Camera Parameters for Stereo Camera" << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;
                yaml << std::endl;
                yaml << "# Camera calibration and distortion parameters for left camera (OpenCV) " << std::endl;
                yaml << "Camera.fx: " << left_camera_matrix.at<double>(0,0) << std::endl;
                yaml << "Camera.fy: " << left_camera_matrix.at<double>(1,1) << std::endl;
                yaml << "Camera.cx: " << left_camera_matrix.at<double>(0,2) << std::endl;
                yaml << "Camera.cy: " << left_camera_matrix.at<double>(1,2) << std::endl;
                yaml << std::endl; //All distortion constants are 0 as the image is already rectified
                yaml << "Camera.k1: " << 0 << std::endl;
                yaml << "Camera.k2: " << 0 << std::endl;
                yaml << "Camera.p1: " << 0 << std::endl;
                yaml << "Camera.p2: " << 0 << std::endl;
                yaml << std::endl;
                yaml << "Camera.width: " << frame_size.width << std::endl;
                yaml << "Camera.height: " << frame_size.height << std::endl;
                yaml << std::endl;
                yaml << "Camera.fps: " << frame_rate << std::endl;
                yaml << std::endl;
                yaml << "# stereo baseline times fx" << std::endl;
                yaml << "Camera.bf: " << (left_camera_matrix.at<double>(0,0) * baseline_dist) << std::endl;
                yaml << std::endl;
                yaml << "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)" << std::endl;
                yaml << "Camera.RGB: 0" << std::endl; //OpenCV always uses BGR
                yaml << std::endl;
                yaml << "# Close/Far threshold. Baseline times." << std::endl;
                yaml << "ThDepth: " << ThDepth << std::endl;
                yaml << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;   
                yaml << "# ORB Parameters" << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Number of features per image" << std::endl;
                yaml << "ORBextractor.nFeatures: " << orb_params[0] << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Scale factor between levels in the scale pyramid" << std::endl;
                yaml << "ORBextractor.scaleFactor: " << orb_params[1] << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Number of levels in the scale pyramid" << std::endl;
                yaml << "ORBextractor.nLevels: " << orb_params[2] << std::endl;
                yaml << std::endl;
                yaml << "# ORB Extractor: Fast threshold" << std::endl;
                yaml << "# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response." << std::endl;
                yaml << "# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST" << std::endl;
                yaml << "# You can lower these values if your images have low contrast" << std::endl;			
                yaml << "ORBextractor.iniThFAST: " << orb_params[3] << std::endl;
                yaml << "ORBextractor.minThFAST: " << orb_params[4] << std::endl;
                yaml << std::endl;
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;	
                yaml << "# Viewer Parameters" << std::endl;	
                yaml << "#--------------------------------------------------------------------------------------------" << std::endl;	
                yaml << "Viewer.KeyFrameSize: " << viewer_params[0] << std::endl;
                yaml << "Viewer.KeyFrameLineWidth: " << viewer_params[1] << std::endl;
                yaml << "Viewer.GraphLineWidth: " << viewer_params[2] << std::endl;
                yaml << "Viewer.PointSize: " << viewer_params[3] << std::endl;
                yaml << "Viewer.CameraSize: " << viewer_params[4] << std::endl;
                yaml << "Viewer.CameraLineWidth: " << viewer_params[5] << std::endl;
                yaml << "Viewer.ViewpointX: " << viewer_params[6] << std::endl;
                yaml << "Viewer.ViewpointY: " << viewer_params[7] << std::endl;
                yaml << "Viewer.ViewpointZ: " << viewer_params[8] << std::endl;
                yaml << "Viewer.ViewpointF: " << viewer_params[9] << std::endl;
                //end of file contents
                yaml.close();
            }
            void create_disparity_matcher(int num_disparities=0, int block_size=21, int prefilter_cap=31, int prefilter_size=9, int prefilter_type=0, int texture_threshold=10, int uniqueness_ratio=15){
                /* STEREOBM PARAMETER DESCRIPTIONS
                 num_disparities = 0;
                 block_size = 21; must be odd
                 prefilter_cap = 31; value of I_capp
                 prefilter_size = 9; limit of the window size for CV_STEREO_NORMALIZED_RESPONSE
                 prefilter_type = 0; type of algorithm used to smooth intensities
                    0=CV_STEREO_BM_NORMALIZED_RESPONSE normalizes intensities
                    1=CV_STEREO_BM_XSOBEL uses first order sobel derivatives in the x direction
                 texture_threshold = 10; minimum amount of intensity variation for block to be used
                 uniqueness_ratio = 15;
                 */
                stereo_matcher = cv::StereoBM::create(num_disparities, block_size);
                stereo_matcher->setPreFilterType(prefilter_type);
                stereo_matcher->setPreFilterSize(prefilter_size);
                stereo_matcher->setPreFilterCap(prefilter_cap);
                stereo_matcher->setTextureThreshold(texture_threshold);
                stereo_matcher->setUniquenessRatio(uniqueness_ratio);
            }
            void get_disparity(cv::Mat& out_disparity){
                if(stereo_matcher == nullptr)
                    create_disparity_matcher();
                //grab frames
                left_camera.grab();
                right_camera.grab();
                //decode frames
                cv::Mat left_frame_color, right_frame_color;
                left_camera.retrieve(left_frame_color, 0);
                right_camera.retrieve(right_frame_color, 0);
                //convert frames to grayscale
                cv::Mat left_frame_gray, right_frame_gray;
                cv::cvtColor(left_frame_color, left_frame_gray, cv::COLOR_BGR2GRAY);
                cv::cvtColor(right_frame_color, right_frame_gray, cv::COLOR_BGR2GRAY);
                //recitfy frames to be row aligned
                cv::Mat left_fixed, right_fixed;
                cv::remap(left_frame_gray, left_fixed, C0M0, C0M1, cv::INTER_LINEAR);
                cv::remap(right_frame_gray, right_fixed, C1M0, C1M1, cv::INTER_LINEAR);
                //generate dispairty map
                stereo_matcher->compute(left_fixed, right_fixed, out_disparity);
            }
        private:
            cv::VideoCapture left_camera, right_camera;
            cv::Size frame_size;
            int frame_rate;
            cv::Mat left_camera_matrix, right_camera_matrix;
            cv::Mat left_dist_coeffs, right_dist_coeffs;            
            cv::Mat R, T; //relative Rotation and Translation between two cameras
            cv::Mat C0M0, C0M1; //cam 0 map 0, cam 0 map 1 -- 0=left 1=right
            cv::Mat C1M0, C1M1; //cam 1 map 0, cam 1 map 1 -- 0=left 1=right
            double baseline_dist; //horizontal distance between camera centers in meters
            cv::Ptr<cv::StereoBM> stereo_matcher;
            void calibrate_stereo_camera(cv::Size chessboard_dim){
                left_camera.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
                left_camera.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
                left_camera.set(cv::CAP_PROP_FPS, frame_rate);

                right_camera.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
                right_camera.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
                right_camera.set(cv::CAP_PROP_FPS, frame_rate);

                std::vector<std::vector<cv::Point2f>> left_imgpoints, right_imgpoints; //stores 2D points for location of chessboard corners found in image
                std::vector<std::vector<cv::Point3f>> left_objpoints, right_objpoints; //stores 3D points for location of chessboard corners in world frame
                { //Block for gathering 2D->3D correspondence from chessboard images
                    //Creating appropriate 3D coordinate plane for chessboard size
                    std::vector<cv::Point3f> real_corners;
                    for(int j=0; j<chessboard_dim.height; ++j)
                        for(int i=0; i<chessboard_dim.width; ++i)
                            real_corners.push_back(cv::Point3f(i,j,0)); //Maps corners to x,y,z plane with z=0 being chessboard surface
                    
                    cv::namedWindow("Left Camera Feed");
                    cv::namedWindow("Right Camera Feed");
                    int fail_count = 0;
                    cv::Mat left_frame_color, right_frame_color;
                    cv::Mat left_frame_gray, right_frame_gray;
                    while(fail_count < 10){
                        if( !left_camera.grab() || !right_camera.grab()){ //one or more of the cameras failed to get frame
                            ++fail_count;
                            continue;
                        }
                        left_camera.retrieve(left_frame_color, 0);
                        right_camera.retrieve(right_frame_color);
                        cv::imshow("Left Camera Feed", left_frame_color);
                        cv::imshow("Right Camera Feed", right_frame_color);
                        int key = cv::waitKey(1);
                        if(key%256 == 27) //ESC was pressed, stop taking frames
                            break;
                        if(key%256 == 32){ //SPACE was presed, process the frames
                            std::vector<cv::Point2f> left_frame_corners, right_frame_corners; //2D location of chessboard corners in each frame
                            
                            //convert both frames to grayscale
                            cv::cvtColor(left_frame_color, left_frame_gray, cv::COLOR_BGR2GRAY);
                            cv::cvtColor(right_frame_color, right_frame_gray, cv::COLOR_BGR2GRAY);
                            
                            //search for chessboards in both frames
                            bool found_left_chessboard = cv::findChessboardCorners(left_frame_gray, chessboard_dim, left_frame_corners, 0);
                            bool found_right_chessboard = cv::findChessboardCorners(right_frame_gray, chessboard_dim, right_frame_corners, 0);
                            
                            //if one or more frame did not contaiin a chessbaord
                            if(!found_left_chessboard || !found_right_chessboard)
                                continue; //jump to the next loop
                            cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,20,0.03);
                            
                            //refine corner locations for both frames
                            cv::cornerSubPix(left_frame_gray, left_frame_corners, {5,5}, {-1,-1}, termcrit); //refine the location of the chessboard corners
                            cv::cornerSubPix(right_frame_gray, right_frame_corners, {5,5}, {-1,-1}, termcrit); //refine the location of the chessboard corners
                            
                            left_imgpoints.push_back(left_frame_corners); //save 2D points
                            left_objpoints.push_back(real_corners); //save 3D mapping of 2D points

                            right_imgpoints.push_back(right_frame_color); //save 2D points
                            right_objpoints.push_back(real_corners); //save 3D mapping of 2D points

                            //display found corners
                            cv::drawChessboardCorners(left_frame_color, chessboard_dim, left_frame_corners, true);
                            cv::drawChessboardCorners(right_frame_color, chessboard_dim, right_frame_corners, true);
                            cv::imshow("Left Camera Feed", left_frame_color);
                            cv::imshow("Right Camera Feed", right_frame_color);
                            cv::waitKey(500);
                        }
                    }
                    cv::destroyAllWindows();
                    if(fail_count>=10) throw std::runtime_error( "UNABLE TO GET IMAGES FROM CAMERA" );
                }

                //Use objpoints and imgpoints to create a camera_matrix and dist_coeffs for both frames
                //These are used to undistort later images and create a 2D->3D mapping 
		        std::vector<cv::Mat> left_rvecs, right_rvecs;
		        std::vector<cv::Mat> left_tvecs, right_tvecs;
                cv::calibrateCamera(left_objpoints, left_imgpoints, chessboard_dim, left_camera_matrix, left_dist_coeffs, left_rvecs, left_tvecs, 0);
                cv::calibrateCamera(right_objpoints, right_imgpoints, chessboard_dim, right_camera_matrix, right_dist_coeffs, right_rvecs, right_tvecs, 0);

                //Create remapping matrices C0M0 C0M1 C1M0 C1M1 used to rectify the two images (make them row aligned)
                cv::Mat R0, R1, P0, P1, Q;
                //CALIB_ZERO_DISPARITY = use already determined camera matrices
                cv::stereoRectify(left_camera_matrix, left_dist_coeffs, right_camera_matrix, right_dist_coeffs, frame_size, R, T, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0);
                cv::initUndistortRectifyMap(left_camera_matrix, left_dist_coeffs, R0, P0, frame_size, CV_32FC1, C0M0, C0M1);
                initUndistortRectifyMap(right_camera_matrix, right_dist_coeffs, R1, P1, frame_size, CV_32FC1, C1M0, C1M1);
            }
            void load_camera_properties(std::string prop_path){
                cv::FileStorage fs(prop_path, cv::FileStorage::READ);
                if (!fs.isOpened()) throw std::runtime_error("UNABLE TO OPEN PROPERTIES FILE");
                fs["frame_size"] >> frame_size;
                fs["frame_rate"] >> frame_rate;
                fs["baseline_dist"] >> baseline_dist;
                fs["left_camera_matrix"] >> left_camera_matrix;
                fs["right_camera_matrix"] >> right_camera_matrix;
                fs["left_dist_coeffs"] >> left_dist_coeffs;
                fs["right_dist_coeffs"] >> right_dist_coeffs;
                fs["R"] >> R;
                fs["T"] >> T;
                fs.release();
                left_camera.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
                left_camera.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
                left_camera.set(cv::CAP_PROP_FPS, frame_rate);
                right_camera.set(cv::CAP_PROP_FRAME_WIDTH, frame_size.width);
                right_camera.set(cv::CAP_PROP_FRAME_HEIGHT, frame_size.height);
                right_camera.set(cv::CAP_PROP_FPS, frame_rate);
                //Create remapping matrices C0M0 C0M1 C1M0 C1M1 used to rectify the two images (make them row aligned)
                cv::Mat R0, R1, P0, P1, Q;
                //CALIB_ZERO_DISPARITY = use already determined camera matrices
                cv::stereoRectify(left_camera_matrix, left_dist_coeffs, right_camera_matrix, right_dist_coeffs, frame_size, R, T, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0);
                cv::initUndistortRectifyMap(left_camera_matrix, left_dist_coeffs, R0, P0, frame_size, CV_32FC1, C0M0, C0M1);
                initUndistortRectifyMap(right_camera_matrix, right_dist_coeffs, R1, P1, frame_size, CV_32FC1, C1M0, C1M1);
            }
            void save_stereo_camera_properties(std::string prop_path){
                cv::FileStorage fs(prop_path, cv::FileStorage::WRITE);
                if (!fs.isOpened())  throw std::runtime_error("UNABLE TO CREATE PROPERTIES FILE");
                fs << "frame_size" << frame_size;
                fs << "frame_rate" << frame_rate;
                fs << "baseline_dist" << baseline_dist;
                fs << "left_camera_matrix" << left_camera_matrix;
                fs << "right_camera_matrix" << right_camera_matrix;
                fs << "left_dist_coeffs" << left_dist_coeffs;
                fs << "right_dist_coeffs" << right_dist_coeffs;
                fs << "R" << R;
                fs << "T" << T;
                fs.release();
            }
    };
}