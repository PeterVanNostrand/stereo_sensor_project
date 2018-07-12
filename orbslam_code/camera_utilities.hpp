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
    const double default_viewer_params[10] = {0.05,1,0.9,2,0.08,3,0,-0.7,-1.8,500};
    const double default_orb_params[5] = {1000,1.2,8,20,7};
    class Monocular_Camera {
        public:
            //Constructor for camera with known intrinsic properties (matrix, dist coeffs, etc)
            Monocular_Camera(int cam_num, std::string prop_path){
                camera = cv::VideoCapture(cam_num);
                load_camera_properties(prop_path);
            }
            //Constructor for camera with unknown intrinsic properties (matrix, dist_coeffs, etc)
            //These values are found using chessboard image calibration
            Monocular_Camera(int cam_num, std::string prop_path, cv::Size chessboard_dim, cv::Size frame_size, int frame_rate){
                camera = cv::VideoCapture(cam_num);
                this->frame_size = frame_size;
                this->frame_rate = frame_rate;
                calibrate_camera(chessboard_dim);
                save_camera_properties(prop_path);
            }
            cv::Mat get_frame_undistorted(){
                camera.grab();
                cv::Mat frame;
                cv::Mat frame_undistorted;
                camera.retrieve(frame, 0);
                cv::undistort(frame, frame_undistorted, camera_matrix, dist_coeffs, camera_matrix);
                return frame_undistorted;
            }
            cv::Mat get_frame(){
                camera.grab();
                cv::Mat frame;
                camera.retrieve(frame, 0);
                return frame;
            }
            void create_orbslam_settings(std::string yaml_path, const double orb_params[5]=default_orb_params, const double viewer_params[10]=default_viewer_params){
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
}