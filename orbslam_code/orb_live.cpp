#include <iostream>
#include<opencv2/core/core.hpp>
#include "System.h"
#include "camera_utilities.hpp"

using namespace std;

void start_mono_slam(camera_utilities::Monocular_Camera mono_camera);
void start_stereo_slam(camera_utilities::Stereo_Camera stereo_camera);

int main(){
    char option;
    std::cout << "SELECT AN OPTION TO RUN:" << std::endl;
    std::cout << "\t1) Monocular SLAM new calibration" << std::endl;
    std::cout << "\t2) Monocular SLAM saved calibration" << std::endl;
    std::cout << "\t3) Stereo SLAM new calibration" << std::endl;
    std::cout << "\t4) Stereo SLAM saved calibration" << std::endl;
    std::cout << "OPTION: ";
    std::cin >> option;
    
    if(option == '1'){
        int frame_width, frame_height, chess_width, chess_height, fps;
        std::cout << "ENTER FRAME WIDTH: ";
        std::cin >> frame_width;
        std::cout << "ENTER FRAME HEIGHT: ";
        std::cin >> frame_height;
        std::cout << "ENTER FRAME RATE: ";
        std::cin >> fps;
        std::cout << "ENTER CHESSBOARD WIDTH: ";
        std::cin >> chess_width;
        std::cout << "ENTER CHESSBOARD HEIGHT: ";
        std::cin >> chess_height;
        std::cout << "STARTING CALIBRATION..." << std::endl;
        camera_utilities::Monocular_Camera mono_camera(0, "../data/mono_camera_properties.txt", {frame_width, frame_height}, fps, {chess_width, chess_height});
        std::cout << "CALIBRATION COMPLETE. LAUNCHING ORB SLAM 2..." << std::endl << std::endl;
        mono_camera.create_orbslam_settings("../data/mono_camera.yaml");
        start_mono_slam(mono_camera);
    }
    else if (option == '2'){
        camera_utilities::Monocular_Camera mono_camera(0, "../data/mono_camera_properties.txt");
        start_mono_slam(mono_camera);
    }
    else if (option == '3'){
         int frame_width, frame_height, chess_width, chess_height, fps;
        double baseline;
        std::cout << "ENTER FRAME WIDTH: ";
        std::cin >> frame_width;
        std::cout << "ENTER FRAME HEIGHT: ";
        std::cin >> frame_height;
        std::cout << "ENTER FRAME RATE: ";
        std::cin >> fps;
        std::cout << "ENTER CHESSBOARD WIDTH: ";
        std::cin >> chess_width;
        std::cout << "ENTER CHESSBOARD HEIGHT: ";
        std::cin >> chess_height;
        std::cout << "ENTER BASELINE DISTANCE IN METERS: ";
        std::cin >>baseline;
        std::cout << "STARTING CALIBRATION..." << std::endl;
        camera_utilities::Stereo_Camera stereo_camera(0, 2, "../data/stereo_camera_properties.txt", {frame_width,frame_height}, fps, baseline, {chess_width,chess_height});
        std::cout << "CALIBRATION COMPLETE. LAUNCHING ORB SLAM 2..." << std::endl << std::endl;
        stereo_camera.create_orbslam_settings("../data/stereo_camera.yaml");
        start_stereo_slam(stereo_camera);
    }
    else if (option == '4'){
        camera_utilities::Stereo_Camera stereo_camera(0, 2,  "../data/stereo_camera_properties.txt");
        start_stereo_slam(stereo_camera);
    }
    else{
        std::cout << "INVALID OPTION. EXITING..." << std::endl;
        return -1;
    }
    return 0;
}

void start_mono_slam(camera_utilities::Monocular_Camera mono_camera){
    ORB_SLAM2::System SLAM("/home/peter/ORB_SLAM2/Vocabulary/ORBvoc.txt","/home/peter/Documents/Peter/REU/stereo_sensor_project/orbslam_code/data/mono_camera.yaml",ORB_SLAM2::System::MONOCULAR,true);
    cv::Mat frame;
    mono_camera.get_frame(frame);
    for(unsigned long int time_stamp=0;time_stamp<500;time_stamp++){
        SLAM.TrackMonocular(frame, time_stamp);
        mono_camera.get_frame(frame);
    }
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("../data/MonoKeyFrameTrajectory.txt");
}

void start_stereo_slam(camera_utilities::Stereo_Camera stereo_camera){
    ORB_SLAM2::System SLAM("~/ORB_SLAM2/Vocabulary/ORBvoc.txt","../data/stereo_camera.yaml",ORB_SLAM2::System::STEREO,true);
    cv::Mat frames[2];
    stereo_camera.get_stereo_frame_rectified(frames);
    for(unsigned long int time_stamp=0;time_stamp<500;time_stamp++){
        SLAM.TrackStereo(frames[0], frames[1], time_stamp);
        stereo_camera.get_stereo_frame_rectified(frames);
    }
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("../data/StereoKeyFrameTrajectory.txt");
}