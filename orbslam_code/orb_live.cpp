#include <iostream>
#include <fstream>
#include <sstream>
#include<chrono>
#include <algorithm>
#include <vector> 
#include <sys/stat.h>
#include<opencv2/core/core.hpp>
#include "System.h"
#include "camera_utilities.hpp"

using namespace std;

std::string orb_slam_dir = "/home/peter/ORB_SLAM2/";
std::string orb_voc_path = orb_slam_dir + "Vocabulary/ORBvoc.txt";
std::string output_dir = "../data/";
int session_num = 0;
string session_dir = "session_0000/";
std::string frame_dir;
//std::string properties_path;
//std::string yaml_path;
unsigned long int num_frames_to_capture = 500;

void start_mono_slam(camera_utilities::Monocular_Camera mono_camera);
void start_stereo_slam(camera_utilities::Stereo_Camera stereo_camera);
void display_statistics(std::vector<double> &frame_times);

int main(){
    char option;
    std::cout << "SELECT AN OPTION TO RUN:" << std::endl;
    std::cout << "\t1) Monocular SLAM new calibration" << std::endl;
    std::cout << "\t2) Monocular SLAM saved calibration" << std::endl;
    std::cout << "\t3) Stereo SLAM new calibration" << std::endl;
    std::cout << "\t4) Stereo SLAM saved calibration" << std::endl;
    std::cout << "OPTION: ";
    std::cin >> option;
    
    //check that ouput directory exits
    if(mkdir(output_dir.c_str(), 0777) != -1) throw std::runtime_error("NO SUCH OUTPUT DIRECTORY: " + output_dir);
    //create a new directory for this sesion
    while(mkdir((output_dir+session_dir).c_str(), 0777) == -1){ //Failed to create folder, already exits
			session_num++;
			stringstream ss;
			ss << setw(4) << setfill('0') << session_num;
			session_dir = "session_" + ss.str() + "/";
		}
    //create directory to store frames in
    frame_dir = output_dir + session_dir + "frames/";
    mkdir(frame_dir.c_str(), 0777);

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
        camera_utilities::Monocular_Camera mono_camera(0, output_dir+session_dir+"mono_camera_properties.txt", {frame_width, frame_height}, fps, {chess_width, chess_height});
        std::cout << "CALIBRATION COMPLETE. LAUNCHING ORB SLAM 2..." << std::endl << std::endl;
        mono_camera.create_orbslam_settings(output_dir+session_dir+"mono_camera.yaml");
        start_mono_slam(mono_camera);
    }
    else if (option == '2'){
        string prop_path;
        //try to find the properties file from a previous session
        stringstream ss;
        ss << setw(4) << setfill('0') << (session_num-1);
        std::string prev_session_dir = "session_" + ss.str() + "/";
        prop_path = output_dir+prev_session_dir+"mono_camera_properties.txt";
        std::ifstream old_properties(prop_path, std::ios::binary);
        if(!old_properties.is_open()){ //file exists and is opened        
            std::cout << "NO PREVIOUS SESSION. ENTER PATH TO PROPERTIES FILE: ";
            std::cin >> prop_path;
        }
        old_properties.open(prop_path);
        if(!old_properties.is_open()) throw std::runtime_error("FAILED TO OPEN PROPERTIES FILE");
        std::ofstream new_properties(output_dir+session_dir+"mono_camera_properties.txt", std::ios::binary);
        new_properties << old_properties.rdbuf();

        //create a camera object and start the SLAM tracking
        camera_utilities::Monocular_Camera mono_camera(0, prop_path);
        mono_camera.create_orbslam_settings(output_dir+session_dir+"mono_camera.yaml");
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
        camera_utilities::Stereo_Camera stereo_camera(0, 2, output_dir+session_dir+"stereo_camera_properties.txt", {frame_width,frame_height}, fps, baseline, {chess_width,chess_height});
        std::cout << "CALIBRATION COMPLETE. LAUNCHING ORB SLAM 2..." << std::endl << std::endl;
        stereo_camera.create_orbslam_settings(output_dir+session_dir+"stereo_camera.yaml");
        start_stereo_slam(stereo_camera);
    }
    else if (option == '4'){
        string prop_path;
        //try to find the properties file from a previous session
        stringstream ss;
        ss << setw(4) << setfill('0') << (session_num-1);
        std::string prev_session_dir = "session_" + ss.str() + "/";
        prop_path = output_dir+prev_session_dir+"stereo_camera_properties.txt";
        std::ifstream old_properties(prop_path, std::ios::binary);
        if(!old_properties.is_open()){ //file exists and is opened        
            std::cout << "NO PREVIOUS SESSION. ENTER PATH TO PROPERTIES FILE: ";
            std::cin >> prop_path;
        }
        old_properties.open(prop_path);
        if(!old_properties.is_open()) throw std::runtime_error("FAILED TO OPEN PROPERTIES FILE");
        std::ofstream new_properties(output_dir+session_dir+"stereo_camera_properties.txt", std::ios::binary);
        new_properties << old_properties.rdbuf();

        //create a camera object and start the SLAM tracking
        camera_utilities::Stereo_Camera stereo_camera(0, 2,  prop_path);
        stereo_camera.create_orbslam_settings(output_dir+session_dir+"stereo_camera.yaml");
        start_stereo_slam(stereo_camera);
    }
    else{
        std::cout << "INVALID OPTION. EXITING..." << std::endl;
        return -1;
    }
    return 0;
}

void start_mono_slam(camera_utilities::Monocular_Camera mono_camera){
    ORB_SLAM2::System SLAM(orb_voc_path, output_dir+session_dir+"mono_camera.yaml",ORB_SLAM2::System::MONOCULAR,true);
    cv::Mat frame;
    mono_camera.get_frame(frame);
    stringstream ss;
    int pad_to_width = to_string(num_frames_to_capture).length();
    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    std::vector<double> frame_times;
    for(unsigned long int time_stamp=0; time_stamp<num_frames_to_capture; time_stamp++){
        t1 = std::chrono::steady_clock::now();
        SLAM.TrackMonocular(frame, time_stamp);
        mono_camera.get_frame(frame);
        ss << setw(pad_to_width) << setfill('0') << time_stamp;
        cv::imwrite(frame_dir+ ss.str() +".png", frame);
        ss.str("");
        t2 = std::chrono::steady_clock::now();
        frame_times.push_back(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count());
    }
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM(output_dir+session_dir+"mono_trajectory.txt");
    display_statistics(frame_times);
}

void start_stereo_slam(camera_utilities::Stereo_Camera stereo_camera){
    ORB_SLAM2::System SLAM(orb_voc_path, output_dir+session_dir+"stereo_camera.yaml",ORB_SLAM2::System::STEREO,true);
    cv::Mat frames[2];
    stereo_camera.get_stereo_frame_rectified(frames);
    stringstream ss;
    int pad_to_width = to_string(num_frames_to_capture).length();
    std::chrono::steady_clock::time_point t1;
    std::chrono::steady_clock::time_point t2;
    std::vector<double> frame_times;
    for(unsigned long int time_stamp=0; time_stamp<num_frames_to_capture; time_stamp++){
        t1 = std::chrono::steady_clock::now();
        SLAM.TrackStereo(frames[0], frames[1], time_stamp);
        stereo_camera.get_stereo_frame_rectified(frames);
        ss << setw(pad_to_width) << setfill('0') << time_stamp;
        cv::imwrite(frame_dir + ss.str() +"_left.png", frames[0]);
        cv::imwrite(frame_dir+ ss.str() +"_right.png", frames[1]);
        ss.str("");
        t2 = std::chrono::steady_clock::now();
        frame_times.push_back(std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count());
    }
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM(output_dir+session_dir+"stereo_trajectory.txt");
    display_statistics(frame_times);
}

void display_statistics(std::vector<double> &frame_times){
    std::sort(frame_times.begin(), frame_times.end());
    double total_time = 0;
    for(unsigned int i=0; i<frame_times.size(); i++)
        total_time += frame_times[i];
    double mean_frame_time = total_time / frame_times.size();
    double median_frame_time = frame_times[frame_times.size()/2];

    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Total Time:\t\t" << total_time << std::endl;
    std::cout << "Mean Frame Time:\t" << mean_frame_time << std::endl;
    std::cout << "Median Frame Time:\t" << median_frame_time << std::endl;
}