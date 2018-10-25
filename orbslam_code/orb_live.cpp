#include <iostream>
#include <fstream>
#include <sstream>
#include<chrono>
#include <algorithm>
#include <vector> 
#include <thread>
#include <sys/stat.h>
#include<opencv2/core/core.hpp>
#include "System.h"
#include "../camera_utilities.hpp"

using namespace std;

std::string orb_slam_dir = "/home/peter/ORB_SLAM2/";
std::string orb_voc_path = orb_slam_dir + "Vocabulary/ORBvoc.bin";
std::string output_dir = "../data/";
int session_num = 0;
string session_dir = "session_0000/";
std::string frame_dir;
unsigned int session_duration_sec = 30;
 std::string prev_session_dir;

void start_mono_slam(camera_utilities::Monocular_Camera &mono_camera);
void start_stereo_slam(camera_utilities::Stereo_Camera &stereo_camera);
void start_stereo_disparity(camera_utilities::Stereo_Camera &stereo_camera);
void display_statistics(std::vector<double> &frame_times);
string load_previous_calibration(string camera_type);

int main(){
    char option;
    std::cout << "SELECT AN OPTION TO RUN:" << std::endl;
    std::cout << "\t1) Monocular SLAM new calibration" << std::endl;
    std::cout << "\t2) Monocular SLAM saved calibration" << std::endl;
    std::cout << "\t3) Stereo SLAM new calibration" << std::endl;
    std::cout << "\t4) Stereo SLAM saved calibration" << std::endl;
    std::cout << "\t5) Stereo Depth new calibration" << std::endl;
    std::cout << "\t6) Stereo Depth saved calibration" << std::endl;
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

    if(option == '1'){  //Monocular SLAM, new calibration
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
        std::cout << "ENTER SESSION DURATION (SECONDS): ";
        std::cin >> session_duration_sec;
        std::cout << "STARTING CALIBRATION..." << std::endl;
        camera_utilities::Monocular_Camera mono_camera(0, output_dir+session_dir+"mono_camera_properties.yaml", {frame_width, frame_height}, fps, {chess_width, chess_height});
        std::cout << "CALIBRATION COMPLETE. LAUNCHING ORB SLAM 2..." << std::endl << std::endl;
        mono_camera.create_orbslam_settings(output_dir+session_dir+"mono_camera.yaml");
        start_mono_slam(mono_camera);
    }
    else if (option == '2') {//Monocular SLAM, saved calibration
        string prop_path = load_previous_calibration("mono");
        //create a camera object and start the SLAM tracking
        camera_utilities::Monocular_Camera mono_camera(0, prop_path);
        start_mono_slam(mono_camera);
    }
    else if (option == '3'){ //Stereo SLAM, new calibration
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
        std::cout << "ENTER SESSION DURATION (SECONDS): ";
        std::cin >> session_duration_sec;
        std::cout << "STARTING CALIBRATION..." << std::endl;
        camera_utilities::Stereo_Camera stereo_camera(0, 2, output_dir+session_dir+"stereo_camera_properties.yaml", {frame_width,frame_height}, fps, baseline, {chess_width,chess_height});
        std::cout << "CALIBRATION COMPLETE. LAUNCHING ORB SLAM 2..." << std::endl << std::endl;
	    const double stereo_orb_params[5] = {1000, 1.2, 8, 12, 7};
        stereo_camera.create_orbslam_settings(output_dir+session_dir+"stereo_camera.yaml", stereo_orb_params);
        start_stereo_slam(stereo_camera);
    }
    else if (option == '4'){ //Stereo SLAM, saved calibration
        string prop_path = load_previous_calibration("stereo");
        camera_utilities::Stereo_Camera stereo_camera(0, 2,  prop_path);
        start_stereo_slam(stereo_camera);
    }
    else if (option == '5'){ //Stereo Depth, new calibration
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
        camera_utilities::Stereo_Camera stereo_camera(0, 2, output_dir+session_dir+"stereo_camera_properties.yaml", {frame_width,frame_height}, fps, baseline, {chess_width,chess_height});
        std::cout << "CALIBRATION COMPLETE! STARTING DISPARITY MATCHER..." << std::endl << std::endl;
        start_stereo_disparity(stereo_camera);
    }
    else if (option == '6'){ //Stereo Depth, saved calibration
	    string prop_path = load_previous_calibration("stereo");
        camera_utilities::Stereo_Camera stereo_camera(0, 2,  prop_path);
        start_stereo_disparity(stereo_camera);
    }
    else{
        std::cout << "INVALID OPTION. EXITING..." << std::endl;
    	return -1;
    }
    return 0;
}

void start_mono_slam(camera_utilities::Monocular_Camera &mono_camera){
    std::string map_path = "";
    char load_map;
    std::cout << "LOAD MAP? <Y/N>: ";
    std::cin >> load_map;
    if(load_map=='Y' or load_map=='y'){
        map_path = output_dir+prev_session_dir+"mono_data.map";
    }

    //Creating ORB object
    ORB_SLAM2::System SLAM(orb_voc_path, output_dir+session_dir+"mono_camera.yaml", map_path, ORB_SLAM2::System::MONOCULAR,true);
    
    //Prepare to load frame
    int frame_number = 0;
    cv::Mat* frame;
    mono_camera.init_frame(&frame);
    mono_camera.start(true);
    
    //Determine naming schema for saved frames
    stringstream ss;
    int pad_to_width = to_string(session_duration_sec*100).length();

    //Prepare variables for statistics tracking
    std::chrono::steady_clock::time_point frame_start;
    std::chrono::steady_clock::time_point frame_end;
    std::vector<double> frame_times;

    std::chrono::steady_clock::time_point session_start = std::chrono::steady_clock::now();
    while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - session_start).count() < session_duration_sec){
        //Process frame
        frame_start = std::chrono::steady_clock::now();
        mono_camera.get_frame_undistorted(&frame);
        SLAM.TrackMonocular(*frame, frame_number);

        //Save frame
        ss << setw(pad_to_width) << setfill('0') << frame_number;
        cv::imwrite(frame_dir+ ss.str() +".png", *frame);
        frame_number++;
        ss.str("");

        frame_end = std::chrono::steady_clock::now();
        frame_times.push_back(std::chrono::duration_cast<std::chrono::duration<double>>(frame_end - frame_start).count());
    }
    mono_camera.stop();
    delete frame;
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM(output_dir+session_dir+"mono_keyframe_trajectory.txt");
    SLAM.SaveMap(output_dir+session_dir+"mono_data.map");
    display_statistics(frame_times);
}

void start_stereo_slam(camera_utilities::Stereo_Camera &stereo_camera){
    std::string map_path = "";
    char load_map;
    std::cout << "LOAD MAP? <Y/N>: ";
    std::cin >> load_map;
    if(load_map=='Y' or load_map=='y'){
        map_path = output_dir+prev_session_dir+"stereo_data.map";
    }

    //Creating ORB object
    ORB_SLAM2::System SLAM(orb_voc_path, output_dir+session_dir+"stereo_camera.yaml", map_path, ORB_SLAM2::System::STEREO,true);
    //Determine naming schema for saved frames
    stringstream ss;
    int pad_to_width = to_string(session_duration_sec*100).length();
    int frame_number = 0;
    //Prepare variables for statistics tracking
    std::chrono::steady_clock::time_point frame_start;
    std::chrono::steady_clock::time_point frame_end;
    std::vector<double> frame_times;
    stereo_camera.start(true);
    std::chrono::steady_clock::time_point session_start = std::chrono::steady_clock::now();
    cv::Mat *left_frame, *right_frame;
    stereo_camera.init_frames(&left_frame, &right_frame);
    while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - session_start).count() < session_duration_sec){
        //Process frame
        frame_start = std::chrono::steady_clock::now();
        stereo_camera.get_stereo_frame_rectified(&left_frame, &right_frame);
        SLAM.TrackStereo(*left_frame, *right_frame, frame_number);
        //Save frame
        ss << setw(pad_to_width) << setfill('0') << frame_number;
        cv::imwrite(frame_dir + ss.str() + ".png", *left_frame);
        frame_number++;
        ss.str("");
        // update statistics
        frame_end = std::chrono::steady_clock::now();
        frame_times.push_back(std::chrono::duration_cast<std::chrono::duration<double>>(frame_end - frame_start).count());
    }
    delete left_frame;
    delete right_frame;

    //Close ORB object, save trajectory and map
    SLAM.Shutdown();
    stereo_camera.stop();
    SLAM.SaveTrajectoryTUM(output_dir+session_dir+"stereo_trajectory.txt");
    SLAM.SaveMap(output_dir+session_dir+"stereo_data.map");
    display_statistics(frame_times);
}

void start_stereo_disparity(camera_utilities::Stereo_Camera &stereo_camera){
	stereo_camera.create_disparity_matcher();
    cv::Mat frame;
	int key;
    cv::namedWindow("Disparity");
    while(true){
        stereo_camera.get_disparity(frame);
        cv::imshow("Disparity", frame*1000);
        key = cv::waitKey(1);
        if(key%256 == 27) //ESC pressed
            break;
    }
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
    std::cout << "Mean Frame Rate:\t" << (1/mean_frame_time) << std::endl;
    std::cout << "Median Frame Rate:\t" << (1/median_frame_time) << std::endl;
}

string load_previous_calibration(string camera_type){
    //Find the previous session directory
    stringstream ss;
    string prop_path;
    ss << setw(4) << setfill('0') << (session_num-1);
    prev_session_dir = "session_" + ss.str() + "/";
        
    //Check the previous session directory for a camera properties file
    prop_path = output_dir+prev_session_dir+ camera_type + "_camera_properties.yaml";
    std::ifstream old_camera_properties(prop_path, std::ios::binary);
    if(!old_camera_properties.is_open()){ //if one couldn't be found or couldn't be opened
        std::cout << "NO PREVIOUS SESSION. ENTER PATH TO CAMERA PROPERTIES FILE: ";
        std::cin >> prop_path; //prompt the user for the correct path
        old_camera_properties.open(prop_path);
        if(!old_camera_properties.is_open()) throw std::runtime_error("FAILED TO OPEN CAMERA PROPERTIES FILE"); //check the new path opened successfully
    }

    //Copy the old camera properties to the current session directory
    std::ofstream new_camera_properties(output_dir+session_dir+ camera_type + "_camera_properties.yaml", std::ios::binary);
    new_camera_properties << old_camera_properties.rdbuf();
    old_camera_properties.close();
    new_camera_properties.close();

    //Check the previous session directory for an ORB settings file
    string settings_path = output_dir+prev_session_dir+ camera_type + "_camera.yaml";
    std::ifstream old_ORB_settings(settings_path, std::ios::binary);
    if(!old_ORB_settings.is_open()){ //if one couldn't be found or couldn't be opened
        std::cout << "NO PREVIOUS SESSION. ENTER PATH TO ORB SETTINGS FILE: ";
        std::cin >> settings_path; //prompt the user for the correct path
        old_ORB_settings.open(settings_path);
        if(!old_ORB_settings.is_open()) throw std::runtime_error("FAILED TO OPEN PROPERTIES FILE"); //check the new path opened successfully
    }

    //Copy the old ORB settings to the current session directory
    std::ofstream new_ORB_settings(output_dir+session_dir + camera_type + "_camera.yaml", std::ios::binary);
    new_ORB_settings << old_ORB_settings.rdbuf();
    old_ORB_settings.close();
    new_ORB_settings.close();

    //Determine how long to run
    std::cout << "ENTER SESSION DURATION (SECONDS): ";
    std::cin >> session_duration_sec;

    return prop_path;
}