#include <iostream>
#include<opencv2/core/core.hpp>
#include "../camera_utilities.hpp"

int main(){
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
        camera_utilities::Stereo_Camera stereo_camera(0, 1, "stereo_camera_properties.txt", {frame_width,frame_height}, fps, baseline, {chess_width,chess_height});
        const double stereo_orb_params[5] = {1000, 1.2, 8, 12, 7}; //reduced number of ORB features
        stereo_camera.create_orbslam_settings("stereo_camera.yaml", stereo_orb_params);

        cv::namedWindow("Left");
        cv::namedWindow("Right");
        cv::Mat frames[2];
        int key = cv::waitKey(1);
        while(key%256 != 27){
            stereo_camera.get_stereo_frame_rectified(frames);
            cv::imshow("Left", frames[0]);
            cv::imshow("Right", frames[1]);
            key = cv::waitKey(1);
        }
        cv::destroyAllWindows();
}