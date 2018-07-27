#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

cv::VideoCapture camera;

void test_resolution(int width, int height);

int main(){
    int camera_num;
    std::cout << "ENTER CAMERA NUMBER: ";
    std::cin >> camera_num;
    std::cout << "CREATING CAMERA" << std::endl;
    camera = cv::VideoCapture(camera_num);
    if(camera.isOpened())
        std::cout << "CAMERA CREATED!" << std::endl;
    else{
        std::cout << "ERROR CREATING CAMERA. EXITING..." << std::endl;
    }
    std::cout << "TESTING RESOLUTIONS" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    test_resolution(1920, 1080); //1080p
    test_resolution(1280, 720); //720p
    test_resolution(640, 480); //480p
    test_resolution(480, 360); //360p
    test_resolution(320, 240); //240p
    std::cout << "----------------------------------------" << std::endl;
    return 0;
}

void test_resolution(int width, int height){
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    int real_height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
    int real_width = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    std::vector<int> supported_fps;
    for(int fps=0; fps<=60; fps++){
        camera.set(cv::CAP_PROP_FPS, fps);
        int real_fps = camera.get(cv::CAP_PROP_FPS);
        if(real_fps == fps)
            supported_fps.push_back(fps);
    }
    std::cout << "\t\tWidth:\tHeight: " << std::endl;
    std::cout << "Desired:\t" << width << "\t" << height << std::endl;
    std::cout << "Actual:\t\t" << real_width << "\t" << real_height << std::endl;
    std::cout << "FPS:\t\t";
    for(unsigned int i=0; i<supported_fps.size(); i++)
        std::cout << supported_fps[i] << " ";
    std::cout << std::endl;
    std::cout << "----------------------------------------" << std::endl;
}