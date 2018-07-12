#include<opencv2/core/core.hpp>
#include "System.h"
#include "camera_utilities.hpp"

using namespace std;

int main(){
    camera_utilities::Monocular_Camera camera(0, "./camera_properties.txt");// {6,8}, {640,480}, 15);
    camera.create_orbslam_settings("camera.yaml");
    ORB_SLAM2::System SLAM("/home/peter/ORB_SLAM2/Vocabulary/ORBvoc.txt","camera.yaml",ORB_SLAM2::System::MONOCULAR,true);
    cv::Mat frame = camera.get_frame();
    for(unsigned long int time_stamp=0;time_stamp<500;time_stamp++){
        SLAM.TrackMonocular(frame, time_stamp);
        frame = camera.get_frame();
    }
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}