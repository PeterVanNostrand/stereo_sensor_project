//#include "System.h"
#include "Camera_Utilities.hpp"

using namespace std;

int main(){
    camera_utilities::Monocular_Camera camera(0, "./camera_properties.txt", {6,8}, {640,480}, 15);
    return 0;
}