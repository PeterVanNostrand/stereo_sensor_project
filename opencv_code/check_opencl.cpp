#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(){
	cv::ocl::Context ctx = cv::ocl::Context::getDefault();
	if(!ctx.ptr()){
		cout << "OpenCL is not available" << endl;
	}
	cout << getBuildInformation() << endl;
	if (!cv::ocl::haveOpenCL())
{
    cout << "OpenCL is not available..." << endl;
    //return;
}

cv::ocl::Context context;
if (!context.create(cv::ocl::Device::TYPE_GPU))
{
    cout << "Failed creating the context..." << endl;
    //return;
}
else
    cout << "Context Created!" << std::endl;

cout << context.ndevices() << " GPU devices are detected." << endl; //This bit provides an overview of the OpenCL devices you have in your computer
for (int i = 0; i < context.ndevices(); i++)
{
    cv::ocl::Device device = context.device(i);
    cout << "name:              " << device.name() << endl;
    cout << "available:         " << device.available() << endl;
    cout << "imageSupport:      " << device.imageSupport() << endl;
    cout << "OpenCL_C_Version:  " << device.OpenCL_C_Version() << endl;
    cout << endl;
}
	return 0;
}
