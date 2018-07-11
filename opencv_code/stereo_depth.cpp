#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

//Camera properties
vector<VideoCapture> cameras;
int num_cameras = 2;
int frame_width = 640;
int frame_height = 480;
int fps = 15;

//Calibration parameters
vector<vector<vector<Point3f>>> objpoints;
vector<vector<vector<Point2f>>> imgpoints;
Size frame_size = {frame_width, frame_height};
vector<Mat> camera_matrices;
vector<Mat> dist_coeffs;
vector<vector<Mat>> rvecs;
vector<vector<Mat>> tvecs;
vector<double> reprojection_errors;
int flags;

//Chessboard properties
int chess_width;
int chess_height;
vector<Point3f> real_corners;

//Path to store all photos in
string foldername;
int session_num;
int img_count = 0;

//Cross calibration results
Mat R, T, E, F;
Mat C0M0, C0M1; //cam 0 map 0, cam 0 map 1
Mat C1M0, C1M1; //cam 1 map 0, cam 1 map 1

//Stereo matcher paramters
int num_disparities = 0;
int block_size = 21; //must be odd
int prefilter_cap = 31; //value of I_capp
int prefilter_size = 9; //limit of the window size for CV_STEREO_NORMALIZED_RESPONSE
int prefilter_type = 0; //type of algorithm used to smooth intensities, 0=CV_STEREO_BM_NORMALIZED_RESPONSE normalizes intensities
//1=CV_STEREO_BM_XSOBEL uses first order sobel derivatives in the x direction
int texture_threshold = 10;
int uniqueness_ratio = 15;

/*--------------------------------------------------- function stubs ---------------------------------------------------*/
void setup_chessboard();
bool get_data_set();
bool setup_cameras();
bool camera_capture();
bool load_capture(string folder);
void process_frames(vector<Mat> frames);
void calibrate_cameras();
void display_undistorted();
void cross_calibrate_cameras();
void capture_depth();
void generate_depth_map(vector<Mat> frames);
void save_camera_properites();
bool load_camera_properties();
void print_properties();
void print_bm_params();
void disp_change(int, void*);
void block_change(int, void*);
void pre_cap_change(int, void*);
void pre_size_change(int, void*);
/*------------------------------------------------ end function stubs ------------------------------------------------*/

int main(){
	//Determine whether to load a saved camera confgiuration or create one from a set of images
	char load_config;
	cout << "LOAD A PREVIOUS CONFIGURATION? <Y/N>:";
	cin >> load_config;
	if(load_config==89 || load_config==121){ //entered 'Y' or 'y' load config
		load_camera_properties();
		if(!setup_cameras()) return -1;
	}
	else if(load_config==78 || load_config==110){ //entered 'N' or 'n' create config from images
		if(!setup_cameras()) return -1;
		setup_chessboard();
		if(!get_data_set()) return -1;
		calibrate_cameras();
		cross_calibrate_cameras();
		display_undistorted();
	}
	else{ //User entered something other than Y, y, N, or n
		cout << "AN INVALID CHARACTER WAS ENTERED, EXITING" << endl;
		return -1;
	}
	//Take a depth image
	capture_depth();
	return 0;
}
/**
 *	Creates a coordinate plane matching the dimensions of the calibration chessboard as defined by the user, this is used during camera calibration as the "ground truth" to map 2D into 3D
 */
void setup_chessboard(){
	//Get chessbord information from user
		cout << "ENTER CHESSBOARD WIDTH: ";
		cin >> chess_width;
		cout << "ENTER CHESSBOARD HEIGHT: ";
		cin >> chess_height;
		
		//Creating appropriate coordinate plane for chessboard size
		for(int j=0; j<chess_height; ++j)
			for(int i=0; i<chess_width; ++i)
				real_corners.push_back(Point3f(i,j,0)); //Maps corners to x,y,z plane
}
/**
 *	Gathers sets of 2D->3D correspondences from stereo images of a calibration chessboard.  To be valid all image pairs must contain a complete view of the chessboard from both perspectives.
 *	Images may be loaded from a folder or captured with a camera depending on user input.  Images are validated and processed by the "load_capture" and "camera_capture" functions
 *	depending on image source.
 *	@return whether the capture process was successful
 */
bool get_data_set(){
	//Initializing img and obj point vectors
	for(int i=0; i<num_cameras; ++i){
		vector<vector<Point3f>> cam_objpoints;
		vector<vector<Point2f>> cam_imgpoints;
		objpoints.push_back(cam_objpoints);
		imgpoints.push_back(cam_imgpoints);
	}
	//Determing whether to take a new data set or use an old one
	char capture_new;
	cout << "CAPTURE NEW DATA SET? <Y/N>: ";
	cin >> capture_new;
	if(capture_new==89 || capture_new==121){ //entered 'Y' or 'y'
		//Creating appropriate directories{
		cout << "BUILDING DIRECTORIES..." << endl;
		mkdir("chessboards",0777); //Default permissions
		foldername = "./chessboards/session_0000/";
		while(mkdir(foldername.c_str(), 0777) == -1){ //Failed to create folder, already exits
			++session_num;
			stringstream ss;
			ss<< setw(4) << setfill('0') << session_num;
			foldername = "./chessboards/session_" + ss.str() + "/";
		}
		cout << "CREATED DIRECTORY " << foldername << endl;
		cout << "CAPTURING..." << endl;
		//Take new images
		if(!camera_capture()){
			cout << "CAPTURE PROCESS FAILED, EXITING" << endl;
			return false;
		}
	}
	else if(capture_new==78 || capture_new==110){ //entered 'N' or 'n'
		//Get the path to the saved data set and process it
		string folder;
		cout << "ENTER FOLDER PATH: ";
		cin >> foldername;
		cout << "CAPTURING..." << endl;
		if(!load_capture(foldername)){
			cout << "CAPTURE PROCESS FAILED, EXITING" << endl;
			return false;
		}
	}
	else{ //User entered something other than Y, y, N, or n
		cout << "AN INVALID CHARACTER WAS ENTERED, EXITING" << endl;
		return false;
	}
	cout << "CAPTURE COMPLETE!" << endl;
	return true;
}
/**
 *	Instanties camera objects and configures them for the desired frame rate and resolution specified by the global variables frame_width, frame_height, and fps
 *	@return whether all cameras were successfully configured
 */
bool setup_cameras(){
	//Open all cameras
	cout << endl << "--------------------------------------------------" << endl;
	cout << "INITIALIZING CAMERAS..." << endl;
	for(int i=0; i<num_cameras; ++i)
		cameras.push_back(VideoCapture(i));

	//Check that all cameras have opened successfully
	for(int i=0; i<num_cameras; ++i){
		if(cameras[i].isOpened())
			cout << "CAMERA " << i << " INITIALIZED" << endl;
		else{
			cout << "FAILED TO INITIALIZE CAMERA " << i << endl;
			cout << "--------------------------------------------------" << endl << endl;
			cout << "ONE OR MORE CAMERA IS NOT PROPERLY CONFIGURED, EXITING" << endl;
			return false;
		}
	}	
	
	//Print default camera properties
	cout << endl << "READING CAMERA DEFAULT PROPERTIES..." << endl;
	for(int i=0; i<num_cameras; ++i){
		int cam_width = cameras[i].get(CAP_PROP_FRAME_WIDTH);
		int cam_height = cameras[i].get(CAP_PROP_FRAME_HEIGHT);
		int cam_fps = cameras[i].get(CAP_PROP_FPS);
		cout << "CAMERA " << i << ": " << cam_width << "x" << cam_height << " " << cam_fps << "fps" << endl;
	}

	//Set all cameras to the desired resolution and framerate
	cout << endl << "SETTING CAMERA PARAMETERS..." << endl;
	for(int i=0; i<num_cameras; ++i){
		cameras[i].set(CAP_PROP_FRAME_WIDTH, frame_width);
		cameras[i].set(CAP_PROP_FRAME_HEIGHT, frame_height);
		cameras[i].set(CAP_PROP_FPS, fps);
	}

	//Check and print updated camera properties
	bool all_match = true;
	for(int i=0; i<num_cameras; ++i){
		int cam_width = cameras[i].get(CAP_PROP_FRAME_WIDTH);
		int cam_height = cameras[i].get(CAP_PROP_FRAME_HEIGHT);
		int cam_fps = cameras[i].get(CAP_PROP_FPS);
		cout << "CAMERA " << i << ": " << cam_width << "x" << cam_height << " " << cam_fps << "fps" << endl;
		if(cam_width != frame_width || cam_height!= frame_height || cam_fps != fps)
			all_match = false;
	}
	cout << "--------------------------------------------------" << endl << endl;
	return all_match;
}
/**
 *	Allows the user to capture a series of stereo images containing chessboards to be used for camera calibration. Press SPACE to capture an image or ESC to exit when finished capturing images.
 *	Frames are obtained using the "grab()" function and then decoded separately using the "retrieve()" function to ensure minimal latency between the left and right frames.
 *	Once obtained frames are processed by the "process_frames" function to extract calibration information and saved iff they contain chessboards
 *	@return whether frames were successfully captured from the cameras
*/
bool camera_capture(){
	//Create a window to display the feed from each camera
	for(int i=0; i<num_cameras; ++i)
		namedWindow("Camera " + to_string(i) + " Feed");
	
	vector<Mat> frames;
	bool grabbed_frame;
	int fail_count = 0;
	while(true && (fail_count < 25)){
		//Grab a frame from each camera
		for(int i=0; i<num_cameras; ++i){
			grabbed_frame = cameras[i].grab();
			if(!grabbed_frame){
				cout << "FAILED TO READ FRAME, TRYING AGAIN" << endl;
				frames.clear();
				++fail_count;
				continue;
			}
		}
		
		//Decode, store, and display the frame from each camera
		for(int i=0; i<num_cameras; ++i){
			Mat frame;
			cameras[i].retrieve(frame, 0);
			if(frame.empty()){
				cout << "FRAME WAS EMPTY, TRYING AGAIN" << endl;
				frames.clear();
				++fail_count;
				continue;
			}
			frames.push_back(frame);
			imshow("Camera " + to_string(i) + " Feed", frame);
		}

		int key = waitKey(1);
		if(key%256 == 27) //ESC was pressed, stop taking frames
			break;
		if(key%256 == 32) //SPACE was presed, process and save the frames
			process_frames(frames);
		frames.clear();
	}
	destroyAllWindows();
	return (fail_count < 25);
}
/**
 *	Loads a series of stereo images from the folder specified and proccesses them to extract calibration information using the "process_frames" function
 *	@param folder path to folder containing stereo image pairs
 *	@return whether images were loaded successfully
 */
bool load_capture(string folder){
	for(int i=0; i<num_cameras; ++i){
		int img_num = 0;
		string filename = "chessboard_0000_cam" + to_string(i) + ".png";
		Mat img = imread(folder + filename, IMREAD_UNCHANGED );
		while(!img.empty()){
			vector<Point2f> frame_corners;
			findChessboardCorners(img, {chess_width, chess_height}, frame_corners, 0);
			imgpoints[i].push_back(frame_corners);
			objpoints[i].push_back(real_corners);

			++img_num;
			stringstream ss;
			ss << setw(4) << setfill('0') << img_num;
			filename = "chessboard_" + ss.str() + "_cam" + to_string(i) + ".png";
			img = imread(folder + filename, IMREAD_UNCHANGED );
		}
		if(img_num==0)
			return false;
	}
	return true;
}
/**
 *	Analyzes the pair of frames in search of a chessboard pattern. If a chessboard is found in both images the location of the chessboard's interior corners
 *	are added to the global variable "imgpoints" and the corresponding 3D coordinates from "real_corners" are added to the global variable "objpoints," then
 *	the images are saved. These values in "imgpoints" and "objpoints" represent a 2D to 3D mapping to be used in camera calibration.
 *	If a chessbord is not found in both images the pair is discarded and no data is saved.
 *	@param frames a pair of matrices representing a stereo image of a chessboard
 */
void process_frames(vector<Mat> frames){
	/*For each frame a vector<Point2f> represents the location of the chessboard corners in the image. "corners" stores 
	one of these vectors for each camera. Data is added to the data set iff all camera can see the chessboard*/
	bool all_found_chessboards = true;
	vector<vector<Point2f>> corners;
	for(int i=0; i<num_cameras; ++i){
		vector<Point2f> frame_corners;
		cvtColor(frames[i], frames[i], COLOR_BGR2GRAY);
		all_found_chessboards &= findChessboardCorners(frames[i], {chess_width, chess_height}, frame_corners, 0); //locate chesboard corners
		TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
		cornerSubPix(frames[i] , frame_corners, {5,5}, {-1,-1}, termcrit); //refine the location of the chessboard corners
		corners.push_back(frame_corners);
	}

	//If a chessboard was not found in both images return
	if(!all_found_chessboards){
		cout << "ONE OR MORE IMAGES DID NOT CONTAIN A CHESSBOARD, DISCARDING" << endl;
		return;
	}

	//Add the 2D and 3D points to the data set
	for(int i=0; i<num_cameras; ++i){
		imgpoints[i].push_back(corners[i]);
		objpoints[i].push_back(real_corners);
		Mat frame_drawn = frames[i].clone();
		drawChessboardCorners(frame_drawn, {chess_width,chess_height}, corners[i], true);
		imshow("Camera " + to_string(i) + " Feed", frame_drawn);
		waitKey(500);
	}

	//Save the images
	for(int i=0; i<num_cameras; ++i){
		stringstream ss;
		ss << setw(4) << setfill('0') << img_count;
		string filename = "chessboard_" + ss.str() + "_cam" + to_string(i) + ".png";
		string path = foldername+filename;
		imwrite(path, frames[i]);
	}
	++img_count;
}
/**
 *	Uses the 2D->3D correspondences gathered by "get_data_set" to characerize the physical properties of each camera.  This allows later images to correct for distortion
 *	and map 2D points into 3D space.
 */
void calibrate_cameras(){
	cout << "CALIBRATING..." << endl;
	for(int i=0; i<num_cameras; ++i){
		Mat cam_matrix;
		Mat cam_dist_coeffs;
		vector<Mat> cam_rvecs;
		vector<Mat> cam_tvecs;
	    double cam_reproj_error = calibrateCamera(objpoints[i], imgpoints[i], {chess_width,chess_height}, cam_matrix, cam_dist_coeffs, cam_rvecs, cam_tvecs, 0);
		camera_matrices.push_back(cam_matrix);
		dist_coeffs.push_back(cam_dist_coeffs);
		rvecs.push_back(cam_rvecs);
		tvecs.push_back(cam_tvecs);
		reprojection_errors.push_back(cam_reproj_error);
	}
}
/**
 *	Uses the values generated by "calibrate_cameras" to undistort and display the chessboard images captured by "get_data_set."  
 *	This correction removes any 'fisheye' effect imparted by the camera lens.
 *	The user may skip this process by entering 'n' when prompted
 */
void display_undistorted(){
	char display_undist;
	cout << "SHOW UNDISTORTED? <Y/N>: ";
	cin >> display_undist;
	if(!(display_undist==89 || display_undist==121)  && !(display_undist==78 || display_undist==110)){ //if not Y, y, N, or n
		cout << "AN INVALID CHARACTER WAS ENTERED, SKIPPING DISPLAY" << endl;
		return ;
	}
	if(display_undist==78 || display_undist==110){ //entered 'N' or 'n'
		cout << "SKIPPING DISPLAY..." << endl;
		return;
	}
	//User entered Y or y, display the undistorted images
	namedWindow("Origional");
	namedWindow("Undistorted");
	for(int i=0; i<num_cameras; ++i){
		int img_num = 0;
		string filename = "chessboard_0000_cam" + to_string(i) + ".png";
		Mat img = imread(foldername + filename, IMREAD_UNCHANGED );
		while(!img.empty()){
			Mat img_undistorted;
			undistort(img, img_undistorted, camera_matrices[i], dist_coeffs[i], camera_matrices[i]);
			imshow("Origional",img);
			imshow("Undistorted", img_undistorted);
			waitKey(0);
			++img_num;
			stringstream ss;
			ss << setw(4) << setfill('0') << img_num;
			filename = "chessboard_" + ss.str() + "_cam" + to_string(i) + ".png";
			img = imread(foldername + filename, IMREAD_UNCHANGED );
		}
	}
	destroyWindow("Origional");
	destroyWindow("Undistorted");
}
/**
 *	Uses the camera matrices and distortion coefficients for each camera (found by "calibrate_camera") to create a set of remapping matrices C0M0, C0M1, C1M0, and C1M1
 *	These matrices represent a set of transforms for each camera that rotate and translate images such that the left and right perspectives are row aligned,
 *	i.e. if the left and right frames are remapped and placed side by side, every point in the left image will be in the same row of pixels as the corresponding point in the right image.
 *	This allows the use of epipolar geometry when determing depth via block matching.
 */
void cross_calibrate_cameras(){
	stereoCalibrate(objpoints[0], imgpoints[0], imgpoints[1], camera_matrices[0], dist_coeffs[0], camera_matrices[1], dist_coeffs[1], frame_size, R, T, E, F, CALIB_FIX_INTRINSIC);
	Mat R0, R1, P0, P1, Q;
	stereoRectify(camera_matrices[0], dist_coeffs[0], camera_matrices[1], dist_coeffs[1], frame_size, R, T, R0, R1, P0, P1, Q, CALIB_ZERO_DISPARITY, 0);
	initUndistortRectifyMap(camera_matrices[0], dist_coeffs[0], R0, P0, frame_size, CV_32FC1, C0M0, C0M1);
	initUndistortRectifyMap(camera_matrices[1], dist_coeffs[1], R1, P1, frame_size, CV_32FC1, C1M0, C1M1);
}
void capture_depth(){
	//Create a window to display the feed from each camera
	for(int i=0; i<num_cameras; ++i)
		namedWindow("Camera " + to_string(i) + " Feed");
	//namedWindow("Depth Map UP 100");
	namedWindow("Depth Map UP 1000");
	//namedWindow("Depth Map UP 10000");
	namedWindow("StereoBM Parameters", CV_GUI_NORMAL);

	createTrackbar("num_disparities", "StereoBM Parameters", &num_disparities, 256, disp_change);
	createTrackbar("block_size", "StereoBM Parameters", &block_size, 41, block_change);
	createTrackbar("prefilter_cap", "StereoBM Parameters", &prefilter_cap, 63, pre_cap_change);
	createTrackbar("prefilter_size", "StereoBM Parameters", &prefilter_size, 100, pre_size_change);
	createTrackbar("prefilter_type", "StereoBM Parameters", &prefilter_type, 1);
	createTrackbar("texture_threshold", "StereoBM Parameters", &texture_threshold, 50);
	createTrackbar("uniqueness_ratio", "StereoBM Parameters", &uniqueness_ratio, 50);
				   
	vector<Mat> frames;
	bool grabbed_frame;
	int fail_count = 0;
	while(fail_count < 25){
		//Grab a frame from each camera
		for(int i=0; i<num_cameras; ++i){
			grabbed_frame = cameras[i].grab();
			if(!grabbed_frame){
				cout << "FAILED TO READ FRAME, TRYING AGAIN" << endl;
				frames.clear();
				++fail_count;
				continue;
			}
		}
		
		//Decode, store, and display the frame from each camera
		for(int i=0; i<num_cameras; ++i){
			Mat frame;
			cameras[i].retrieve(frame, 0);
			if(frame.empty()){
				cout << "FRAME WAS EMPTY, TRYING AGAIN" << endl;
				frames.clear();
				++fail_count;
				continue;
			}
			frames.push_back(frame);
			imshow("Camera " + to_string(i) + " Feed", frame);
		}

		int key = waitKey(1);
		if(key%256 == 27) //ESC was pressed, stop taking frames
			break;
		if(key%256 == 115) //'s' was pressed, save camera config
			save_camera_properites();
		if(key%256 == 112) //'p' was pressed, save camera config
			print_bm_params();
		//if(key%256 == 32) //SPACE was presed, process the frames
			generate_depth_map(frames);
		frames.clear();
	}
	if(fail_count >= 25){
		cout << "ERROR, EXITING" << endl;
		return;
	}
	destroyAllWindows();
}
void generate_depth_map(vector<Mat> frames){
	for(int i=0; i<num_cameras; ++i)
		cvtColor(frames[i], frames[i], COLOR_BGR2GRAY);
	Mat fixed0, fixed1, depth_map;
	remap(frames[0], fixed0, C0M0, C0M1, INTER_LINEAR);
	remap(frames[1], fixed1, C1M0, C1M1, INTER_LINEAR);
	Ptr<StereoBM> matcher = StereoBM::create(num_disparities, block_size);
	matcher->setPreFilterType(prefilter_type);
	matcher->setPreFilterSize(prefilter_size);
	matcher->setPreFilterCap(prefilter_cap);
	matcher->setTextureThreshold(texture_threshold);
	matcher->setUniquenessRatio(uniqueness_ratio);
	matcher->compute(fixed0, fixed1, depth_map);
	// cout << "PreFilterCap=" << matcher->getPreFilterCap() << endl;
	// cout << "PreFilterSize=" << matcher->getPreFilterSize() << endl;
	// cout << "PreFilterType=" << matcher->getPreFilterType() << endl;
	// cout << "SmallerBlockSize=" << matcher->getSmallerBlockSize() << endl;
	// cout << "TextureThreshold=" << matcher->getTextureThreshold() << endl;
	// cout << "UniquenessRatio=" << matcher->getUniquenessRatio() << endl;
	// imshow("Frame 0", frames[0]);->
	// imshow("Frame 1", frames[1]);
	// imshow("Fixed 0", fixed0);
	// imshow("Fixed 1", fixed1);
	// imshow("Fixed 0", fixed0);
	// imshow("Fixed 1", fixed1);
	// imshow("Depth Map", depth_map);
	// imshow("Depth Map DWN", depth_map/2048);
	// imshow("Depth Map UP 100", depth_map*100);
	imshow("Depth Map UP 1000", depth_map*1000);
	//imshow("Depth Map UP 10000", depth_map*10000);
	waitKey(1);
}
/**
 *	Writes the camera configuration, camera matrices, distortion coefficients, rotation matrix, and translation matrix to a file for later use.
 *	This allows the user to reload these values later and skip the calibration step.
 */
void save_camera_properites(){
	cout << "SAVING CAMERA PROPERTIES..." << endl;
	FileStorage fs("filename.txt", FileStorage::WRITE);
	fs << "num_cameras" << num_cameras;
	fs << "frame_width" << frame_width;
	fs << "frame_height" << frame_height;
	fs << "fps" << fps;
	fs << "c0_matrix" << camera_matrices[0] ;
	fs << "c0_dist_coeffs" << dist_coeffs[0];
	fs << "c1_matrix" << camera_matrices[1] ;
	fs << "c1_dist_coeffs" << dist_coeffs[1] ;
	fs << "R" << R ;
	fs << "T" << T ;
}
/**
 *	Reads the camera configuration, camera matrices, distortion coefficients, rotation matrix, and translation matrix from a file allowing the user to skip the calibration step.
 *	These values are then used to genereate the remapping matrices C0M0, C0M1, C1M0, and C1M1 needed for depth calculation.
 *	Note that if the camera setup has changed since this data was saved the resulting remapping matrices will not be accurate and calibration will need to be repeated.
 *	@return whether the save file was successfully opened
 */
bool load_camera_properties(){
	FileStorage fs("camera_properties.txt", FileStorage::READ);
	if (!fs.isOpened())
    	return false;
	fs["num_cameras"] >> num_cameras;
	fs["frame_width"] >> frame_width;
	fs["frame_height"] >> frame_height;
	fs["fps"] >> fps;
	Mat cm0, dc0, cm1, dc1;
	fs["c0_matrix"] >> cm0;
	fs["c0_dist_coeffs"] >> dc0;
	fs["c1_matrix"] >> cm1;
	fs["c1_dist_coeffs"] >> dc1;
	camera_matrices.push_back(cm0);
	camera_matrices.push_back(cm1);
	dist_coeffs.push_back(dc0);
	dist_coeffs.push_back(dc1);
	fs["R"] >> R ;
	fs["T"] >> T ;
	Mat R0, R1, P0, P1, Q;
	frame_size = {frame_width, frame_height};
	//CALIB_ZERO_DISPARITY = use already determined camera matrices
	stereoRectify(camera_matrices[0], dist_coeffs[0], camera_matrices[1], dist_coeffs[1], frame_size, R, T, R0, R1, P0, P1, Q, CALIB_ZERO_DISPARITY, 0);
	initUndistortRectifyMap(camera_matrices[0], dist_coeffs[0], R0, P0, frame_size, CV_32FC1, C0M0, C0M1);
	initUndistortRectifyMap(camera_matrices[1], dist_coeffs[1], R1, P1, frame_size, CV_32FC1, C1M0, C1M1);
	return true;
}
/**
 *	Prints the camera configuration, camera matrices, distortion coefficients, rotation matrix, and translation matrix for examination by the user.
 *	Used for debugging only
 */
void print_properties(){
		cout << "num_cameras=" << num_cameras << endl;
		cout << "frame_width=" << frame_width << endl;
		cout << "frame_height=" << frame_height << endl;
		cout << "fps=" << fps << endl;
		cout << "c0_matrix=" << camera_matrices[0] << endl;
		cout << "c0_dist_coeffs=" << dist_coeffs[0] << endl;
		cout << "c1_matrix=" << camera_matrices[1] << endl;
		cout << "c1_dist_coeffs=" << dist_coeffs[1] << endl;
		cout << "R=" << R << endl;
		cout << "T=" << T << endl;
}
/**
 *	Prints the block matching parameters for use in debugging
 */
void print_bm_params(){
	cout <<"num_disparities=" << num_disparities << endl;
	cout << "block_size=" << block_size << endl;
	cout << "prefilter_cap" << prefilter_cap << endl;
	cout << "prefilter_size=" << prefilter_size << endl;
	cout << "prefilter_type" << prefilter_type << endl;
	cout << "texture_threshold=" << texture_threshold << endl;
	cout << "uniqueness_ratio=" << uniqueness_ratio << endl;
}
/*--------------------------------------------------- trackar functions ---------------------------------------------------*/
void disp_change(int, void*){
	num_disparities = (num_disparities/16)*16; //must be positive and divisible by 16
}
void block_change(int, void*){
	if(block_size<5) block_size=5; //must be odd and in range 5-255
	block_size = ((block_size/2)*2) + 1;
}
void pre_cap_change(int, void*){
	if(prefilter_cap<1) prefilter_cap=1; //must be in range 1-63
}
void pre_size_change(int, void*){
	if(prefilter_size<5) prefilter_size=5; //must be odd and in range 5-255
	prefilter_size = ((prefilter_size/2)*2) + 1;
}
/*------------------------------------------------ end trackar functions ------------------------------------------------*/