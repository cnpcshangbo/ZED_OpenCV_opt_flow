///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to use the ZED SDK with OpenCV. 					  	      **
 ** Depth and images are captured with the ZED SDK, converted to OpenCV format and displayed. **
 ***********************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
// OpenCV optical flow includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

// Sample includes
#include <SaveDepth.hpp>

// Mouse hover and click
#include "opencv2/highgui/highgui.hpp"

using namespace sl;
using namespace cv;
using namespace std;

cv::Mat slMat2cvMat(sl::Mat& input);
vector<Point2f> clicked_point;
void printHelp();

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if ( event == EVENT_LBUTTONDOWN )
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		clicked_point =  vector<Point2f>{cv::Point2f(x, y)};
	        cout << vector<Point2f>{cv::Point2f(x, y)} << endl;
	}
}

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD1080;
    init_params.depth_mode = DEPTH_MODE::ULTRA;
    init_params.coordinate_units = UNIT::METER;
    if (argc > 1) init_params.input.setFromSVOFile(argv[1]);

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_resolution;
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    sl::Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    sl::Mat point_cloud;

    //Optical flow related definitions
     // const string about =
     //     "This sample demonstrates Lucas-Kanade Optical Flow calculation.\n"
     //     "The example file can be downloaded from:\n"
     //     "  https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/slow_traffic_small.mp4";
     // const string keys =
     //     "{ h help |      | print this help message }"
     //     "{ @image | vtest.avi | path to image file }";
     // CommandLineParser parser(argc, argv, keys);
     // parser.about(about);
     // if (parser.has("help"))
     // {
     //     parser.printMessage();
     //     return 0;
     // }
     // string filename = samples::findFile(parser.get<string>("@image"));
     // if (!parser.check())
     // {
     //     parser.printErrors();
     //     return 0;
     // }
     // VideoCapture capture(filename);
     // if (!capture.isOpened()){
     //     //error in opening the video input
     //     cerr << "Unable to open file!" << endl;
     //     return 0;
     // }
     // Create some random colors
     vector<Scalar> colors;
     RNG rng;
     for(int i = 0; i < 100; i++)
     {
         int r = rng.uniform(0, 256);
         int g = rng.uniform(0, 256);
         int b = rng.uniform(0, 256);
         colors.push_back(Scalar(r,g,b));
     }
     cv::Mat old_frame, old_gray;
     vector<Point2f> p0, p1;
     // Take first frame and find corners in it
     //capture >> old_frame;
     old_frame = image_ocv.clone();
     cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
     goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
     
     //Creat a window for selecting point
     namedWindow("Click to select point to track",1);
     
     //set the callback function for mouse click event
     setMouseCallback("Click to select point to track", CallBackFunc, NULL);
     
     //show the image
     imshow("Click to select point to track", image_ocv);

     //p0 = clicked_point;
     //calc distances
     //double res = cv::norm(cv::Mat(clicked_point[0]),cv::Mat(p0[0]));
     
     //Point2f p = p0[0];
     //Point2f q = clicked_point[0];
     //Point2f diff = p - q;
     //float res = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
     
     cout << "distance: " << 0 << endl;

     //Wait until user press some key
     waitKey(0);

     // Create a mask image for drawing purposes
     cv::Mat mask = cv::Mat::zeros(old_frame.size(), old_frame.type());




    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {
      //optical flow algorithm related
      cv::Mat frame, frame_gray;


        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
            zed.retrieveImage(depth_image_zed, VIEW::DEPTH, MEM::CPU, new_image_size);

            // Retrieve the RGBA point cloud in half-resolution
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, new_image_size);

            // Display image and depth using cv:Mat which share sl:Mat data
            //cv::imshow("Image", image_ocv);
            //cv::imshow("Depth", depth_image_ocv);



                    //capture >> frame;
                    //This part is opticle flow
                    frame = image_ocv.clone();
                    if (frame.empty())
                        break;
                    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
                    // calculate optical flow
                    vector<uchar> status;
                    vector<float> err;
                    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
                    calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15,15), 2, criteria);
                    vector<Point2f> good_new;
                    for(uint i = 0; i < p0.size(); i++)
                    {
                        // Select good points
                        if(status[i] == 1) {
                            good_new.push_back(p1[i]);
                            // draw the tracks
                            line(mask,p1[i], p0[i], colors[i], 2);
                            circle(frame, p1[i], 5, colors[i], -1);
                        }
                    }
                    cv::Mat img;
                    add(frame, mask, img);
                    imshow("Frame", img);
                    int keyboard = waitKey(30);
                    if (keyboard == 'q' || keyboard == 27)
                        break;
                    // Now update the previous frame and previous points
                    old_gray = frame_gray.clone();
                    p0 = good_new;



            // Handle key event
            key = cv::waitKey(10);
            processKeyEvent(zed, key);
        }
    }
    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
