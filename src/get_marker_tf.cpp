/* Copyright by Huong Do Van - 10/11/2018
Any suggestion or advice, pls send via email: vanhuong.robotics@gmail.com
*/
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include<sstream>
// Add new topic
#include "geometry_msgs/Point.h"
#include <iostream>
#include <vector>
#include <cmath>
// Create new for circle drawing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

// Add global variable for pixel.
bool flag;
int posX[4]; 
int posY[4]; 

float X_111[4];
float Y_111[4];
float Z_111[4];
float x_value, y_value, z_value;
int x_position[4], y_position[4], z_position[4];
//End of global variable.
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::PointCloud2 my_pcl;
std::string color[4] = {"blue", 
                        "green", 
                        "red", 
                        "yellow"}; 

using namespace std;
using namespace cv;
int H_MIN_BLUE = 0;
int H_MAX_BLUE = 256;
int S_MIN_BLUE = 0;
int S_MAX_BLUE = 256;
int V_MIN_BLUE = 0;
int V_MAX_BLUE = 256;

int H_MIN_GREEN = 0;
int H_MAX_GREEN = 256;
int S_MIN_GREEN = 0;
int S_MAX_GREEN = 256;
int V_MIN_GREEN = 0;
int V_MAX_GREEN = 256;

int B_MIN_RED = 0;
int B_MAX_RED = 256;
int G_MIN_RED = 0;
int G_MAX_RED = 256;
int R_MIN_RED = 0;
int R_MAX_RED = 256;

int H_MIN_YELLOW = 0;
int H_MAX_YELLOW = 256;
int S_MIN_YELLOW = 0;
int S_MAX_YELLOW = 256;
int V_MIN_YELLOW = 0;
int V_MAX_YELLOW = 256;

int max_idx_color = 3; 
int idx_color = 0; 

//Publishe new topic
ros::Publisher *pub;
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS = 50;
const int MIN_OBJECT_AREA = 1;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

const int BLUE = 0; 
const int GREEN = 1;
const int RED = 2;
const int YELLOW = 3;

static const std::string OPENCV_WINDOW = "Image Window";
static const std::string windowName1 = "HSV image";
static const std::string windowName2 = "blue Thresholded Image";
static const std::string windowName5 = "red Thresholded Image";
static const std::string windowName3 = "After Morphological Operations";
static const std::string trackbarWindowName = "Track bars";
//for mouse event//
//bool mouse_is_pressing = false;   // 왼쪽 마우스 버튼 상태 체크를 위해 사용

int color_margin = 10;

int start_x = -1;
int start_y = -1; //최초로 왼쪽 마우스 버튼 누른 위치를 저장하기 위해 사용
int end_x = -1; 
int end_y = -1; 

bool track[4]; 

///////////////////
void mouse_callback(int event, int x, int y, int flags, void *userdata){
	

    if (event == EVENT_LBUTTONDOWN) {
        //mouse_is_pressing = true;
        start_x = x;
        start_y = y; 

    }
    else if (event == EVENT_LBUTTONUP) { 
        end_x = x; 
        end_y = y; 
        


        int ulx = std::min(start_x, end_x); 
        int uly = std::min(start_y, end_y);

        int drx = std::max(start_x, end_x);
        int dry = std::max(start_y, end_y);

        int width = drx - ulx;
        int height =  dry - uly;

        if(width == 0 || height == 0)
        return;

        cout << "start : " << ulx <<','<< uly <<endl; 
        cout << "end : " << dry <<','<< drx <<endl;
        


        Mat roi(cv_ptr->image, Rect(ulx, uly, drx - ulx, dry - uly));


        
        unsigned char tempmin[3] = {255,255,255};
        unsigned char tempmax[3] = {0  ,0  ,0  }; 
        
        if(idx_color != RED){
                cvtColor(roi, roi, COLOR_BGR2HSV);
        }
        for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++)
        for(int k = 0; k < 3; k++){
                tempmin[k] = min(tempmin[k], roi.at<Vec3b>(i,j)[k]); 
                tempmax[k] = max(tempmax[k], roi.at<Vec3b>(i,j)[k]); 
        }
        for(int i = 0; i < 3; i++){
                tempmin[i] *= (1 - (float)color_margin/100); 
                tempmax[i] += ((255 - tempmax[i]) * (float)color_margin/100); 
        }


        for(int i = 0; i < 3; i++){
                cout << (int)tempmin[i] <<','<< (int)tempmax[i] << endl;
        }
        
        if(idx_color == BLUE){
                setTrackbarPos("H_MIN_BLUE"   , "Trackbars" , tempmin[0]);                    
                setTrackbarPos("H_MAX_BLUE"   , "Trackbars" , tempmax[0]);        
                setTrackbarPos("S_MIN_BLUE"   , "Trackbars" , tempmin[1]);        
                setTrackbarPos("S_MAX_BLUE"   , "Trackbars" , tempmax[1]); 
                setTrackbarPos("V_MIN_BLUE"   , "Trackbars" , tempmin[2]);  
                setTrackbarPos("V_MAX_BLUE"   , "Trackbars" , tempmax[2]); 
                
        }
        if(idx_color == GREEN){
                setTrackbarPos("H_MIN_GREEN"   , "Trackbars" , tempmin[0]);
                setTrackbarPos("H_MAX_GREEN"   , "Trackbars" , tempmax[0]);
                setTrackbarPos("S_MIN_GREEN"   , "Trackbars" , tempmin[1]);
                setTrackbarPos("S_MAX_GREEN"   , "Trackbars" , tempmax[1]);
                setTrackbarPos("V_MIN_GREEN"   , "Trackbars" , tempmin[2]);
                setTrackbarPos("V_MAX_GREEN"   , "Trackbars" , tempmax[2]);
        }
        if(idx_color == RED){
                setTrackbarPos("B_MIN_RED"   , "Trackbars" , tempmin[0]);
                setTrackbarPos("B_MAX_RED"   , "Trackbars" , tempmax[0]);
                setTrackbarPos("G_MIN_RED"   , "Trackbars" , tempmin[1]);
                setTrackbarPos("G_MAX_RED"   , "Trackbars" , tempmax[1]);
                setTrackbarPos("R_MIN_RED"   , "Trackbars" , tempmin[2]);
                setTrackbarPos("R_MAX_RED"   , "Trackbars" , tempmax[2]);
        }
        if(idx_color == YELLOW){
                setTrackbarPos("H_MIN_YELLOW"   , "Trackbars" , tempmin[0]);
                setTrackbarPos("H_MAX_YELLOW"   , "Trackbars" , tempmax[0]);
                setTrackbarPos("S_MIN_YELLOW"   , "Trackbars" , tempmin[1]);
                setTrackbarPos("S_MAX_YELLOW"   , "Trackbars" , tempmax[1]);
                setTrackbarPos("V_MIN_YELLOW"   , "Trackbars" , tempmin[2]);
                setTrackbarPos("V_MAX_YELLOW"   , "Trackbars" , tempmax[2]); 
        }
        track[idx_color] = true; 
        imshow("roi", roi);
        
    }
}



void on_trackbar(int, void*){}
string intToString(int number)
{
        std::stringstream ss;
        ss << number;
        return ss.str();
}


void createTrackbars()
{
        //Create window for trackbars
        namedWindow("Trackbars", 0);
        char TrackbarName[50];
        createTrackbar("H_MIN_BLUE"     , "Trackbars", &H_MIN_BLUE, H_MAX_BLUE, on_trackbar);
        createTrackbar("H_MAX_BLUE"     , "Trackbars", &H_MAX_BLUE, H_MAX_BLUE, on_trackbar);
        createTrackbar("S_MIN_BLUE"     , "Trackbars", &S_MIN_BLUE, S_MAX_BLUE, on_trackbar);
        createTrackbar("S_MAX_BLUE"     , "Trackbars", &S_MAX_BLUE, S_MAX_BLUE, on_trackbar);
        createTrackbar("V_MIN_BLUE"     , "Trackbars", &V_MIN_BLUE, V_MAX_BLUE, on_trackbar);
        createTrackbar("V_MAX_BLUE"     , "Trackbars", &V_MAX_BLUE, V_MAX_BLUE, on_trackbar);
        createTrackbar("H_MIN_GREEN"    , "Trackbars", &H_MIN_GREEN, H_MAX_GREEN, on_trackbar);
        createTrackbar("H_MAX_GREEN"    , "Trackbars", &H_MAX_GREEN, H_MAX_GREEN, on_trackbar);
        createTrackbar("S_MIN_GREEN"    , "Trackbars", &S_MIN_GREEN, S_MAX_GREEN, on_trackbar);
        createTrackbar("S_MAX_GREEN"    , "Trackbars", &S_MAX_GREEN, S_MAX_GREEN, on_trackbar);
        createTrackbar("V_MIN_GREEN"    , "Trackbars", &V_MIN_GREEN, V_MAX_GREEN, on_trackbar);
        createTrackbar("V_MAX_GREEN"    , "Trackbars", &V_MAX_GREEN, V_MAX_GREEN, on_trackbar);
        createTrackbar("B_MIN_RED"      , "Trackbars", &B_MIN_RED, B_MAX_RED, on_trackbar);
        createTrackbar("B_MAX_RED"      , "Trackbars", &B_MAX_RED, B_MAX_RED, on_trackbar);
        createTrackbar("G_MIN_RED"      , "Trackbars", &G_MIN_RED, G_MAX_RED, on_trackbar);
        createTrackbar("G_MAX_RED"      , "Trackbars", &G_MAX_RED, G_MAX_RED, on_trackbar);
        createTrackbar("R_MIN_RED"      , "Trackbars", &R_MIN_RED, R_MAX_RED, on_trackbar);
        createTrackbar("R_MAX_RED"      , "Trackbars", &R_MAX_RED, R_MAX_RED, on_trackbar);
        createTrackbar("H_MIN_YELLOW"   , "Trackbars", &H_MIN_YELLOW, H_MAX_YELLOW, on_trackbar);
        createTrackbar("H_MAX_YELLOW"   , "Trackbars", &H_MAX_YELLOW, H_MAX_YELLOW, on_trackbar);
        createTrackbar("S_MIN_YELLOW"   , "Trackbars", &S_MIN_YELLOW, S_MAX_YELLOW, on_trackbar);
        createTrackbar("S_MAX_YELLOW"   , "Trackbars", &S_MAX_YELLOW, S_MAX_YELLOW, on_trackbar);
        createTrackbar("V_MIN_YELLOW"   , "Trackbars", &V_MIN_YELLOW, V_MAX_YELLOW, on_trackbar);
        createTrackbar("V_MAX_YELLOW"   , "Trackbars", &V_MAX_YELLOW, V_MAX_YELLOW, on_trackbar);
        
        createTrackbar("0:B 1:G 2:R 3:Y", "Trackbars", &idx_color, max_idx_color, on_trackbar);

        createTrackbar("color margin", "Trackbars", &color_margin, 100, on_trackbar);

}


void drawObject(int x, int y, Mat &frame, int i)
{
        circle(frame, Point(x, y), 40, Scalar(0, 255, 0), 2); //50
        if (y - 25 > 0)
                line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
        if (y + 25 < FRAME_HEIGHT)
                line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
        if (x - 25 > 0)
                line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
        if (x + 25 < FRAME_WIDTH)
                line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

        //putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
        ::posX[i] = x;
        ::posY[i] = y;
        //putText(frame, "X_Y_Z coordinate", Point(20, 200), 1, 2, Scalar(0, 255, 0), 2);
        //putText(frame, "X = " + intToString(x_position) + "(mm)" , Point(20, 250), 1, 2, Scalar(0, 255, 0), 2);
        //putText(frame, "Y = " + intToString(y_position) + "(mm)" , Point(20, 300), 1, 2, Scalar(0, 255, 0), 2);
        //putText(frame, "Z = " + intToString(z_position) + "(mm)" , Point(20, 350), 1, 2, Scalar(0, 255, 0), 2);

}
void morphOps(Mat &thresh)
{
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);

        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
}
void trackFilteredObject(Mat b_threshold, Mat g_threshold, Mat r_threshold, Mat y_threshold, Mat &cameraFeed)
{
        Mat temp[4]; 
        b_threshold.copyTo(temp[0]);
        g_threshold.copyTo(temp[1]);
        r_threshold.copyTo(temp[2]);
        y_threshold.copyTo(temp[3]);
        
        int x[4];
        int y[4]; 

        for(int i = 0; i < 4; i++){
                x[i] = 0; 
                y[i] = 0;
        }
        //These two vectors needed for output of findContours
        vector< vector<Point> > contours[4];
        vector<Vec4i> hierarchy[4];
        
        //Find contours of filtered image using openCV findContours function
        for(int i = 0 ; i < 4; i++) findContours(temp[i], contours[i], hierarchy[i], CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        
        
        //Use moments method to find our filtered object
        
        bool objectFound = false;
        for(int i = 0; i < 4; i++){
                double refArea = 0;
        if (hierarchy[i].size() > 0)
        {
                int numObjects = hierarchy[i].size();
                //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
                if (true)//(numObjects < MAX_NUM_OBJECTS)
                {
                        for (int index = 0; index >= 0; index = hierarchy[i][index][0])
                        {
                                Moments moment = moments((cv::Mat)contours[i][index]);
                                double area = moment.m00;
                                //if the area is less than 20 px by 20px then it is probably just noise
                                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                                //we only want the object with the largest area so we safe a reference area each
                                //iteration and compare it to the area in the next iteration.
                                if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                                {
                                        x[i] = moment.m10 / area;
                                        y[i] = moment.m01 / area;
                                        objectFound = true;
                                        ::flag = true;
                                        refArea = area;
                                }
                                else
                                {
                                 objectFound = false;
                                 ::flag = false;
                                }
                        }
                        //let user know you found an object
                        if (true)// objectFound == true)
                        {
                                //::flag = true;
                                //putText(cameraFeed, "Position Object tracking ", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                                //draw object location on screen
                                drawObject(x[i], y[i], cameraFeed, i);
                        }

                }
                else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
        }
        ::flag = true;//posx posy 배열을 건드릴수 있게 수정해야함
        }
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        //cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        bool trackObjects = true; //false
	bool useMorphOps = false;  //false

	Mat HSV;
        Mat RGB; 
	Mat b_threshold;
        Mat g_threshold; 
        Mat r_threshold;
        Mat y_threshold;

	int x = 0, y = 0;
	
        //std::cout << " The output of Object tracking by OpenCV!\n";

        cvtColor(cv_ptr->image, HSV, COLOR_BGR2HSV);
        cv_ptr->image.copyTo(RGB); 
        inRange(HSV, Scalar(H_MIN_BLUE      , S_MIN_BLUE    , V_MIN_BLUE)   , Scalar(H_MAX_BLUE     , S_MAX_BLUE    , V_MAX_BLUE)   , b_threshold);
        inRange(RGB, Scalar(B_MIN_RED       , G_MIN_RED     , R_MIN_RED)    , Scalar(B_MAX_RED      , G_MAX_RED     , R_MAX_RED)    , r_threshold);
        inRange(HSV, Scalar(H_MIN_GREEN     , S_MIN_GREEN   , V_MIN_GREEN)  , Scalar(H_MAX_GREEN    , S_MAX_GREEN   , V_MAX_GREEN)  , g_threshold);
        inRange(HSV, Scalar(H_MIN_YELLOW    , S_MIN_YELLOW  , V_MIN_YELLOW) , Scalar(H_MAX_YELLOW   , S_MAX_YELLOW  , V_MAX_YELLOW) , y_threshold);

    if (useMorphOps){
        morphOps(b_threshold);
        morphOps(g_threshold);
        morphOps(r_threshold);
        morphOps(y_threshold);
    }
        
    if (trackObjects)
        trackFilteredObject(b_threshold, g_threshold, r_threshold, y_threshold,  cv_ptr->image);
    //show frames
    imshow(windowName2                  , b_threshold);
    imshow("red thresholded image"      , r_threshold); 
    imshow("yellow thresholded image"   , y_threshold); 
    imshow("green thresholded image"    , g_threshold); 
    //imshow(OPENCV_WINDOW, cameraFeed);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //imshow(windowName1, HSV);
    cv::waitKey(3);

}

void getXYZ(int x, int y, int i) //i : 0123 = color : bgry
{
    int arrayPosition = y*my_pcl.row_step + x*my_pcl.point_step;
    int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &my_pcl.data[arrayPosX], sizeof(float));
    memcpy(&Y, &my_pcl.data[arrayPosY], sizeof(float));
    memcpy(&Z, &my_pcl.data[arrayPosZ], sizeof(float));

    ::X_111[i] = X;
    ::Y_111[i] = Y;
    ::Z_111[i] = Z;
    ::x_position[i] = int(X*1000);
    ::y_position[i] = int(Y*1000);
    ::z_position[i] = int(Z*1000);

    //printf("Position in X coordinate X = %.4f\n", X);
    //printf("Position in Y coordinate Y = %.4f\n", Y);
    //printf("Position in Z coordinate Z = %.4f\n", Z);

    return; 
}

void depthcallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    my_pcl = *cloud_msg;
    for(int i = 0; i < 4; i++)getXYZ(posX[i] , posY[i], i);
}

double dotp(double x1, double y1, double z1, double x2, double y2, double z2){
        return x1 * x2 + y1 * y2 + z1 * z2; 
}

int main(int argc, char** argv)
{
        namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);
        
        createTrackbars();
        ros::init (argc, argv, "color_tracker");
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Vector3 origin;
        tf::Matrix3x3 tf3d; 

        //transform.setOrigin( tf::Vector3(2, 2, 2) );
        tf::Quaternion q;
        q.setRPY(0,0,0); 
        transform.setRotation(q);

        
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw",1, imageCallback);
        ros::Subscriber dep;
        dep = nh.subscribe ("/camera/depth_registered/points", 1, depthcallback);

        //Publish new topic.
        //ros::Publisher pub = nh.advertise<opencv_object_tracking::position_publish>("position_object", 1);
        //Set the loop period with 0.1 second.
        ros::Rate loop_rate(10);

        //opencv_object_tracking::position_publish msg;
        //msg.counter = 0;

        
        setMouseCallback(OPENCV_WINDOW, mouse_callback, 0);

        //int count = 0;
        while ((ros::ok()))
        {
                double a_x, a_y, a_z;

                a_z = (X_111[0]-X_111[3]) * (X_111[0]-X_111[3]) + (Y_111[0]-Y_111[3]) * (Y_111[0]-Y_111[3]) + (Z_111[0]-Z_111[3]) * (Z_111[0]-Z_111[3]); 
                a_z = sqrt(a_z);
                a_y = (X_111[1]-X_111[3]) * (X_111[1]-X_111[3]) + (Y_111[1]-Y_111[3]) * (Y_111[1]-Y_111[3]) + (Z_111[1]-Z_111[3]) * (Z_111[1]-Z_111[3]); 
                a_y = sqrt(a_y); 
                a_x = (X_111[2]-X_111[3]) * (X_111[2]-X_111[3]) + (Y_111[2]-Y_111[3]) * (Y_111[2]-Y_111[3]) + (Z_111[2]-Z_111[3]) * (Z_111[2]-Z_111[3]); 
                a_x = sqrt(a_x); 
                
                double xx, xy, xz; 
                double yx, yy, yz; 
                double zx, zy, zz; 

                double dotxy, dotyz, dotzx;
                
                zx = (X_111[0]-X_111[3]) / a_z;  zy = (Y_111[0]-Y_111[3]) / a_z;  zz = (Z_111[0]-Z_111[3]) / a_z;  
                yx = (X_111[1]-X_111[3]) / a_y;  yy = (Y_111[1]-Y_111[3]) / a_y;  yz = (Z_111[1]-Z_111[3]) / a_y; 
                xx = (X_111[2]-X_111[3]) / a_x;  xy = (Y_111[2]-Y_111[3]) / a_x;  xz = (Z_111[2]-Z_111[3]) / a_x; 

                dotxy = dotp(xx, xy, xz, yx, yy, yz); 
                dotyz = dotp(yx, yy, yz, zx, zy, zz);
                dotzx = dotp(xx, xy, xz, zx, zy, zz);

                if(abs(dotxy) + abs(dotyz) + abs(dotzx) < 0.3){
                        tf3d.setValue(xx, yx ,zx,
                                      xy, yy ,zy,
                                      xz, yz ,zz
                                      ); 
                        tf3d.getRotation(q);
                        transform.setRotation(q);
                        
                        origin.setValue(X_111[3] ,Y_111[3], Z_111[3]); 
                        transform.setOrigin(origin); 
                        
                        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", "test"));        
                }
                // if ((flag == true) || true)
                // {


                //         // for(int i = 0; i < 4; i++){
                //         //         std::cout << color[i] << std::endl; 
                //         //         cout << posX[i] << ',' << posY[i] << std::endl;
                //         // }

                // }

                ros::spinOnce();
        }


    return 0;
}



