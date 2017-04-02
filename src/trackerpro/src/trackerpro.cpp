#include <iostream>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

void scan();
void track();

const int MINX = 0;
const int MAXX = 180;
const int MINY = 75;
const int MAXY = 125;

bool tracking = false;
bool dirX = true;
bool dirY = true;
int curX = 90;
int curY = 90;
int counter = 0;

Mat imgLines;
Mat globeimg;
VideoCapture cap(0); 


// (very accurate atleast for my light) 
const int iLowH = 163; // 0        0
const int iHighH = 177; // 9     179

const int iLowS = 206; // 111     191
const int iHighS = 255; // 209    255

const int iLowV = 148; // 89          4
const int iHighV = 190; // 255      150

const int moveThresh = 20;
const int movement = 1;

int main(int argc, char** argv)
{
	//ros setup
	ros::init(argc, argv, "eye_ball");
	ros::NodeHandle n;
	ros::Publisher eyeB_pub = n.advertise<std_msgs::Int16MultiArray>("servo", 1000);
	ros::Rate loop_rate(1000);
	

	// if not success, exit program
	if ( !cap.isOpened() )  
	{
      cout << "Cannot open the web cam" << endl;
      return -1;
	}
	
	// Capture a temporary image from the camera
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	
	Mat imgTmp;
	cap.read(imgTmp); 

	// Create a black image with the size as the camera output
	imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );

	//loop setup
	char c;
	char dim0_label[] = "servo";
	vector<int> msgdata;
	msgdata.push_back(90); msgdata.push_back(90);
	int count = 0;
	std_msgs::Int16MultiArray msg;
	
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].size = msgdata.size();
	msg.layout.dim[0].stride = 1;
	msg.layout.dim[0].label = "servo";
	
	while(ros::ok())
	{		
		msg.data.clear();
		
		if(!tracking)
			scan();
		
		track();
			
		msgdata[0] = curX; //x
		msgdata[1] = curY; //y
			
		// send (Needs to have a Transfrom setup to base, CurX as arg
		msg.data.insert(msg.data.end(), msgdata.begin(), msgdata.end());
		eyeB_pub.publish(msg);
		ros::spinOnce();
		this_thread::sleep_for(chrono::milliseconds(10));
		++count;
		//imshow("Thresholded Image", globeimg);
		
		if(waitKey(30) == 27) break;
	}  
	

	return 0;
}


void scan()
{
	int m, n; 
	if(dirX)
		m = 1;
	else
		m = -1;
	
	if(dirY)
		n = 1;
	else
		n = -1;
		
	curX += m*6;
	if(curX > MAXX)
	{
		dirX = false;
		curY += n*12;
	}	
	
	if(curX < MINX)
	{
		dirX = true;
		curY += n*12;
	}
	
	if( curY > MAXY )
		dirY = false;
	if( curY < MINY )
		dirY = true;	
		
		/*
	curX += m*15;
	curY += n*10;
	
	
	
	if(curX > MAXX)  
		dirX = false;
		
	if(curX < MINX)
		dirX = true;
	
	if(curY > MAXY)
		dirY = false;
		
	if(curY < MINY)
		dirY = true;	*/
}

void track()
{		
	int iLastX = -9999; 
	int iLastY = -9999;
		
      // read a new frame from video
      Mat imgOriginal;
      bool bSuccess = cap.read(imgOriginal); 
      //resize(imgOriginal, imgOriginal, Size(640, 360), 0, 0, INTER_CUBIC);

      // if not success, break loop
      if (!bSuccess) 
      {
		cout << "Cannot read a frame from video stream" << endl;
      }

      //Declare some Mat objects
      Mat imgHSV, imgThresholded;

      //Convert the captured frame from BGR to HSV
      cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

      //Threshold the image
      inRange( imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 
      
      //morphological opening (removes small objects from the foreground)
      erode( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

      //morphological closing (removes small holes from the foreground)
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
      erode( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      //Calculate the moments of the thresholded image
      Moments oMoments = moments(imgThresholded);
  	  //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
      //imshow( "Display window", imgThresholded );                   // Show our image inside it.
      //waitKey(0);   
      
      double dM01 = oMoments.m01;
      double dM10 = oMoments.m10;
      double dArea = oMoments.m00;
      
      // if the area <= 10000, I consider that the there are no object
      // in the image and it's because of the noise, the area is not zero 
      if (dArea > 10000)
      {
		//calculate the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;
		
		//#############################
		// x y positions are here
		//#############################
		iLastX = posX - 320;
		iLastY = -(posY - 180);

      }
      
		// print to console
		cout << "X : " << iLastX << " Y : " << iLastY << "\n";
		cout << "CurX: " << curX << " CurY: " << curY << "\n\n";
      
    if( iLastX == -9999 && iLastY == -9999 )
    {
		counter++;
		if(counter > 10)
		{
			tracking = false;
			counter = 0;
		}
	}
	
	else
	{
		tracking = true;
		if(iLastX < -moveThresh)
			curX -= movement;
		if(iLastX > moveThresh)
			curX += movement;
		if(iLastY < -moveThresh)
			curY -= movement;
		if(iLastY > moveThresh)
			curY += movement;
	}
}

