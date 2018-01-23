#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
 
#include <stdio.h>

using namespace std;
using namespace cv;

#define REST 5.0
#define MINX 0
#define MINY 75
#define MAXX 180
#define MAXY 125

/** Function Headers */
int coordToPos (int p);
void stateTransition (bool notFound);
void search ();
bool detectAndDisplay (Mat frame);
void prepareCoordinates (int x, int y);


/** Global variables */
const String cascade_path = "../../../data/";
const String face_cascade_name  = cascade_path + "haarcascade_frontalface_alt.xml";
const String face_cascade_name2 = cascade_path + "haarcascade_frontalface_default.xml";
const String eyes_cascade_name  = cascade_path + "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
String window_name = "Face Detection";

int imgWidth;
int imgHeight;

struct STATE {

   STATE (int s, bool r, bool d) :
   speed(s), right(r), down(d) {}

   int lastX;
   int lastY;

   int speed;
   bool right;
   bool down;

   double rosTime;
   bool asleep;
   bool roaming;

} cur(1, true, true);

std_msgs::Int16MultiArray msg;

int main (int argc, char** argv) {

   //
   //    OpenCV Setup
   //

   //-- Load the cascades
   if (!face_cascade.load(face_cascade_name)) {
      cout << "--(!)Error loading : " << face_cascade_name.c_str() << "\n";
      return -1;
   }

   //-- Set Up OpenCV video stream
   VideoCapture capture;
   capture.open(0);
   if (!capture.isOpened()) { 
      printf("--(!)Error opening video capture\n"); 
      return -1; 
   }

   //-- Read the video stream
   Mat frame;
   capture.read(frame);
   
   //-- Find the middle of the Camera.
   //-- Used in normalizing later.
   imgWidth  = (frame.cols / 2);
   imgHeight = (frame.rows / 2);
   cur.asleep = false;



   //
   //    ROS Setup
   //   
   
   //-- Set Up ROS Node
   ros::init(argc, argv, "face_detect");
   ros::NodeHandle n;
   ros::Publisher track_pub = n.advertise<std_msgs::Int16MultiArray> ("track", 1000);
   ros::Rate loop_rate(10);

   //-- Set the first message to be "look at centre" (0, 0)
   msg.data.clear();                    // clear anything out
   msg.data.push_back(cur.lastX = 90);  // set x - yaw
   msg.data.push_back(cur.lastY = 90);  // set y - pitch
   track_pub.publish(msg);              // publish the message


 
   //
   //    Main Loop
   //
   //    Condition : Loop if frame capture was sucessful
   //                and if ROS network is okay
   // 
   while (capture.read(frame) && ros::ok()) {

      //-- Error Checking
      if (frame.empty()) {
	     printf(" --(!) No captured frame -- Break!");
	     break;
      }

      //-- Clear this out at the beginning
      msg.data.clear();

      //-- Apply the classifier to the frame
      stateTransition (detectAndDisplay (frame));

      //-- Publish the location of a face
      track_pub.publish(msg);
      ros::spinOnce();

      //-- Make the loop sleep
      //loop_rate.sleep();

      //-- If escape key is pressed, leave.      
      if (waitKey(10) == 27) { break; }
   }

   return 0;
}


inline int coordToPos (int p) {

   return 90 + p;
}


/** @function stateTransition */
void stateTransition(bool notFound) {

   //-- if notFound, and not previously assleep
   //-- set some states
   if (notFound && !cur.asleep) { 

      cur.asleep = true;
      cur.roaming = false;
      cur.rosTime = ros::Time::now().toSec();
      msg.data.push_back(cur.lastX); // push on the last known 
      msg.data.push_back(cur.lastY); // coordinates of something
      return;

   } else if (!notFound) {
      cur.asleep = false;
      cur.roaming = false;
      return;
   }


   //-- when 5 seconds have passed, enter "roaming" mode
   if (cur.asleep) {

      if (ros::Time::now().toSec() - cur.rosTime > REST) {

         cur.roaming = true;

      } else {
         msg.data.push_back(cur.lastX);
         msg.data.push_back(cur.lastY);
      }

      if (cur.roaming) {
         //-- look around
         search();
      }
   }
}


/** @function search */
void search() {

   if (cur.right) {
      cur.lastX -= cur.speed;
   } else {
      cur.lastX += cur.speed;
   } if (cur.down) {
      cur.lastY += cur.speed;
   } else {
      cur.lastY -= cur.speed;
   }

   if (cur.lastX <= MINX && cur.right) {
      cur.right = false;
   } else if (cur.lastX >= MAXX && !cur.right) {
      cur.right = true;
   } if (cur.lastY <= MINY && !cur.down) {
      cur.down = true;
   } else if (cur.lastY >= MAXY && cur.down) {
      cur.down = false;
   }

   msg.data.push_back(cur.lastX);
   msg.data.push_back(cur.lastY);
}


/** @function detectAndDisplay */
bool detectAndDisplay (Mat frame) {
   
   std::vector<Rect> faces;
   Mat frame_gray;

   cvtColor (frame, frame_gray, COLOR_BGR2GRAY);
   equalizeHist (frame_gray, frame_gray);

   //-- Detect faces
   face_cascade.detectMultiScale (frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(60, 60));

   for (size_t i = 0; i < faces.size(); i++) {

      //-- Draw Rectangles around things that look like a face
      rectangle (frame, faces[i].tl(), faces[i].br(), Scalar( 0, 180, 0 ), 5);
   }

   if (faces.size() >= 0) {

      //-- find the center of the first found face
      Point center (faces[0].x + faces[0].width/2, faces[0].y + faces[0].height/2);
      
      //-- prepare this coordinate for publishing
      prepareCoordinates (center.x, center.y);      
   }

   //-- Show what you got
   imshow (window_name, frame);

   //-- If no faces were found, lets know.
   return faces.empty();
}


/** @function prepareCoordinates */
void prepareCoordinates (int x, int y) {

   //-- Normalize
   x -= imgWidth;
   y -= imgHeight;

   //-- Add this data
   msg.data.push_back(cur.lastX = coordToPos(x));  // set x - yaw
   msg.data.push_back(cur.lastY = coordToPos(y));  // set y - pitch

   cout << "Face at { " << x << ", " << y << " }\n";
}