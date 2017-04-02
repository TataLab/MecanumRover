#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

float proportion;
float integral;
float derivative;


int main(int argc, char **argv){

	//initialize ros and set up a node
	ros::init(argc, argv, "mecanum_commands");  
	ros::NodeHandle nh;
	
	//create a publisher and tell it to publish messages on the miniq/cmd_vel topic with a queue size of 100
	ros::Publisher pid=nh.advertise<geometry_msgs::Twist>("mecanum/cmd_vel",100);

	//set the loop rate of the publisher to 10hz
	ros::Rate rate(10);
	
	while(ros::ok()){
		
		//declare the message
		geometry_msgs::Twist msg;
	
		//publish the first twist message
		pub.publish(msg);
	
		while(1)
		{
			cout << "Enter the proportional float: ";
			cin >> proportion;
			cout << "Enter the integral float: ";
			cin >> integral;
			cout << "Enter the derivative float: ";
			cin >> derivative;
			msg.linear.x=proportion;
			msg.linear.y=integral;
			msg.angular.z=derivative;
			
			pub.publish(msg);
			rate.sleep();
		}
	}
}