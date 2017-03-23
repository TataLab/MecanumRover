//a simple interface for a miniq rover over ROS


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv){

	//initialize ros and set up a node
	ros::init(argc, argv, "mecanum_commands");  
	ros::NodeHandle nh;
	
	//create a publisher and tell it to publish messages on the miniq/cmd_vel topic with a queue size of 100
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("mecanum/cmd_vel",100);


	//set the loop rate of the publisher to 10hz
	ros::Rate rate(10);
	
	while(ros::ok()){
		
		//declare the message
		geometry_msgs::Twist msg;
	
		//these are the values to send in the twist message
		msg.linear.x=1.0;
		msg.linear.y = 0.0;
		msg.angular.z=0.0;
		double speed=1;
	
		//publish the first twist message
		pub.publish(msg);
	
		while(1)
		{
			for(int i=0;i<100;i++)
			{
				msg.linear.x=speed*cos((3.14159/100)*i);
				msg.linear.y=speed*sin((3.14159/100)*i);
				pub.publish(msg);
				rate.sleep();
			}
		}
	}
}