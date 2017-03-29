void odomMessage(){


  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;
  
  
  vx = (vel[0] + vel[3] + vel[1] + vel[2]) * (WHEEL_RADIUS/4);
  vy = ( -vel[0] + vel[3] + vel[1] - vel[2]) * (WHEEL_RADIUS/4);
  vth = ( -vel[0] + vel[3] - vel[1] + vel[2]) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH))); 
 
 unsigned long dt_micros = micros()-timeStamp;
 timeStamp+=dt_micros;
 
 
  //compute odometry in a typical way given the velocities of the robot
  double dt = dt_micros/1000000.0;
  double delta_x = (vx * cos(pose[2]) - vy * sin(pose[2])) * dt;
  double delta_y = (vx * sin(pose[2]) + vy * cos(pose[2])) * dt;
  double delta_th = vth * dt;
  
  pose[0] += delta_x;
  pose[1] += delta_y;
  pose[2] += delta_th;
  
  //compute this in quaternions
  geometry_msgs::Quaternion  q;
  q.x = 0;
  q.y = 0;
  q.z = sin(pose[2] * 0.5);
  q.w = cos(pose[2] * 0.5);
  
  ros::Time rosTime(timeStamp/1000000,timeStamp*1000);
  
    //next, we'll publish the odometry message over ROS
    odom->header.stamp = rosTime;
    odom->header.frame_id = "odom";
    
    //set the position
    odom->pose.pose.position.x = pose[0];
    odom->pose.pose.position.y = pose[1];
    odom->pose.pose.position.z = 0.0;
    odom->pose.pose.orientation = q;

    //set the velocity
    odom->child_frame_id = "base_link";
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy;
    odom->twist.twist.angular.z = vth;
  
       //publish the message
    odom_pub.publish(odom);
}
