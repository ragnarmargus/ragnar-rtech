#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <stdlib.h>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "range_pub");
	ros::NodeHandle nh;
	
	sensor_msgs::Range range_msg; //Create the message from sensor_msgs Range class
	ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>("range", 1000); //Create a publisher
	
	
	ros::Rate loop_rate(5); //Running rate of the loop in Hz

	//Fill the static data of the message
	range_msg.min_range 	  = 0.02;
	range_msg.max_range 	  = 4;
	range_msg.field_of_view   = 3.14/6;
	range_msg.header.frame_id = "lens";

	while(ros::ok())
	{	
		//Fill the rest of the message
		range_msg.range = ((float)rand()/RAND_MAX)*4;
		pub_range.publish(range_msg);
	
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}
