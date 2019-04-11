#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <stdlib.h>

#define BUCKET_SIZE 10

void range_message_callback(const sensor_msgs::Range msg);

sensor_msgs::Range range_msg; //Create the message from sensor_msgs Range class
float bucket[BUCKET_SIZE] = {}; //Create a bucket for the average
int bucket_position = 0;      //Variable for adding the new member

ros::Publisher pub_range;
ros::Subscriber sub;

float moving_average(float new_element)
{
	float average = 0;			//Init local variable
	//If the position for the new element is passing the array size go to the start	
	if(bucket_position >= BUCKET_SIZE)	
	{
		bucket_position = 0;
	}
	
	bucket[bucket_position] = new_element;  //Add new element to the array
	bucket_position++;			//Increase the pointer
	//Sum up the array
	for(int i = 0; i<BUCKET_SIZE; i++)
	{
	    average+=bucket[i];
	}
	return average/BUCKET_SIZE;		//Return the average
}

void range_message_callback(const sensor_msgs::Range msg)
{
  range_msg = msg; 			       //Copy the structure
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "range_lstn_pub");
	ros::NodeHandle nh;

	pub_range = nh.advertise<sensor_msgs::Range>("ultrasound/filtered", 1000); //Create a publisher
	sub = nh.subscribe("ultrasound/raw", 1000, range_message_callback);       //Create a subscriber to handle messages

	ros::Rate loop_rate(5); //Running rate of the loop in Hz

	while(ros::ok())
	{	
		
  		range_msg.range = moving_average(range_msg.range);   //Replace the range with the average
  		pub_range.publish(range_msg);		       //Publish the new message
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}

