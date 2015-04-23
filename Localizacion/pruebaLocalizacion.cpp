#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>
#include "MCL.h"



int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("localization"));
	ros::NodeHandle n;

	
	ros::Publisher pose_pub;
	MCL mcl;
	geometry_msgs::PoseWithCovarianceStamped poserobot;

	pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pos", 1000);

	ros::Rate loop_rate(10);
	int count = 0;
	sleep(3);
	while (ros::ok())
	{
	
		//===============================================================================
		//Publish Pose
		mcl.correct();
		poserobot = mcl.getPose();
		//mcl.publishInfo();
		
		/*std::cout<<"Real pose: "<<poserobot.pose.pose.position.x<<", "<<
				poserobot.pose.pose.position.y<<", "<<
				poserobot.pose.pose.position.z<<")"<<std::endl;*/

		poserobot.header.frame_id = "world";
		poserobot.header.stamp = ros::Time::now();
		poserobot.header.seq = count;


		pose_pub.publish(poserobot);
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
