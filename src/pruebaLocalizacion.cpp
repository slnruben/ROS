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

	tf::StampedTransform W2BL;
	tf::TransformBroadcaster tfB;
	ros::Publisher pose_pub;
	MCL mcl;
	geometry_msgs::PoseWithCovarianceStamped poserobot;

	pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pos", 1000);

	ros::Rate loop_rate(10);
	int count = 0;
	sleep(5);
	while (ros::ok())
	{
	
		//===============================================================================
		//Publish Pose
		//mcl.predict();
		poserobot = mcl.getPose();
		
		
		
		std::cout<<"Real pose: "<<poserobot.pose.pose.position.x<<", "<<
				poserobot.pose.pose.position.y<<", "<<
				poserobot.pose.pose.position.z<<")"<<std::endl;

		poserobot.header.frame_id = "/world";
		poserobot.header.stamp = ros::Time::now();
		poserobot.header.seq = count;

		W2BL.frame_id_ = "/world";
		W2BL.child_frame_id_ = "/base_link";
		W2BL.stamp_ = ros::Time::now() + ros::Duration(1.0);

		W2BL.setOrigin(tf::Vector3(poserobot.pose.pose.position.x, poserobot.pose.pose.position.y, 0.0));
		W2BL.setRotation(tf::Quaternion(poserobot.pose.pose.orientation.x, poserobot.pose.pose.orientation.y,
			poserobot.pose.pose.orientation.z, poserobot.pose.pose.orientation.w));
		
		try{
			tfB.sendTransform(W2BL);
		}catch(tf::TransformException & ex){
			ROS_WARN("%s",ex.what());
		}

		

		pose_pub.publish(poserobot);
		
		mcl.correct();
		mcl.publishInfo();

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
