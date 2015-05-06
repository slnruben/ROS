#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "gazebo_msgs/ModelStates.h"
#include "Common.h"
#include <string>

//gazebo_msgs::ModelStates model;


/*void
modelCB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	model = *msg;
}

void
publishModel(const gazebo_msgs::ModelStates& msg)
{
	for(int i=0; i< msg.name.size(); i++)
	{
		if(isPrefix("robot", msg.name[i]))
		{
			tf::StampedTransform W2R;

			W2R.frame_id_ = "/world";
			W2R.setOrigin(tf::Vector3(msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z));
			W2R.setRotation(tf::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w));
			W2R.stamp_ = ros::Time::now();
			W2R.child_frame_id_ = msg.name[i]+"/base_footprint";

			try
			{
				tfB.sendTransform(W2R);

			}catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
			}


		}
	}
}*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, std::string("worldmodel"));
  ros::NodeHandle n;

  //ros::Subscriber submodel = n.subscribe("/gazebo/model_states", 1000, &modelCB);

tf::TransformBroadcaster tfB;

  ros::Rate loop_rate(1);
  int count = 0;

  while (ros::ok())
  {
	tf::StampedTransform W2Lm1, W2Lm2, W2Lm3, W2Lm4, W2NetB, W2NetY, base;

	W2Lm1.frame_id_ = "world";
	W2Lm1.child_frame_id_ = "0";
	W2Lm1.stamp_ = ros::Time::now() + ros::Duration(1.0);

	W2Lm1.setOrigin(tf::Vector3(1.5, 2.15, 0.15));
	W2Lm1.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	
	W2Lm2.frame_id_ = "world";
	W2Lm2.child_frame_id_ = "1";
	W2Lm2.stamp_ = ros::Time::now()+ ros::Duration(1.0);

	W2Lm2.setOrigin(tf::Vector3(-1.5, 2.15, 0.15));
	W2Lm2.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

	W2Lm3.frame_id_ = "world";
	W2Lm3.child_frame_id_ = "2";
	W2Lm3.stamp_ = ros::Time::now()+ ros::Duration(1.0);

	W2Lm3.setOrigin(tf::Vector3(-1.5, -2.15, 0.15));
	W2Lm3.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

	W2Lm4.frame_id_ = "world";
	W2Lm4.child_frame_id_ = "3";
	W2Lm4.stamp_ = ros::Time::now() + ros::Duration(1.0);

	W2Lm4.setOrigin(tf::Vector3(1.5, -2.15, 0.15));
	W2Lm4.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	
	W2NetB.frame_id_ = "world";
	W2NetB.child_frame_id_ = "5";
	W2NetB.stamp_ = ros::Time::now()+ ros::Duration(1.0);

	W2NetB.setOrigin(tf::Vector3(3.30, 0.0, 0.01));
	W2NetB.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

	W2NetY.frame_id_ = "world";
	W2NetY.child_frame_id_ = "4";
	W2NetY.stamp_ = ros::Time::now()+ ros::Duration(1.0);

	W2NetY.setOrigin(tf::Vector3(-3.30, 0.0, 0.01));
	W2NetY.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

	/*base.frame_id_ = "world";
	base.child_frame_id_ = "base_footprint";
	base.stamp_ = ros::Time::now()+ ros::Duration(1.0);

	base.setOrigin(tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
	base.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));*/

	try
	{
		tfB.sendTransform(W2Lm1);
		tfB.sendTransform(W2Lm2);
		tfB.sendTransform(W2Lm3);
		tfB.sendTransform(W2Lm4);
		tfB.sendTransform(W2NetB);
		tfB.sendTransform(W2NetY);
		//tfB.sendTransform(base);

	}catch(tf::TransformException & ex)
	{
		ROS_WARN("%s",ex.what());
	}


		//publishModel(model);
     	ros::spinOnce();

     	loop_rate.sleep();
     	++count;
  }
  return 0;
}
