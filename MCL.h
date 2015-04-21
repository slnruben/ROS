/*
 * MCL.h
 *
 *  Created on: 07/12/2014
 *      Author: paco
 */

#ifndef MCL_H_
#define MCL_H_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "exps_sound/PoseWithProb.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include <boost/math/distributions.hpp>
#include <boost/math/distributions/normal.hpp>
#include "Common.h"
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <time.h>
#include <random>


#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include "ros/message_event.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_types_conversion.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef struct
{
	geometry_msgs::Pose coord;
	float p;
}Particle;

class MCL {
public:
	MCL();
	virtual ~MCL();

	void correct();
	void predict();
	void publishInfo();

	geometry_msgs::PoseWithCovarianceStamped getPose();

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
	int getStates() {return NUMPARTICLES;};



	void MapPCs(const sensor_msgs::PointCloud2::ConstPtr& mapmsg);
	void RobotPCs(const ros::MessageEvent<sensor_msgs::PointCloud2 const>& event);
private:

	static const float field_width;
	static const float field_height;

	static const int NUMPARTICLES = 100;
	static const int PERCEN_RANDOM_PARTICLES = 5;
	static constexpr float PCL_SEARCH_RADIUS = 0.05;
	static const int PCL_SEARCH_POINTS = 100;
	static constexpr float KO = 0.1;
	static constexpr float KH = 0.7;
	static constexpr float KS = 0.1;
	static constexpr float KV = 0.1;
	static constexpr float NOISE_LEVEL = 0.1; // 10%
	static const int RESET_COUNT = 20;
	static constexpr float RESET_TH = 0.3;

	void resetParticles();
	void updatePos();
	void reseed();
	void normalize();

	tf::Transform genNoise(tf::Transform &base);

	void printParticles();

	Particle particles[NUMPARTICLES];
	geometry_msgs::PoseWithCovariance pose;
	int seq;
	std::default_random_engine generator;
	std::normal_distribution<float> *distribution;

	void publishOrientations();
	void publishPose();

	ros::NodeHandle n;

	ros::Publisher part_pub;
	ros::Publisher pose_pub;
	ros::Publisher resetodom_pub;
	ros::Subscriber odom_sub;

	tf::TransformListener tfL;
	tf::TransformBroadcaster tfB;

	nav_msgs::Odometry odom;

	ros::Subscriber map_sub;
	ros::Subscriber camRobot_sub;
	int creset;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr *map;
	pcl::PointCloud<pcl::PointXYZRGB> robotcam;
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	bool kdtree_inited;
	void updateParticle(Particle &part);
	float doTestPcl(pcl::PointXYZRGB &searchPoint);

	void setParticle(Particle &particle, float x, float y, float theta);

};

#endif /* MCL_H_ */
