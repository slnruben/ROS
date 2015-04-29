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
#include "robotica/PoseWithProb.h"
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
	void publishInfo();
	geometry_msgs::PoseWithCovarianceStamped getPose();

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
	int getStates() {return NUMPARTICLES;};
private:

	static const float field_width;
	static const float field_height;
	static const int NUMPARTICLES = 100;
	static const int PERCEN_RANDOM_PARTICLES = 5;

	void resetParticles();
	void updatePos();
	void reseed();
	void normalize();

	void resetOdom();

	void printParticles();

	Particle particles[NUMPARTICLES];
	geometry_msgs::PoseWithCovariance pose;
	int seq;
	std::default_random_engine generator;


	void updateObservation2(std::string obs, std::string real);
	void publishOrientations();
	void publishPose();

	float getProbPos(float ideal, float obs, float desv);
	float getProbRot(float ideal, float obs, float desv);

	ros::NodeHandle n;

	ros::Publisher part_pub;
	ros::Publisher pose_pub;
	ros::Subscriber odom_sub;
	tf::TransformListener tfL;
	tf::TransformBroadcaster tfB;

	nav_msgs::Odometry last_odom, odom;
};

#endif /* MCL_H_ */
