/*
 * MCL.cpp
 *
 *  Created on: 07/12/2014
 *      Author: paco
 */

#include "MCL.h"

const float MCL::field_width = 5.4;
const float MCL::field_height = 7.4;

MCL::MCL() {

	srand(time(0));
	generator.seed(time(NULL));

	resetParticles();
	updatePos();

	part_pub = n.advertise<geometry_msgs::PoseArray>("/MCL_particles", 1000);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/MCL_pos", 1000);
	resetodom_pub = n.advertise<std_msgs::Empty>("/robot/commands/reset_odometry", 1);

	odom_sub = n.subscribe<nav_msgs::Odometry>("/robot/odom", 10, &MCL::odomCB, this);
	map_sub = n.subscribe("/map", 1, &MCL::MapPCs, this);
	camRobot_sub = n.subscribe("/camera/depth/points", 1, &MCL::RobotPCs, this);

	seq = 0;
	kdtree_inited = false;

	map = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);


	distribution = new std::normal_distribution<float>(0.0, NOISE_LEVEL);
	creset = 0;
}

MCL::~MCL() {

}

void MCL::RobotPCs(const ros::MessageEvent<sensor_msgs::PointCloud2 const>& event) {

	sensor_msgs::PointCloud2 in_basefoot;

	pcl_ros::transformPointCloud("/base_footprint", *(event.getMessage()),
			in_basefoot, tfL);

	pcl::PointCloud<pcl::PointXYZRGB> inimage;

	pcl::fromROSMsg(in_basefoot, inimage);

	robotcam.clear();

	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

	int c = 0;
	for (it = inimage.begin(); it != inimage.end(); ++it)
		if ((it->x == it->x) && (it->z > 0.1)) {
			c++;
			robotcam.push_back(*it);
		}

}

void MCL::MapPCs(const sensor_msgs::PointCloud2::ConstPtr& mapmsg)
{
	pcl::fromROSMsg(*mapmsg, **map);

	if(map->get()->size()>0)
	{
		kdtree.setInputCloud(*map);
		kdtree_inited = true;
	}


}


geometry_msgs::PoseWithCovarianceStamped MCL::getPose() {
	geometry_msgs::PoseWithCovarianceStamped ret;

	ret.header.seq = seq++;
	ret.header.frame_id = "world";
	ret.header.stamp = ros::Time::now();

	ret.pose = pose;

	return ret;
}

void MCL::odomCB(const nav_msgs::Odometry::ConstPtr& msg) {

	odom = *msg;

}

void MCL::resetParticles() {

	for (int i = 0; i < NUMPARTICLES; i++) {
		float x, y, t;
		particles[i].p = 1.0 / ((float) NUMPARTICLES);

		do {
			x = ((float) rand() / (float) RAND_MAX) * field_height
					- (field_height / 2.0);
			y = ((float) rand() / (float) RAND_MAX) * field_width
					- (field_width / 2.0);
		} while ((x < -(field_height / 2.0)) || (x > (field_height / 2.0))
				|| (y < -(field_width / 2.0)) || (y > (field_height / 2.0)));

		t = normalizePi(((float) rand() / (float) RAND_MAX) * 2.0 * M_PI);

		particles[i].coord.position.x = x;
		particles[i].coord.position.y = y;
		particles[i].coord.position.z = 0.0;

		tf::Quaternion q;
		q.setEuler(0.0, 0.0, t);

		particles[i].coord.orientation.x = q.x();
		particles[i].coord.orientation.y = q.y();
		particles[i].coord.orientation.z = q.z();
		particles[i].coord.orientation.w = q.w();

	}
}

void MCL::updatePos() {


	//std::cerr<<"U<";
	float x, sx2;
	float y, sy2;
	float xa, sxa2;
	float ya, sya2;
	float t, st2;
	double roll, pitch, yaw;

	x = 0.0;
	y = 0.0;
	t = 0.0;
	xa = ya = 0.0;


	for (int i = 0; i < NUMPARTICLES; i++) {
		float cx, cy;
		float cax, cay;

		cx = particles[i].coord.position.x;
		cy = particles[i].coord.position.y;


		tf::Quaternion q(particles[i].coord.orientation.x,
				particles[i].coord.orientation.y,
				particles[i].coord.orientation.z,
				particles[i].coord.orientation.w);

		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		cax = cos(yaw);
		cay = sin(yaw);

		//sdt::cerr<<"\t\t("<<cax<<","<<cay<<"): "<< particles[i].p<<std::endl;

		x = x + cx * particles[i].p;
		y = y + cy * particles[i].p;

		xa = xa + cax * particles[i].p;
		ya = ya + cay * particles[i].p;


		//sdt::cerr<<"\t\t---->("<<xa<<","<<ya<<")"<<std::endl;
	}
	//sdt::cerr<<"\t\t---->("<<xa<<","<<ya<<")"<<std::endl;


	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = 0.0;


	t = atan2(ya, xa);

	sx2 = 0.0;
	sy2 = 0.0;
	sxa2 = 0.0;
	sya2 = 0.0;
	st2 = 0.0;

	for (int i = 0; i < NUMPARTICLES; i++) {
		float cx, cy;
		float cax, cay;

		cx = particles[i].coord.position.x;
		cy = particles[i].coord.position.y;

		tf::Quaternion q(particles[i].coord.orientation.x,
				particles[i].coord.orientation.y,
				particles[i].coord.orientation.z,
				particles[i].coord.orientation.w);

		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		cax = cos(yaw);
		cay = sin(yaw);

		sx2 = sx2 + particles[i].p * ((cx - x) * (cx - x));
		sy2 = sy2 + particles[i].p * ((cy - y) * (cy - y));

		sxa2 = sxa2 + particles[i].p * ((cax - xa) * (cax - xa));
		sya2 = sya2 + particles[i].p * ((cay - ya) * (cay - ya));

		float taux= normalizePi(yaw - t);

		st2 = st2 + particles[i].p * taux * taux;

	}


	tf::Quaternion q;

	q.setEuler(0.0, 0.0, t);

	pose.pose.orientation.x = q.x();
	pose.pose.orientation.y = q.y();
	pose.pose.orientation.z = q.z();
	pose.pose.orientation.w = q.w();


	//st2 = atan2(sya2, sxa2);
	//printParticles();

	/*
	 sx2 = sx2 / (float) NUMPARTICLES;
	 sy2 = sy2 / (float) NUMPARTICLES;
	 st2 = st2 / (float) NUMPARTICLES;
	 */
	//

	//sdt::cerr<<"Pos = ("<<pose.pose.position.x<<", "<<pose.pose.position.y<<", "<<t<<") ["<<sx2<<", "<<sy2<<", "<<st2<<"]"<<std::endl;
	for (int i = 0; i < 36; i++)
		pose.covariance[i] = 0.0;

	pose.covariance[0] = sx2; //X*X
	pose.covariance[1 * 6 + 1] = sy2; //Y*Y
	pose.covariance[5 * 6 + 5] = st2; //rZ*rZ
	//std::cerr<<">"<<std::endl;

}

void MCL::normalize() {

	//std::cerr<<"N<";

	float sum = 0.0;
	float factor;

	for (int i = 0; i < NUMPARTICLES; i++)
		sum = sum + particles[i].p;

	factor = 1.0 / sum;

	for (int i = 0; i < NUMPARTICLES; i++)
		particles[i].p = particles[i].p * factor;

	//std::cerr<<">"<<std::endl;;
}

void MCL::publishOrientations() {
	geometry_msgs::PoseArray paray;
	static int seq = 0;

	paray.header.frame_id = "world";
	paray.header.stamp = ros::Time::now();
	paray.header.seq = seq++;

	for (int i = 0; i < NUMPARTICLES; i++)
		paray.poses.push_back(particles[i].coord);

	part_pub.publish(paray);
}

void MCL::publishPose() {
	geometry_msgs::PoseStamped paray;
	static int seq = 0;

	paray.header.frame_id = "world";
	paray.header.stamp = ros::Time::now();
	paray.header.seq = seq++;

	paray.pose = pose.pose;

	pose_pub.publish(paray);
}

void MCL::publishInfo() {

	publishPose();
	publishOrientations();
}


tf::Transform
MCL::genNoise(tf::Transform &base)
{

	tf::Transform ret;

	ret.setOrigin(tf::Vector3(base.getOrigin().getX()*(*distribution)(generator),
			base.getOrigin().getY()*(*distribution)(generator),
			base.getOrigin().getZ()*(*distribution)(generator)));


	tf::Quaternion q(base.getRotation().getX(),
			base.getRotation().getY(),
			base.getRotation().getZ(),
			base.getRotation().getW());

	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	yaw = yaw * (*distribution)(generator);

	q.setEuler(0.0, 0.0, yaw);

	ret.setRotation(q);

	return ret;

}

void MCL::predict() {

	tf::Transform desp;

	desp.setOrigin(
			tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y,
					odom.pose.pose.position.z));
	desp.setRotation(
			tf::Quaternion(odom.pose.pose.orientation.x,
					odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
					odom.pose.pose.orientation.w));

	if (desp.getRotation().getX() != desp.getRotation().getX())
		return;

	for (int i = 0; i < NUMPARTICLES; i++) {
		tf::Transform part, parttf;

		part.setOrigin(
				tf::Vector3(particles[i].coord.position.x,
						particles[i].coord.position.y,
						particles[i].coord.position.z));
		part.setRotation(
				tf::Quaternion(particles[i].coord.orientation.x,
						particles[i].coord.orientation.y,
						particles[i].coord.orientation.z,
						particles[i].coord.orientation.w));

		tf::Transform noise = genNoise(desp);

		parttf = part * desp * noise;

		particles[i].coord.position.x = parttf.getOrigin().getX();
		particles[i].coord.position.y = parttf.getOrigin().getY();
		particles[i].coord.position.z = parttf.getOrigin().getZ();
		particles[i].coord.orientation.x = parttf.getRotation().getX();
		particles[i].coord.orientation.y = parttf.getRotation().getY();
		particles[i].coord.orientation.z = parttf.getRotation().getZ();
		particles[i].coord.orientation.w = parttf.getRotation().getW();

	}

	std_msgs::Empty rmsg;
	resetodom_pub.publish(rmsg);
}

void MCL::setParticle(Particle &particle, float x, float y, float theta)
{

	particle.coord.position.x = x;
	particle.coord.position.y = y;
	particle.coord.position.z = 0.0;

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, theta);

	particle.coord.orientation.x = q.x();
	particle.coord.orientation.y = q.y();
	particle.coord.orientation.z = q.z();
	particle.coord.orientation.w = q.w();

}

void MCL::correct() {

	if (robotcam.size() == 0)
		return;

	if(!kdtree_inited)
	{
		ROS_ERROR("No input clouds. Is map_node running?");
		return;
	}

	//sdt::cerr<<"================================================================="<<std::endl;

	//setParticle(particles[0], 0.0, 0.0, M_PI);
	//setParticle(particles[0], 0.0, 0.0, 0.0);
	/*setParticle(particles[0], 0.0, 0.0, M_PI);
	setParticle(particles[1], 0.0, 0.0, M_PI);
	setParticle(particles[2], 0.0, 0.0, M_PI);
	setParticle(particles[3], 0.0, 0.0, M_PI);
	setParticle(particles[4], 0.0, 0.0, M_PI);
*/

	for (int i = 0; i < NUMPARTICLES; i++)
		updateParticle(particles[i]);

	float sump;
	for (int i = 0; i < NUMPARTICLES; i++)
		sump = sump + particles[i].p;

	sump = sump/(float)NUMPARTICLES;


	//std::cerr<<"["<<sump<<", "<<RESET_TH<<"]"<<std::endl;

	if(sump<RESET_TH)
	{
		//std::cerr<<"+";
		creset++;
	}
	else
	{
		//std::cerr<<"-";
		creset = 0;
	}

	//std::cerr<<"{"<<creset<<"}";

	if(creset>RESET_COUNT)
	{
		//std::cerr<<"RESET"<<std::endl;
		creset = 0;
		resetParticles();
	}



	normalize();
	updatePos();
	reseed();

	publishInfo();

}

void MCL::updateParticle(Particle &part) {

	tf::Transform W2H;

	W2H.setOrigin(
			tf::Vector3(part.coord.position.x, part.coord.position.y,
					part.coord.position.z));

	W2H.setRotation(
			tf::Quaternion(part.coord.orientation.x, part.coord.orientation.y,
					part.coord.orientation.z, part.coord.orientation.w));


	tf::Quaternion q(part.coord.orientation.x, part.coord.orientation.y,
			part.coord.orientation.z, part.coord.orientation.w);

	double roll, pitch, yaw;

	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	//sdt::cerr<<"Particle in ("<<part.coord.position.x<<", "<<part.coord.position.y<<", "<<yaw<<"): ";


	for (int i = 0; i < PCL_SEARCH_POINTS; i++) {
		int idx = rand() % robotcam.size();

		pcl::PointXYZRGB testpoint = robotcam[idx];

		tf::Vector3 posc(testpoint.x, testpoint.y, testpoint.z);
		tf::Vector3 posw;

		posw = W2H * posc;

		//sdt::cerr<<"("<<testpoint.x<<", "<<testpoint.y<<") -> ";

		testpoint.x = posw.getX();
		testpoint.y = posw.getY();
		testpoint.z = posw.getZ();


		//sdt::cerr<<"("<<testpoint.x<<", "<<testpoint.y<<")"<<std::endl;


		part.p = doTestPcl(testpoint);

		if (part.p < 0.00001)
			part.p = 0.00001;
	}

}

float MCL::doTestPcl(pcl::PointXYZRGB &searchPoint) {

	float ret = 0.0;


	int K = 1;
	int cpart = 0;

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);


	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		pcl::PointXYZHSV hsvA, hsvB;
		float diffH, diffS, diffV;

		diffH = 0.0;
		diffS = 0.0;
		diffV = 0.0;

		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
		{
			if(sqrt(pointNKNSquaredDistance[i]) < PCL_SEARCH_RADIUS)
			{

				PointXYZRGBtoXYZHSV(searchPoint, hsvA);

				PointXYZRGBtoXYZHSV((*map)->points[pointIdxNKNSearch[i]], hsvB);

				//sdt::cerr<<"\t("<<hsvA.h<<", "<<hsvA.s<<", "<<hsvA.v<<") ";
				//sdt::cerr<<"("<<hsvB.h<<", "<<hsvB.s<<", "<<hsvB.v<<") ";

				diffH += normalizePi(toRadians(hsvA.h - 180.0) - toRadians(hsvB.h - 180.0));
				diffS += fabs(hsvA.s - hsvB.s);
				diffV += fabs(hsvA.v - hsvB.v);

				cpart++;
			}
		}

		if(cpart>0)
		{
			ret = ret + KO;
			//sdt::cerr<<"\tTotal: ("<<diffH<<", "<<diffS<<", "<<diffV<<")  -> ";

			diffH = diffH / cpart;
			diffS = diffS / cpart;
			diffV = diffV / cpart;

			ret = ret + KH * (fabs(normalizePi(M_PI - diffH)/M_PI)) + KS * (1.0 - diffS) + KV * (1.0 - diffV);
		}else
			ret = 0.0;


		//sdt::cerr<<"["<<KH * (fabs(normalizePi(M_PI - diffH)/M_PI))<<", "<<KS * (1.0 - diffS) <<", "<< KV * (1.0 - diffV)<<"] = "<<ret<<std::endl;
	  }
	/*
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = PCL_SEARCH_RADIUS;

	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
			pointRadiusSquaredDistance) > 0) {

		ret = ret + KO;

		pcl::PointXYZHSV hsvA, hsvB;

		PointXYZRGBtoXYZHSV(searchPoint, hsvA);

		float diffH, diffS, diffV;

		diffH = 0.0;
		diffS = 0.0;
		diffV = 0.0;

		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
			PointXYZRGBtoXYZHSV((*map)->points[pointIdxRadiusSearch[i]], hsvB);

			diffH += normalizePi(
					toRadians(hsvA.h - 180.0) - toRadians(hsvB.h - 180.0));
			diffS += fabs(hsvA.s - hsvB.s);
			diffV += fabs(hsvA.v - hsvB.v);
		}

		diffH = diffH / (float) pointIdxRadiusSearch.size();
		diffS = diffH / (float) pointIdxRadiusSearch.size();
		diffV = diffH / (float) pointIdxRadiusSearch.size();

		ret = ret + KH * (1.0 - diffH) + KS * (1.0 - diffS)
				+ KV * (1.0 - diffV);
	}
	*/
	return ret;

}

void MCL::reseed() {

	float sx2, sy2, st2;

	sx2 = pose.covariance[0]; //X*X
	sy2 = pose.covariance[1 * 6 + 1]; //Y*Y
	st2 = pose.covariance[5 * 6 + 5]; //rZ*rZ

	std::normal_distribution<float> normalX(0.0, sqrt(sx2));
	std::normal_distribution<float> normalY(0.0, sqrt(sy2));
	std::normal_distribution<float> normalT(0.0, sqrt(st2));

	double roll, pitch, yaw;
	tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
			pose.pose.orientation.z, pose.pose.orientation.w);

	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	int rseedA, rseedB;

	rseedB = (NUMPARTICLES * PERCEN_RANDOM_PARTICLES) / 100;
	rseedA = NUMPARTICLES - rseedB;

	for (int i = 0; i < rseedA; i++) {
		float x, y;
		do {
			x = pose.pose.position.x + normalX(generator);
			y = pose.pose.position.y + normalY(generator);

		} while ((x < -(field_height / 2.0)) || (x > (field_height / 2.0))
				|| (y < -(field_width / 2.0)) || (y > (field_height / 2.0)));

		particles[i].coord.position.x = x;
		particles[i].coord.position.y = y;
		particles[i].coord.position.z = 0;

		float newt = normalizePi(yaw + normalizePi((double) normalT(generator)));
		//sdt::cerr<<"\tyaw: "<<yaw<<", "<<newt<<std::endl;
		tf::Quaternion q;
		q.setEuler(0.0, 0.0, newt);

		particles[i].coord.orientation.x = q.x();
		particles[i].coord.orientation.y = q.y();
		particles[i].coord.orientation.z = q.z();
		particles[i].coord.orientation.w = q.w();


		//sdt::cerr<<"R to {"<<particles[i].coord.position.x<<", "<<particles[i].coord.position.y<<","<<newt<<"}"<<std::endl;
	}

	for (int i = rseedA; i < (rseedA + rseedB); i++) {

		float x, y;
		do {
			x = ((float) rand() / (float) RAND_MAX) * field_height
					- (field_height / 2.0);
			y = ((float) rand() / (float) RAND_MAX) * field_width
					- (field_width / 2.0);

		} while ((x < -(field_height / 2.0)) || (x > (field_height / 2.0))
				|| (y < -(field_width / 2.0)) || (y > (field_height / 2.0)));

		particles[i].coord.position.x = x;
		particles[i].coord.position.y = y;
		particles[i].coord.position.z = 0;

		float t = normalizePi(((float) rand() / (float) RAND_MAX) * 2.0 * M_PI);

		tf::Quaternion q;
		q.setEuler(0.0, 0.0, t);

		particles[i].coord.orientation.x = q.x();
		particles[i].coord.orientation.y = q.y();
		particles[i].coord.orientation.z = q.z();
		particles[i].coord.orientation.w = q.w();
	}

}

void MCL::printParticles() {
	for (int i = 0; i < NUMPARTICLES; i++)
		std::cerr << "[" << i << "] (" << particles[i].coord.position.x << ", "
				<< particles[i].coord.position.y << ") " << particles[i].p
				<< std::endl;
}
