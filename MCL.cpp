/*
 * MCL.cpp
 *
 * 
 */

#include "MCL.h"

const float MCL::field_width = 6.0;
const float MCL::field_height = 5.0;

MCL::MCL() {

	srand(time(0));
	generator.seed(time(NULL));

	resetParticles();
	updatePos();

	part_pub = n.advertise<geometry_msgs::PoseArray>("/MCL_particles", 1000);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/MCL_pos", 1000);
	odom_sub = n.subscribe<nav_msgs::Odometry>("/robot1/odom", 10, &MCL::odomCB,this);
	seq = 0;

	resetOdom();

}

MCL::~MCL() {

}

void MCL::resetOdom() {
	odom.pose.pose.position.x = 0;
	odom.pose.pose.position.y = 0;
	odom.pose.pose.position.z = 0;

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, 0.0);
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();

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
	//Hallamos la diferencia con respecto al paso anterior

	nav_msgs::Odometry odom_now;

	odom_now.pose.pose.position.x = msg->pose.pose.position.x
			- last_odom.pose.pose.position.x;
	odom_now.pose.pose.position.y = msg->pose.pose.position.y
			- last_odom.pose.pose.position.y;
	odom_now.pose.pose.position.z = msg->pose.pose.position.z
			- last_odom.pose.pose.position.z;

	tf::Quaternion qnow(msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
	tf::Quaternion qlast(last_odom.pose.pose.orientation.x,
			last_odom.pose.pose.orientation.y,
			last_odom.pose.pose.orientation.z,
			last_odom.pose.pose.orientation.w);
	tf::Quaternion qodom(odom.pose.pose.orientation.x,
			odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
			odom.pose.pose.orientation.w);

	double lroll, lpitch, lyaw;
	double nroll, npitch, nyaw;
	double oroll, opitch, oyaw;

	tf::Matrix3x3(qnow).getRPY(nroll, npitch, nyaw);
	tf::Matrix3x3(qlast).getRPY(lroll, lpitch, lyaw);
	tf::Matrix3x3(qodom).getRPY(oroll, opitch, oyaw);

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, normalizePi(oyaw + nyaw - lyaw));

	//std::cerr<<"{"<<nyaw-lyaw<<"}"<<std::endl;

	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();

	float dlin = odom_now.pose.pose.position.x * cos(nyaw)
			+ odom_now.pose.pose.position.y * sin(nyaw);

	//std::cerr<<"["<<odom_now.pose.pose.position.x<<", "<<odom_now.pose.pose.position.y<<", "<<normalizePi(nyaw-lyaw)<<"] -> "<<dlin<<std::endl;

	odom.pose.pose.position.x = odom.pose.pose.position.x + dlin;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;

	last_odom = *msg;

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
//std::cerr<<"Posiciones = ("<<cx<<","<<cy<<") propos: ("<<particles[i].p<<")"<<std::endl;
		x = x + cx * particles[i].p;
		y = y + cy * particles[i].p;

//std::cerr<<"Centro angulo = ("<<xa<<","<<ya<<") propos: ("<<particles[i].p<<")"<<std::endl;

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
/*std::cout<<"Real pose: "<<paray.pose.position.x<<", "<<
				paray.pose.position.y<<", "<<
				paray.pose.position.z<<")"<<std::endl;*/
	pose_pub.publish(paray);
}

void MCL::publishInfo() {

	publishPose();
	publishOrientations();
}

void MCL::correct() {
	std::vector<std::string> frameList;
	tfL.getFrameStrings(frameList);

	std::vector<std::string>::iterator it;

	for (it = frameList.begin(); it != frameList.end(); ++it) {

		std::string frame = *it;

		if (isPrefix("perceived_", frame)) {

			std::string parent;

			parent = frame.substr(std::string("perceived_").length(),
					std::string::npos);
		
			updateObservation2(frame, parent);
		}

	}

	normalize();
	updatePos();
	reseed();
}

void MCL::updateObservation2(std::string obs, std::string real) {

	//Calcular la posición del robot en coordenadas de world, dada una observación

	tf::StampedTransform O2R, W2R;

	try {
		tfL.lookupTransform(obs, "base_link", ros::Time::now(), O2R);
		tfL.lookupTransform("world", "base_link", ros::Time::now(), W2R);

	} catch (tf::TransformException & ex) {
		ROS_WARN("%s", ex.what());
	}

/*std::cout<<"Real W2R with respect of the robot ("<<W2R.getOrigin().x()<<", "<<
					W2R.getOrigin().y()<<", "<<
					W2R.getOrigin().z()<<")"<<std::endl;*/
	//Comprobamos que está en el campo de visión (57º)

	tf::Matrix3x3 w2r(W2R.getRotation());
	double roll, pitch, yaw;
	w2r.getRPY(roll, pitch, yaw);

	float angle2obs = atan2(O2R.inverse().getOrigin().y(),
			O2R.inverse().getOrigin().x());
	float dista2obs = sqrt(
			O2R.inverse().getOrigin().y() * O2R.inverse().getOrigin().y()
					+ O2R.inverse().getOrigin().x()
							* O2R.inverse().getOrigin().x());

	if (fabs(angle2obs) > (57.0 * M_PI / 180.0))
		return;

	//std::cerr<<"O"<<std::endl;

	tf::StampedTransform L2W;
	try {
		tfL.lookupTransform("world", real,
				ros::Time::now() - ros::Duration(0.2), L2W);
	} catch (tf::TransformException & ex) {
		ROS_WARN("%s", ex.what());
	}

	static int c = 0;
	double mayor = 0.1;

	for (int i = 0; i < NUMPARTICLES; i++) {
		tf::Transform W2H, L2H;

		W2H.setOrigin(
				tf::Vector3(particles[i].coord.position.x,
						particles[i].coord.position.y,
						particles[i].coord.position.z));
		W2H.setRotation(
				tf::Quaternion(particles[i].coord.orientation.x,
						particles[i].coord.orientation.y,
						particles[i].coord.orientation.z,
						particles[i].coord.orientation.w));

		L2H = L2W.inverse() * W2H;

		tf::Matrix3x3 w2r(L2H.getRotation());
		double roll, pitch, yaw;
		w2r.getRPY(roll, pitch, yaw);

		float x, y;

		x = L2H.getOrigin().x() * cos(-yaw) - L2H.getOrigin().y() * sin(-yaw); //Porque es la vista desde la baliza al robot
		y = L2H.getOrigin().x() * sin(-yaw) + L2H.getOrigin().y() * cos(-yaw);
//std::cerr<<"x = ("<<x<<") y: ("<<y<<")"<<std::endl;
		float angle2ideal = normalizePi(atan2(y, x) + M_PI);
		float dista2ideal = sqrt(x * x + y * y);

		float desvDist = 0.2; //20 cms
		float desvAngl = 0.1; //0.1
//std::cerr<<"Posiciones = ("<<dista2obs<<", "<<dista2ideal<<")"<<" angulos: ("<<angle2obs<<", "<<angle2ideal<<")"<<std::endl;
		float probdist = getProbPos(dista2ideal, dista2obs, desvDist);
		float probrota = getProbRot(angle2ideal, angle2obs, desvAngl);
//std::cerr<<"Probdist = ("<<probdist<<") probrota: ("<<probrota<<")"<<std::endl;
		particles[i].p = particles[i].p + (probdist * probrota);

		if (particles[i].p < 0.0000001)
			particles[i].p = 0.0000001;

	}
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
//std::cerr<<"rseedB = ("<<rseedB<<") rseedA: ("<<rseedA<<")"<<std::endl;
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

float MCL::getProbPos(float ideal, float obs, float desv) {
	float dist;
	dist = fabs(ideal - obs);

	boost::math::normal_distribution<> myNormal(0.0, desv);

	return pdf(myNormal, dist);

}

float MCL::getProbRot(float ideal, float obs, float desv) {

	double diff;
	diff = normalizePi(ideal - obs);

	boost::math::normal_distribution<> myNormal(0.0, desv);

	return pdf(myNormal, diff);

}

void MCL::printParticles() {
	for (int i = 0; i < NUMPARTICLES; i++)
		std::cerr << "[" << i << "] (" << particles[i].coord.position.x << ", "
				<< particles[i].coord.position.y << ") " << particles[i].p
				<< std::endl;
}

