/*
 * Behav.h
 *
 *  Created on: 24/4/2015
 *      Author: Rich
 *
me llegan las pelotas en tf local, elijo la q este mas cerca y comienzo el proceso de busqueda y captura. si recibo otras pelotas dependiendo de su posicion en local voy  apor ella(busquda y captura) o la guardo en tf W para poder ir a buscarla cuando termine con la anterior. ami men mandaran las pelotas diferenciadas y debo pasar de las que ya he usado. me mandan las pelotas de cada color mas cercanas todo el tiempo; voy a por la mas cercana q no este r; despues de capturar me dirijo a la pos mas cercana de otra pelota q hubiese visto q no este r pero priorizo lo q veo antes q las pos q tnga guardadas.

de local: recibo mi pos en global para poder ir a capturar y/o centro
de percepcion: recibo pos de pelotas
 */

#ifndef BEHAV_H_
#define BEHAV_H_

#include <string>
#include "ros/ros.h"
//-----------default-pckg------------
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

//------------------------------
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <cstddef>
#include "Localization/Markov/Markov.h"
#include "Localization/QuadMK/QuadMK.h"
#include "Localization/MCL/MCL.h"

class Behav {
private:

	void correct();
	std::string id;

	ros::NodeHandle n;
	ros::Publisher posePub;

	tf::TransformBroadcaster tfB;

	geometry_msgs::PoseWithCovarianceStamped pose;
	geometry_msgs::PoseStamped goalpose;
	  kobuki_msgs::Sound sound_cmd;
	bool goalrecv = false;

	Markov markov;
	QuadMK quadmk;
	MCL mcl;

ros::Subscriber robotposesub ;
	//ros::Subscriber submodelsub = n.subscribe("/gazebo/model_states", 1000, &fakeposeCB::poseCB, this);
	ros::Subscriber goalsub ;
	ros::Publisher  cmdpub_t ;
	ros::Publisher  cmdpub_s ; 


//----------------------------------------------------------
	inline double normalizePi(double data);

//----------------------------------------------------------
	struct Object{
		std::string name;
		float cx;
		float cy;
		float cz;
	};

	struct ball{
		Object o;
		bool found;
	};
	int state;
	static const int num_objects = 3;	
	static const int begin=0;
	static const int lost=1;
	static const int search=2;
	static const int end=3;

	ball array [num_objects];
	Object goal;
	Object center;
	Object pose1;
	
	//tfs

	std::vector<std::string> balls;
	std::vector<std::string> frameList;
	std::vector<std::string>::iterator it;
public:
	//Behav(std::string _id);
	Behav();
	virtual ~Behav();

	void step();

	void groundtruthCB(const gazebo_msgs::ModelStates::ConstPtr& msg);
	void poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void go(Object o);
	void peep();
	bool hayTarget();
	bool terminado();
};


#endif
