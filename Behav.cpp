/*
 * Behav.cpp
 *
 *  Created on: 24/4/2015			vars: array de (pelotas,saved), centro , goal, pose
 *      Author: Rich				

 *		B-Net

	lm1	^x	lm2
		|				mod en cmlist y en vicam para el testA.cpp
	 Y<-----|
	lm3		lm4
		Y-Net
		
 */

//#include "Behav.h"

#include <vector>

//---------------------------------------------quitar si .h --------------------------------

#include <string>
#include "ros/ros.h"
//-----------default-pckg------------
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <kobuki_msgs/Sound.h>
//------------------------------
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <cstddef>


//---------------------------------------------quitar si .h --------------------------------
// Behav::Behav() {

// 133 1 vuelta 37 1 m
		//
// 	//generator.seed(time(NULL));
// 	//distribution = new std::normal_distribution<float>(0.0, 0.1);



// 	//sub_suelo = n.subscribe("/image_converter/output_video_suelo", 1, &SaveVictims::localizaSuelo, this);


// //robotposesub = n.subscribe("/robot_pos", 1000, &Behav::poseCB, this);
// 	//ros::Subscriber submodelsub = n.subscribe("/gazebo/model_states", 1000, &fakeposeCB::poseCB, this);
// //	goalsub = n.subscribe("/move_base_simple/goal", 1000, &Behav::goalCB, this);
// 	  cmdpub_t = n.advertise<geometry_msgs::Twist>("/robot/commands/velocity", 1000);
// 	 cmdpub_s = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1); 


// 	for (int i = 0; i < num_objects; ++i){
// 	    array[i].o.name = "empty";
// 	    array[i].o.cx = 0.0;	//cuidado
// 	    array[i].o.cy = 0.0;
// 	    array[i].o.cz = 0.0;
// 	    array[i].found = true;
// 	}	

// 	 goal.name="Goal";
// 	 goal.cx=0.0;
// 	 goal.cy=0.0;
// 	 goal.cz=0.0;
// 	 center.name="Center";
// 	 center.cx=0.0;
// 	 center.cy=0.0;
// 	 center.cz=0.0;

//  	 state = 0;   
 

// }

	int state;
	static const int num_objects = 3;	
	static const int begin=0;
	static const int search=1;
	static const int rescue=2;
	static const int girydist=3;
	static const int end=4;
	static const int MTODETECT=0.68;
	std::string emp = "empty";

	struct Object{
		std::string name;
		float cx;
		float cy;
		float cz;
	};

	struct ball{
		Object o;
		bool found;
		ros::Time time;
	};


	ball array [num_objects];
	Object goal;
	Object center;
	Object pose1;



	std::string id;


	//ros::Publisher posePub;


	geometry_msgs::PoseWithCovarianceStamped pose;
	geometry_msgs::PoseWithCovarianceStamped fakepose;
	geometry_msgs::PoseStamped goalpose;
	kobuki_msgs::Sound sound_cmd;
	bool goalrecv = false;


	ros::Subscriber robotposesub ;
	ros::Subscriber submodelsub;
	ros::Subscriber goalsub ;
	ros::Publisher  cmdpub_t ;
	ros::Publisher  cmdpub_s ; 



	std::vector<std::string> balls;
	std::vector<std::string> frameList;
	std::vector<std::string>::iterator it;





inline double
normalizePi(double data)
{
  if (data < M_PI && data >= -M_PI) return data;
  double ndata = data - ((int )(data / (M_PI*2.0)))*(M_PI*2.0);
  while (ndata >= M_PI)
  {
    ndata -= (M_PI*2.0);
  }
  while (ndata < -M_PI)
  {
    ndata += (M_PI*2.0);
  }
  return ndata;
}

void poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose = *msg;
  	 pose1.name="Center";
	 pose1.cx= pose.pose.pose.position.x;
	 pose1.cy= pose.pose.pose.position.y;
	 pose1.cz=0.0;
}

void fakeposeCB(const gazebo_msgs::ModelStates &states)
{
	for(int i=0; i< states.name.size(); i++)
	{
		//std::cout<<states.name[i]<<std::endl;
		if(states.name[i] == "mobile_base")
		{
	//std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;


			fakepose.pose.pose.position.x = states.pose[i].position.x;
			fakepose.pose.pose.position.y = states.pose[i].position.y;
			fakepose.pose.pose.position.z = states.pose[i].position.z;


			tf::Quaternion q2(states.pose[i].orientation.x, states.pose[i].orientation.y, states.pose[i].orientation.z, states.pose[i].orientation.w);
		
			fakepose.pose.pose.orientation.x = q2.x();
			fakepose.pose.pose.orientation.y = q2.y();
			fakepose.pose.pose.orientation.z = q2.z();
			fakepose.pose.pose.orientation.w = q2.w();

			for (int i = 0; i < 36; i++)
				fakepose.pose.covariance[i] = 0.0;
		
			//Fake uncertainty
			float sx2 = 0.2;
			float sy2 = 0.2;
			float st2 = 0.1; //rads

			fakepose.pose.covariance[0] = sx2; //X*X
			fakepose.pose.covariance[1 * 6 + 1] = sy2; //Y*Y
			fakepose.pose.covariance[5 * 6 + 5] = st2; //rZ*rZ
			//***************//
			pose1.name="Center";
			pose1.cx= fakepose.pose.pose.position.x;
			pose1.cy= fakepose.pose.pose.position.y;
			pose1.cz=0.0;
			//***************//
		}
	}
}

int
gira(Object o)	
{
	float diffpose;
	int niter=0;

 std::cout<<"x:"<<pose1.cx<<std::endl;
 std::cout<<"y:"<<pose1.cy<<std::endl;	
	

	diffpose = sqrt( (pose.pose.pose.position.x-o.cx)*(pose.pose.pose.position.x-o.cx)+ (pose.pose.pose.position.y-o.cy)*(pose.pose.pose.position.y-o.cy)); 
	

	double roll, pitch, yaw;
	tf::Quaternion q(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, 1.0);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);


				
	float v,w;
	float angle2goal;
	if(diffpose < 0.1){	
		w = v = 0.0;
		niter=0;
	}else{
		//va de 3.1/-3.1 a 0
		// 133 1 vuelta 37 1 m

		angle2goal = normalizePi(atan2(o.cy - pose.pose.pose.position.y, o.cx -pose.pose.pose.position.x) - yaw);

		//70 seria lo q tarda en dar media vuelta +o-
		//niter = niter +  70*angle2goal/3.1;
		niter = 7;

		if(fabs(angle2goal) > 0.08)
		{
			w = (angle2goal/fabs(angle2goal)) * 0.5;
			v = 0.0;
		}else{
			w = v = 0.0;
			niter=0;
		}
	}

	geometry_msgs::Twist cmd;
	
	cmd.linear.x = v;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = w;



	std::cout<<"diffpose:"<<diffpose<<std::endl;
	std::cout<<"angle:"<<angle2goal<<std::endl;

	std::cout<<"v:"<<v<<std::endl;
	std::cout<<"w:"<<w<<std::endl;	
	cmdpub_t.publish(cmd);
	return niter;
}

int
go2goal(Object o)	
{
	float diffpose;
	int niter=0;

 std::cout<<"x:"<<pose1.cx<<std::endl;
 std::cout<<"y:"<<pose1.cy<<std::endl;	
	

	diffpose = sqrt( (pose.pose.pose.position.x-o.cx)*(pose.pose.pose.position.x-o.cx)+ (pose.pose.pose.position.y-o.cy)*(pose.pose.pose.position.y-o.cy)); 
	
	niter = diffpose * 37;

	double roll, pitch, yaw;
	tf::Quaternion q(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, 1.0);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);



				
	float v,w;
	float angle2goal;
	if(diffpose < 0.1){	
		w = v = 0.0;
	}else{
		//va de 3.1/-3.1 a 0
		// 133 1 vuelta 37 1 m

		angle2goal = normalizePi(atan2(o.cy - pose.pose.pose.position.y, o.cx -pose.pose.pose.position.x) - yaw);

		if(fabs(angle2goal) > 0.1)
		{
	 std::cout<<"el angulo deberia ser menos a 0.1 wtf"<<std::endl;	
			w = 0.0;
			v = 0.0;
		}else{
			w = 0.0;
			v = 0.35;
		}
	}

	geometry_msgs::Twist cmd;
	
	cmd.linear.x = v;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = w;



	std::cout<<"diffpose:"<<diffpose<<std::endl;
	std::cout<<"angle:"<<angle2goal<<std::endl;

	std::cout<<"v:"<<v<<std::endl;
	std::cout<<"w:"<<w<<std::endl;	
	cmdpub_t.publish(cmd);
	return niter;
}
void
go2gpos(Object o)	
{
	float diffpose;

 std::cout<<"x:"<<pose1.cx<<std::endl;
 std::cout<<"y:"<<pose1.cy<<std::endl;	
// std::cout<<"z:"<<pose.pose.pose.orientation.z<<std::endl;
 //std::cout<<"w:"<<fakepose.pose.pose.orientation.w<<std::endl;	



	diffpose = sqrt( (fakepose.pose.pose.position.x-o.cx)*(fakepose.pose.pose.position.x-o.cx)+ (fakepose.pose.pose.position.y-o.cy)*(fakepose.pose.pose.position.y-o.cy)); 
	
	double roll, pitch, yaw;

	tf::Quaternion q(fakepose.pose.pose.orientation.x, fakepose.pose.pose.orientation.y, fakepose.pose.pose.orientation.z, fakepose.pose.pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);



	/****************************************FAKE************************
	diffpose = sqrt( (pose.pose.pose.position.x-o.cx)*(pose.pose.pose.position.x-o.cx)+ (pose.pose.pose.position.y-o.cy)*(pose.pose.pose.position.y-o.cy)); 
	
	double roll, pitch, yaw;
	//pose.pose.pose.orientation.z to 1.0
	tf::Quaternion q(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, 1.0);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);


	diffpose = sqrt( (fakepose.pose.pose.position.x-o.cx)*(fakepose.pose.pose.position.x-o.cx)+ (fakepose.pose.pose.position.y-o.cy)*(fakepose.pose.pose.position.y-o.cy)); 
	
	double roll, pitch, yaw;

	tf::Quaternion q(fakepose.pose.pose.orientation.x, fakepose.pose.pose.orientation.y, fakepose.pose.pose.orientation.z, fakepose.pose.pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	****************************************FAKE************************/
				
	float v,w;
	float angle2goal;
	if(diffpose < 0.1){	
		w = v = 0.0;
	}else{

		angle2goal = normalizePi(atan2(o.cy - fakepose.pose.pose.position.y, o.cx -fakepose.pose.pose.position.x) - yaw);	
	
		/****************************************FAKE************************
		angle2goal = normalizePi(atan2(o.cy - pose.pose.pose.position.y, o.cx -pose.pose.pose.position.x) - yaw);
		angle2goal = normalizePi(atan2(o.cy - fakepose.pose.pose.position.y, o.cx -fakepose.pose.pose.position.x) - yaw);	
	****************************************FAKE************************/


		if(fabs(angle2goal) > 0.1)
		{
			w = (angle2goal/fabs(angle2goal)) * 0.5;
			v = 0.0;
		}else{
			w = 0.0;
			v = 0.35;
		}
	}

	geometry_msgs::Twist cmd;
	
	cmd.linear.x = v;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = w;



	std::cout<<"diffpose:"<<diffpose<<std::endl;
	std::cout<<"angle:"<<angle2goal<<std::endl;

	std::cout<<"v:"<<v<<std::endl;
	std::cout<<"w:"<<w<<std::endl;	
	cmdpub_t.publish(cmd);
}


void
go2pos(Object o)	
{
	float v,w;
	float a=o.cy;
	if(o.cy >0.7){
		w=0.4;
	}else if(o.cy >0.1){
		w=0.3;
	}else if(o.cy <-0.1){
		w=-0.3;
	}else if(o.cy <-0.7){
		w=-0.4;
	}else{
		w=0.0;
	}

	if(o.cx>1){
		v=0.3;
	}else if(o.cx >0.5 and o.cx <1){
		v=0.2;
	}else{
		v=0.0;
	}

	geometry_msgs::Twist cmd;
	
	cmd.linear.x = v;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = w;

	
	cmdpub_t.publish(cmd);
}


// void Behav::goalCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//   goalpose = *msg;
//   goalrecv = true;
// }

ball getTarget(){

		Object o;
		bool found=false;
		for(int i=0;i<num_objects;i++){
			if (array[i].o.name.compare(emp)!= 0){
				if(array[i].found==false){
					return array[i];
				}
			}
		}
 	}
void
peep(){

		sound_cmd.value = 6;
		cmdpub_s.publish(sound_cmd);

}

bool hayTarget() {
	int i=0;
	bool hay=false;
	while(i<num_objects){
		if (array[i].o.name.compare(emp)!= 0){
			if(array[i].found==false)	//poner a false cuando encuentro bolas
				hay =true;
		}
		i++;
	}
	return hay;
}

bool isPrefix(std::string const& s1, std::string const&s2)
{
	return s1.compare(s2.substr(0, s1.length()))==0;

}

void borrarpelotas(ball b){
	if(b.time <= ros::Time::now() - ros::Duration(1.0) and b.found==false){

		for (int i = 0; i < num_objects; ++i){ 
			if(array[i].o.name.compare(b.o.name)==0){
				array[i].o.name="empty";
			}	
		}
	}
}

void lost() {

	geometry_msgs::Twist cmd;

	if((fabs(pose1.cx)>2.0 or fabs(pose1.cy)>2.0)){
		go2gpos(center);
	}else if(fabs(pose1.cx)>0.4 or fabs(pose1.cy)>0.4){

	}else if(fabs(pose1.cx)<0.4 or fabs(pose1.cy)<0.4){
	
	cmd.linear.x = 0.3;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = 0.6;

	
	cmdpub_t.publish(cmd);
	}
}

bool terminado() {
		int i=0;
		int k=0;

		while(i<num_objects and k<num_objects){
			if(array[i].o.name.compare(emp)!= 0 and array[i].found==true){
				k++;
			}
			i++;
		}
		if(k==num_objects){
			return true;
		}else{
			return false;
		}
	}
/*
states:
	begin: vuelta para buscar pelotas, elegimos(+cerca o mas lejos de goal?)
	search:vamos a la pelota elegida(beep)
	rescue:vamos a goal.
		si tenemos pelotas guardadas de la primera vuelta nos dirigimos hacia alli ->search
		si no-> lost:vamos al centro
	lost:vamos al centro y luego vueltas espirales crecientes.

	   se va guardando pelotas todo el tiempo mientras	
		necesito un array de pelotas y el goal
	tf->3coor 1 string(roja grande)
*/

int i = 0;
int main(int argc, char **argv)
{


	ros::init(argc, argv, std::string("behav"));


	ros::NodeHandle n;

	tf::TransformListener tfL;
	

	ros::Rate loop_rate(10);
	int count = 0;
	bool ball_in_balls = false;

	inline double normalizePi(double data);

	//robotposesub = n.subscribe("/robot_pos", 1000, &Behav::poseCB, this);
	submodelsub = n.subscribe("/gazebo/model_states", 1000, &fakeposeCB);
	cmdpub_t = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
	cmdpub_s = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1); 


	for (int i = 0; i < num_objects; ++i){
	    array[i].o.name = emp;
	    array[i].o.cx = 0.0;	//cuidado
	    array[i].o.cy = 0.0;
	    array[i].o.cz = 0.0;
	    array[i].found = true;
	    array[i].time = ros::Time::now();	
	}	

	 goal.name="Goal";
	 goal.cx=-3.1;
	 goal.cy=-2.1;
	 goal.cz=0.0;
	 center.name="Center";
	 center.cx=0.0;
	 center.cy=0.0;
	 center.cz=0.0;

 	 state = 0;   

 	 ball target;
 	 ball prueba;
 	 Object p_ball;
 	p_ball.name="pelota";
	p_ball.cx=2.0;
	p_ball.cy=-0.2;
	p_ball.cz=0.0;
	prueba.o=p_ball;
	prueba.found=false;


	int niter;
	int iter=0;
	/////////////////////////////////////////////////
	
	while (ros::ok())
	{	


		tfL.getFrameStrings(frameList);
		balls.clear();
		for (it = frameList.begin(); it != frameList.end(); ++it) {
			std::string frame = *it;
			if(isPrefix("pelota_", frame)){
				balls.push_back(frame);
			}
		}

		tf::StampedTransform L2W;

int k = 0;
		for (it = balls.begin(); it != balls.end(); ++it) {
	   ROS_INFO("it %d", k);
		   std::cout<<"it:"<<*it<<std::endl;
k++;
			ball_in_balls = false;
	

				for (int i = 0; i < num_objects; ++i){ 
					if(array[i].o.name.compare(*it)==0 and ball_in_balls==false){
						ball_in_balls  = true;
					}	
				}


			try {
					//cambio de tf 
				tfL.lookupTransform("base_link", *it, //poner inverse si sale negativo
					ros::Time::now() - ros::Duration(0.2), L2W);
				  for (int i = 0; i < num_objects; ++i){ 
		   std::cout<<"i: "<<i<<std::endl;
				  	//bola nueva 
					if(array[i].o.name.compare("empty")==0 and ball_in_balls==false){ //comprobar length
				   std::cout<<"???????????????????NUEVA?????????????????"<<std::endl;
				  		    array[i].o.name = *it;
						    array[i].o.cx = L2W.getOrigin().x();	
						    array[i].o.cy = L2W.getOrigin().y();
						    array[i].o.cz = L2W.getOrigin().z();
						    array[i].found = false;
	    					    array[i].time = ros::Time::now();
						    ball_in_balls=true;
						  

				  	//actualiza
				  	}else if(array[i].o.name.compare(*it)==0){ 
		   std::cout<<"!!!!!!!!!!!!!!!actualiza!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
				  		if(array[i].found==false){
						    array[i].o.cx = L2W.getOrigin().x();	
						    array[i].o.cy = L2W.getOrigin().y();
						    array[i].o.cz = L2W.getOrigin().z();
						    array[i].time = ros::Time::now();
				  		}
 					}
				  }


			} catch (tf::TransformException & ex) {
				//ROS_WARN("%s", ex.what());
			}
		}
		 for (int i = 0; i < num_objects; ++i){ 
		   std::cout<<"bolas:"<<array[i].o.name<<std::endl;
		 }

	 	switch (state){
		     case begin:
		     			 		std::cout<<"begin:"<<std::endl;
		     	lost();

				if(hayTarget()){

					state = search;
				}else{
					if(terminado())
						state= end;
					else
						state=begin;
				}
			 	break;
		     case search:
		     	std::cout<<"search:"<<std::endl;
		     	target = getTarget();
		     	//target = prueba;
				go2pos(target.o);

				std::cout<<"x:"<<target.o.cx<<std::endl;
				std::cout<<"y:"<<target.o.cy<<std::endl;

				if(target.o.cx< 0.68){
			 		target.found=true;
					for (int i = 0; i < num_objects; ++i){ 
						if(array[i].o.name.compare(target.o.name)==0){
							array[i].found=true;
						}	
					}
					peep();
					state= girydist;
					break;
				}
				borrarpelotas(target);

				if(hayTarget()){
					state = search;
				}else{
					if(terminado())
						state= end;
					else
						state=begin;
				}


				
				break;
			 case girydist:
			 	std::cout<<"GIRYDIST"<<std::endl;
					iter=0;
			 	int ready;
			 	ready = gira(goal);
	std::cout<<"ready:"<<ready<<std::endl;
				if(ready==0){
			 		state = rescue;
				}
				break;
			 
			 case rescue:
			 		std::cout<<"RESCUE:"<<std::endl;
			 	if (iter==0){
			 		niter=go2goal(goal);
			 	}else{
			 		go2goal(goal);
			 	}
	// std::cout<<"POS = x:"<<pose1.cx<<std::endl;
	// std::cout<<"POS = y:"<<pose1.cy<<std::endl;	

			 	//if (goal.cx<pose1.cx+0.25 and goal.cx>pose1.cx-0.25 and goal.cy>pose1.cy-0.25 and goal.cy<pose1.cy+0.25){
			 	if (iter>=niter){
			 		peep();
			 		iter=0;
				 	if(hayTarget())
						state = search;
					else{
						if(terminado())
							state= end;
						else
							state=begin;
					}
			 	}
			 	iter++;
			 	break;
			 case end:	
	
				geometry_msgs::Twist cmd;
	
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.7;



					//std::cout<<"diffpose:"<<diffpose<<std::endl;
					//std::cout<<"angle:"<<angle2goal<<std::endl;

					//std::cout<<"v:"<<v<<std::endl;
					//std::cout<<"w:"<<w<<std::endl;
	
					
				if(i < 133){
					cmdpub_t.publish(cmd);
					ROS_INFO("iteracion: %d",  i);
					i++;
				}


			 break;
		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;




	}
			return 0;
}
