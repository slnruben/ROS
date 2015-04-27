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

#include "Behav.h"

#include <vector>

// Behav::Behav() {


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

Behav::~Behav() {

}

inline double
Behav::normalizePi(double data)
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

void Behav::poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose = *msg;
}



void
Behav::go(Object o)	
{
	float diffpose;
	
	diffpose = sqrt( (pose.pose.pose.position.x-o.cx)*(pose.pose.pose.position.x-o.cx)+ (pose.pose.pose.position.y-o.cy)*(pose.pose.pose.position.y-o.cy)); 
	
	double roll, pitch, yaw;
	tf::Quaternion q(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

				
	float v,w;
	float angle2goal;
	if(diffpose < 0.1){	
		w = v = 0.0;
	}else{

		angle2goal = normalizePi(atan2(goalpose.pose.position.y - pose.pose.pose.position.y, goalpose.pose.position.x -pose.pose.pose.position.x) - yaw);
				

		if(fabs(angle2goal) > 0.1)
		{
			w = (angle2goal/fabs(angle2goal)) * 0.3;
			v = 0.0;
		}else{
			w = 0.0;
			v = 0.3;
		}
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

ball Behav::getTarget(){

		Object o =NULL;
		found=false;
		for(int i=0;i<array.length;i++){
			if (array[i]!= NULL){
				if(array[i].found==false){
					return array[i];
				}
			}
		}
 	}
void
Behav::peep(){
		sound_cmd.value = 3;
		cmdpub_s.publish(sound_cmd);
}

bool Behav::hayTarget() {
	int i=0;
	bool hay=false;
	while(i<num_objects){
		if (strncmp(array[i].o.name,"empty" ,5)!= 0){
			if(array[i].found==false)	//poner a false cuando encuentro bolas
				hay =true;
		}
		i++;
	}
	return hay;
}

/////
bool Behav::terminado() {
		int i=0;
		int k=0;

		while(i<num_objects and k<num_objects){
			if(strncmp(array[i].o.name,"empty" ,5)!= 0 and array[i].found==true){
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


int main(int argc, char **argv)
{


	ros::init(argc, argv, std::string(argv[1]));

	tf::TransformListener tf;

	ros::Rate loop_rate(10);
	int count = 0;
	int f_balls = 0;
	bool ball_in_balls = false;
///////////////////////////////////////////////////77
// 	void correct();
// 	std::string id;

// 	ros::NodeHandle n;
// 	ros::Publisher posePub;

// 	tf::TransformBroadcaster tfB;

// 	geometry_msgs::PoseWithCovarianceStamped pose;
// 	geometry_msgs::PoseStamped goalpose;
// 	  kobuki_msgs::Sound sound_cmd;
// 	bool goalrecv = false;

// 	Markov markov;
// 	QuadMK quadmk;
// 	MCL mcl;

// ros::Subscriber robotposesub ;
// 	//ros::Subscriber submodelsub = n.subscribe("/gazebo/model_states", 1000, &fakeposeCB::poseCB, this);
// 	ros::Subscriber goalsub ;
// 	ros::Publisher  cmdpub_t ;
// 	ros::Publisher  cmdpub_s ; 


// //----------------------------------------------------------
// 	inline double normalizePi(double data);

// //----------------------------------------------------------
// 	struct Object{
// 		std::string name;
// 		float cx;
// 		float cy;
// 		float cz;
// 	};

// 	struct ball{
// 		Object o;
// 		bool found;
// 	};
// 	int state;
// 	static const int num_objects = 3;	
// 	static const int begin=0;
// 	static const int lost=1;
// 	static const int search=2;
// 	static const int end=3;

// 	ball array [num_objects];
// 	Object goal;
// 	Object center;
// 	Object pose1;
	
// 	//tfs

// 	std::vector<std::string> balls;
// 	std::vector<std::string> frameList;
// 	std::vector<std::string>::iterator it;
// 	//·///////////////////////////////////////////////////////////////////////////////////////
// 	cmdpub_t = n.advertise<geometry_msgs::Twist>("/robot/commands/velocity", 1000);
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
	/////////////////////////////////////////////////777
	
	while (ros::ok())
	{	
		tfL.getFrameString(frameList);
		for (it = frameList.begin(); it != frameList.end(); ++it) {
			std::string frame = *it;
			if(isPrefix("pelota_", frame)){
				balls.push_back(frame);
			}
		}

		tf::StampedTransform L2W;

		for (it = balls.begin(); it != balls.end(); ++it) {
			ball_in_balls = false;

			try {	
					//cambio de tf 
				tfL.lookupTransform("world", *it, //poner inverse si sale negativo
					ros::Time::now() - ros::Duration(0.2), L2W);

				  for (int i = 0; i < num_objects; ++i){ 
				  	//actualiza pos 
					if(strncmp(array[i].o.name,*it ,5)){ //comprobar length
				  		if(array[f_balls].found==false){
						    array[f_balls].o.cx = L2W.getOrigin().x();	
						    array[f_balls].o.cy = L2W.getOrigin().y();
						    array[f_balls].o.cz = L2W.getOrigin().z();
						    ball_in_balls= true;
				  		}
				  	//mete bola nueva
				  	}else if(strncmp(array[i].o.name,"empty" ,5) and ball_in_balls==false){ 
				  			array[f_balls].o.name = *it;
						    array[f_balls].o.cx = L2W.getOrigin().x();	
						    array[f_balls].o.cy = L2W.getOrigin().y();
						    array[f_balls].o.cz = L2W.getOrigin().z();
						    array[f_balls].found = false;
						    ball_in_balls= true;
 					}
				  }


			} catch (tf::TransformException & ex) {
				ROS_WARN("%s", ex.what());
			}
		}



		if(goalrecv)
		{
			if(goalpose.header.frame_id == "world")
			{
			  switch (state){
			     case begin:
					go(center);

					if(hayTarget())
						state = search;
					else{
						if(terminado())
							state= end;
						else
							state=begin;
					}
				 	break;
			     case search:
			     	target = getTarget();
					go(target);
					if(target.o.cx<0.5){
						target.found=true;
						peep();
						state= rescue;
					}					
					break;
				 case rescue:
				 	go(goal);
				 	if (goal.cx<pose1.cx+0.25 and goal.cx>pose1.cx-0.25){
					 	if(hayTarget())
							state = search;
						else{
							if(terminado())
								state= end;
							else
								state=lost;
						}
				 	}
				 	break;
				 case end:	
				 break;
		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
