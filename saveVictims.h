#ifndef saveVictims_H
#define saveVictims_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Sound.h>
#include <sensor_msgs/Image.h>
#include <time.h>




class SaveVictims
{
private:
  ros::NodeHandle n;
  geometry_msgs::Twist base_cmd;
  kobuki_msgs::Sound base_sound;
  ros::Subscriber sub_img;
  ros::Subscriber sub_suelo;
  ros::Publisher chatter_pub_vel;
  ros::Publisher chatter_pub_sound;
  int state, pixCount, sPixCount, pPixCount, contH, contV;
  float xp, yp, x, y, w, vz, sx, sy, vmh, vmv,pvmh, pvmv, mediaH, mediaV;
  bool naranjaP, roja, naranjaG; 
  static const int Buscando=0;
  static const int Avanzando=1;
  static const int Encontrado=2;
  static const int Final=3;
  static const int size = 10;
  float arrayH [size];
  float arrayV [size];


public: 
  SaveVictims();
  void chatterCallback(const sensor_msgs::Image& img);
  void localizaSuelo(const sensor_msgs::Image& img);
  float dameMediaH(float val);
  float dameMediaV(float val);
};

#endif