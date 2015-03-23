#include "saveVictims.h"
#include "pid.h"


SaveVictims::SaveVictims()
{
  
  sub_img = n.subscribe("/image_converter/output_video", 1, &SaveVictims::chatterCallback, this);
  sub_suelo = n.subscribe("/image_converter/output_video_suelo", 1, &SaveVictims::localizaSuelo, this);
  chatter_pub_vel = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
  chatter_pub_sound = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);     
  state = 0;
  x = 0.0;
  y = 0.0;
  vmh = 0.0;
  vmv = 0.0;
  pixCount = 0;
  contH = 0;
  contV = 0;
  for (int i = 0; i < size; ++i)
    arrayH[i] = 0.0;
  for (int i = 0; i < size; ++i)
    arrayV[i] = 0.0;
  naranjaP = false;
  naranjaG = false;
  roja = false;
}

float 
SaveVictims::dameMediaH(float val){
  float sum = 0.0;
  arrayH[contH] = val;
  if (contH == 9)
    contH = (contH+1)%10;
  else
    contH++;
  for (int i = 0; i < size; ++i)
    sum = sum + arrayH[i];
  return sum/10;
}

float 
SaveVictims::dameMediaV(float val){
  float sum = 0.0;
  arrayV[contV] = val;
  if (contV == 9)
    contV = (contV+1)%10;
  else
    contV++;
  for (int i = 0; i < size; ++i)
    sum = sum + arrayV[i];
  return sum/10;
}

void 
SaveVictims::localizaSuelo(const sensor_msgs::Image& img){
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat hsvf;  
  cv::cvtColor (cv_ptr->image, hsvf, CV_RGB2HSV);

  int height = hsvf.rows;
  int width = hsvf.cols;
  int step = hsvf.step;
  int channels = 3;

  sPixCount = 0;
  sx= 0.0;
  sy = 0.0;
  for (uint i = 0; i < height; i++)  
    {
      for (uint j = 0; j < width; j++)
      {
        int posdata = i*step+j*channels;
        if (hsvf.data[posdata] != 0)
        {

          sPixCount++;
          sx = sx +i;
          sy = sy +j;
        }

      }
    }
}

void 
SaveVictims::chatterCallback(const sensor_msgs::Image& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat hsvf;  
  cv::cvtColor (cv_ptr->image, hsvf, CV_RGB2HSV);

  int height = hsvf.rows;
  int width = hsvf.cols;
  int step = hsvf.step;
  int channels = 3;

  
  xp = x;
  yp = y;
  pPixCount = pixCount;
  pvmh = vmh;
  pvmv = vmv;
  pixCount = 0;
  x= 0.0;
  y = 0.0;
  vmh = 0.0;
  vmv = 0.0;
  float vm = 0.0;
  for (uint i = 0; i < height; i++)  
    {
      for (uint j = 0; j < width; j++)
      {
        int posdata = i*step+j*channels;
        if (hsvf.data[posdata] != 0)
        {
          vmh = vmh + hsvf.data[posdata];
          vmv = vmv + hsvf.data[posdata+2];
          pixCount++;
          x = x+i;
          y = y +j;
        }

      }
    }
   ROS_INFO("pixeles: [%i]", pixCount);
  switch (state){
      case Buscando:
        base_cmd.angular.z = 0.8;
        base_cmd.linear.x = 0.0;
        if (pixCount > 60)
          state = Avanzando;
        //else{
         // if (sPixCount > 200000)
         // {
         //   ROS_INFO("pixeles.......................................................................: [%i]", sPixCount);
         //   w = sy/320 -1;
         //   Pid pids;
         //   vz = pids.getPid(w);
         //   base_cmd.angular.z = vz;
         //   base_cmd.linear.x = 0.2;
         // }else{
         //   base_cmd.angular.z = 0.5;
         //   base_cmd.linear.x = 0.0;
         // }
        //}        
        break;
      case Avanzando:
        if(pixCount > 60){
          mediaH = dameMediaH(vmh/(float)pixCount);
          mediaV = dameMediaV(vmv/(float)pixCount);
          if ((mediaH < 107) && (naranjaG == true))
            break;
          else if((mediaH > 107) && (mediaV > 180) && (naranjaP == true))
            break;
          else if ((mediaH > 107) && (mediaV < 180) && (roja == true))
            break;
          else{          
            x = x/(float)pixCount;
            y = y/(float)pixCount;
            w = y/320 -1;
            Pid pid;
            vz = pid.getPid(w);
            base_cmd.angular.z = -1.0*2*vz;
            base_cmd.linear.x = 0.2;
          }
          //ROS_INFO("Giro: [%f]", vz);
        }else{
          //ROS_INFO("xp: [%f]", xp);
          if ((xp > 400) && (yp > 40) && (yp < 600))
          {
            base_cmd.linear.x = 0.0;
            base_cmd.angular.z = 0.0;
            state = Encontrado;
          }else
            state = Buscando;
        }
        break;
      case Encontrado:
          ROS_INFO("MediaH: [%f]", mediaH);
          ROS_INFO("MediaV: [%f]", mediaV);
          if (mediaH < 107 && (naranjaG == false)){
            naranjaG = true;
            ROS_INFO("Encontrada naranja grande");
          }
          else if(mediaH > 107 && mediaV > 185 && (naranjaP == false)){
            naranjaP = true;
            ROS_INFO("Encontrada naranja peque");
          }
          else if (mediaH > 107 && mediaV < 185 && (roja == false)){ 
            roja = true;
            ROS_INFO("Encontrada roja");
          }
          if((naranjaP == true) && (naranjaG == true) && (roja == true))
            state = Final;
          else
            state = Buscando;
          chatter_pub_sound.publish(base_sound);
          break;
      case Final:
        sleep(2);
        base_sound.value = 1;
        chatter_pub_sound.publish(base_sound);
        exit(0);
    }

  chatter_pub_vel.publish(base_cmd);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "saveVictims");
  SaveVictims sv;
  ros::spin();
  return 0;
}
