/*
 * 
 * 
 * Autor: Francisco Martín Rico (fmrico@gmail.com)
 * Fecha: 11/02/2014
 *  
 * Programa de prueba de filtrado de imagen en HSV para asignatura de robótica en la URJC
 * 
 * La imagen proviene de la imagen publicada en /camera/rgb/image_raw ; cambiar si es otra
 * 
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types_conversion.h>

enum{
	hupink = 0,
	hlpink = 255,
	supink = 0,
	slpink = 255,
	vupink = 0,
	vlpink = 255,

	hublue = 0,
	hlblue = 255,
	sublue = 0,
	slblue = 255,
	vublue = 0,
	vlblue = 255,

	huyellow = 0,
	hlyellow = 255,
	suyellow = 0,
	slyellow = 255,
	vuyellow = 0,
	vlyellow = 255,
};

class ImageConverter3D {
	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;

	//int hupper, hlower;
	//int supper, slower;
	//int vupper, vlower;

public:
	ImageConverter3D() {
		//std::string topic = nh_.resolveName("point_cloud");
		image_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
				"/camera/depth_registered/points", 1,
				&ImageConverter3D::imageCb, this);
		image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_filtered", 1);

		cvNamedWindow("Imagen Fuente");
		cvNamedWindow("Imagen filtrada");

		// TrackBar
		//cvCreateTrackbar("Hue Upper", "Imagen filtrada", &hupper, 360, NULL);
		//cvCreateTrackbar("Hue Lower", "Imagen filtrada", &hlower, 360, NULL);
		//cvCreateTrackbar("Sat Upper", "Imagen filtrada", &supper, 360, NULL);
		//cvCreateTrackbar("Sat Lower", "Imagen filtrada", &slower, 360, NULL);
		//cvCreateTrackbar("Val Upper", "Imagen filtrada", &vupper, 360, NULL);
		//cvCreateTrackbar("Val Lower", "Imagen filtrada", &vlower, 360, NULL);
		//cvCreateButton("Save",ImageConverter::callbackButton,NULL,CV_PUSH_BUTTON,0);

	}

	~ImageConverter3D() {
		cv::destroyWindow("Imagen Fuente");
		cv::destroyWindow("Imagen filtrada");
	}

	void imageCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
		pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb, PCxyzrgbout;
		sensor_msgs::PointCloud2 out;

		//PointCloud2 -> pcl::PointCloud<pcl::PointXYZRGB>
		pcl::fromROSMsg(*msg, PCxyzrgb);

		//Copy original point cloud to the resulting one.
		//The points in the resulting point cloud are removed completely.
		//The points of this resulting point cloud will be added later, depending on the H filtering process.
		PCxyzrgbout = PCxyzrgb;
		PCxyzrgbout.clear();
		
		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		pcl::PointCloud<pcl::PointXYZRGB>::iterator pinkit;
		pcl::PointCloud<pcl::PointXYZRGB>::iterator blueit;
		pcl::PointCloud<pcl::PointXYZRGB>::iterator yellowit;
			
		it = PCxyzrgb.begin();
		pinkit = PCxyzrgb.begin();
		blueit = PCxyzrgb.begin();
		yellowit = PCxyzrgb.begin();
		
		while(it != PCxyzrgb.end()) {
			pcl::PointXYZHSV hsv;
			pcl::PointXYZRGBtoXYZHSV(*it, hsv);

			//if the point is in the H range, it is added to the resulting point cloud. On the other hand, paint it black (to be displayed later)
			if (((hsv.h >= hlpink) && (hsv.h <= hupink)) && ((hsv.s >= slpink) && (hsv.s <= supink)) && ((hsv.v >= vlpink) && (hsv.v <= vupink))){
				PCxyzrgbout.push_back(*pinkit);
				PCxyzrgbout.push_back(*it);
				blueit->r = yellowit->r = 0;
				blueit->g = yellowit->g = 0;
				blueit->b = yellowit->b = 0;
			}
			else if(((hsv.h >= hlblue) && (hsv.h <= hublue)) && ((hsv.s >= slblue) && (hsv.s <= sublue)) && ((hsv.v >= vlblue) && (hsv.v <= vublue))){
				PCxyzrgbout.push_back(*blueit);
				PCxyzrgbout.push_back(*it);
				pinkit->r = yellowit->r = 0;
				pinkit->g = yellowit->g = 0;
				pinkit->b = yellowit->b = 0;
			}else if(((hsv.h >= hlyellow) && (hsv.h <= huyellow)) && ((hsv.s >= slyellow) && (hsv.s <= suyellow)) && ((hsv.v >= vlyellow) && (hsv.v <= vuyellow))){
				PCxyzrgbout.push_back(*yellowit);
				PCxyzrgbout.push_back(*it);
				pinkit->r = blueit->r = 0;
				pinkit->g = blueit->g = 0;
				pinkit->b = blueit->b = 0;
			}
			else {
				pinkit->r = blueit->r = yellowit->r = it->r = 0;
				pinkit->g = blueit->g = yellowit->g = it->g = 0;
				pinkit->b = blueit->b = yellowit->b = it->b = 0;
			}
			it++;
			pinkit++;
			blueit++;
			yellowit++;
		}

		//tansform PCxyzrgb (pcl::PointCloud<pcl::PointXYZRGB>) to Image to display in the OpenCV window
		pcl::toROSMsg(PCxyzrgb, out);
		sensor_msgs::Image image;
		cv_bridge::CvImagePtr cv_imageout;

		pcl::toROSMsg(out, image);
		cv_imageout = cv_bridge::toCvCopy(image,
				sensor_msgs::image_encodings::BGR8);

		cv::imshow("Imagen filtrada", cv_imageout->image);
		cv::waitKey(3);

		//Get the mean position of the resulting point cloud

		float px, py, pz;
		float bx, by, bz;
		float yx, yy, yz;
		px = py = pz = 0.0;
		bx = by = bz = 0.0;
		yx = yy = yz = 0.0;
		
		int pc, bc, yc = 0;

		pinkit = PCxyzrgbout.begin();
		blueit = PCxyzrgbout.begin();
		yellowit = PCxyzrgbout.begin();

		for (it = PCxyzrgbout.begin(); it != PCxyzrgbout.end(); ++it) {

			if (pinkit->x == pinkit->x){ //This seems to be the only way to detect if it is NaN	{
				px = px + pinkit->x;
				py = py + pinkit->y;
				pz = pz + pinkit->z;
				pc++;
			}else if(blueit->x == blueit->x){
				bx = bx + blueit->x;
				by = by + blueit->y;
				bz = bz + blueit->z;
				bc++;
			}else if(yellowit->x == yellowit->x){
				yx = yx + yellowit->x;
				yy = yy + yellowit->y;
				yz = yz + yellowit->z;
				yc++;
			}
			pinkit++;
			blueit++;
			yellowit++;

		}

		if(pc > 0) {
			px = px / (float) pc;
			py = py / (float) pc;
			pz = pz / (float) pc;
		}
		if(bc > 0) {
			bx = bx / (float) bc;
			by = by / (float) bc;
			bz = bz / (float) bc;
		}
		if(yc > 0) {
			yx = yx / (float) yc;
			yy = yy / (float) yc;
			yz = yz / (float) yc;
		}
		std::cout << "Center of the point cloud filtered PINK = (" << px << ", " << py
				<< ", " << pz << ") [" << pc << "]" << std::endl;
		std::cout << "Center of the point cloud filtered BLUE = (" << bx << ", " << by
				<< ", " << bz << ") [" << bc << "]" << std::endl;
		std::cout << "Center of the point cloud filtered YELLOW = (" << yx << ", " << yy
				<< ", " << yz << ") [" << yc << "]" << std::endl;

		//tranform PCxyzrgbout (pcl::PointCloud<pcl::PointXYZRGB>) to PointCloud2 in order to be published
		sensor_msgs::PointCloud2 pcout;
		pcl::toROSMsg(PCxyzrgbout, pcout);
		image_pub_.publish(pcout);

	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter3D");
	ImageConverter3D ic;
	ros::spin();
	return 0;
}
