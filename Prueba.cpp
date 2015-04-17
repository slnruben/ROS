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

enum{ORANGE, BIGORANGE, RED, BLUE, YELLOW, PINK, NUM_COLORS};

typedef struct NodeColor NodeColor;
typedef struct ListObjetos ListObjetos;
struct NodeColor{
	float cx;
	float cy;
	float cz;
	int total;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	NodeColor *next;
	NodeColor *prev;
};

struct ListObjetos{
	NodeColor *list;
};

class ImageConverter3D {
	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;
	
	ListObjetos objetos[NUM_COLORS];

	int HUORANGE; //ImageConverter3D::HURANGE
	int HLORANGE;
	int SUORANGE;
	int SLORANGE;

	int HUBIGORANGE;
	int HLBIGORANGE;

	int HURED;
	int HLRED;
	int SURED;
	int SLRED;

	int HUBLUE; //265;
	int HLBLUE; //150;

	int HUYELLOW;
	int HLYELLOW;
	
	int HUPINK;
	int HLPINK;

public:
	ImageConverter3D() {
		//std::string topic = nh_.resolveName("point_cloud");
		image_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
				"/camera/depth_registered/points", 1,
				&ImageConverter3D::imageCb, this);
		image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_filtered", 1);
		HUORANGE = 360; //ImageConverter3D::HURANGE
		HLORANGE = 336;
		SUORANGE = 360;
		SLORANGE = 276;
	
		HUBIGORANGE = 35;
		HLBIGORANGE = 0;
	
		HURED = 360;
		HLRED = 336;
		SURED = 275;
		SLRED = 180;
	
		HUBLUE = 360; //265;
		HLBLUE = 360; //150;
	
		HUYELLOW = 149;
		HLYELLOW = 36;
		
		HUPINK = 335;
		HLPINK = 266;

		cvNamedWindow("Imagen filtrada");
		cvNamedWindow("Filtrador Pelotas");
		cvNamedWindow("Filtrador Balizas");

		cvCreateTrackbar("Hue Upper ORANGE", "Filtrador Pelotas", &HUORANGE, 360, NULL);
		cvCreateTrackbar("Hue Lower ORANGE", "Filtrador Pelotas", &HLORANGE, 360, NULL);
		cvCreateTrackbar("Sat Upper ORANGE", "Filtrador Pelotas", &SUORANGE, 360, NULL);
		cvCreateTrackbar("Sat Lower ORANGE", "Filtrador Pelotas", &SLORANGE, 360, NULL);

		cvCreateTrackbar("Hue Upper RED", "Filtrador Pelotas", &HURED, 360, NULL);
		cvCreateTrackbar("Hue Lower RED", "Filtrador Pelotas", &HLRED, 360, NULL);
		cvCreateTrackbar("Sat Upper RED", "Filtrador Pelotas", &SURED, 360, NULL);
		cvCreateTrackbar("Sat Lower RED", "Filtrador Pelotas", &SLRED, 360, NULL);

		cvCreateTrackbar("Hue Upper BIGORANGE", "Filtrador Pelotas", &HUBIGORANGE, 360, NULL);
		cvCreateTrackbar("Hue Lower BIGORANGE", "Filtrador Pelotas", &HLBIGORANGE, 360, NULL);

		cvCreateTrackbar("Hue Upper BLUE", "Filtrador Balizas", &HUBLUE, 360, NULL);
		cvCreateTrackbar("Hue Lower BLUE", "Filtrador Balizas", &HLBLUE, 360, NULL);

		cvCreateTrackbar("Hue Upper PINK", "Filtrador Balizas", &HUPINK, 360, NULL);
		cvCreateTrackbar("Hue Lower PINK", "Filtrador Balizas", &HLPINK, 360, NULL);

		cvCreateTrackbar("Hue Upper YELLOW", "Filtrador Balizas", &HUYELLOW, 360, NULL);
		cvCreateTrackbar("Hue Lower YELLOW", "Filtrador Balizas", &HLYELLOW, 360, NULL);

	}

	~ImageConverter3D() {
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
		initObjetos();
		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		for (it = PCxyzrgb.begin(); it != PCxyzrgb.end(); ++it) {
			pcl::PointXYZHSV hsv;
			pcl::PointXYZRGBtoXYZHSV(*it, hsv);
			//if the point is in the H range, it is added to the resulting point cloud. On the other hand, paint it black (to be displayed later)
			if (((hsv.h >= HLORANGE) && (hsv.h <= HUORANGE)) && ((hsv.s >= ((float)SLORANGE/360)) && (hsv.s <= ((float)SUORANGE/360)))){
				//ROS_INFO("Filtra Naranja");
				addNode(ORANGE, it->x, it->y, it->z);
				PCxyzrgbout.push_back(*it);
			}else if(((hsv.h >= HLRED) && (hsv.h <= HURED)) && ((hsv.s >= ((float)SLRED/360)) && (hsv.s <= ((float)SURED/360)))){
				//ROS_INFO("Filtra Rojo");
				addNode(RED, it->x, it->y, it->z);
				PCxyzrgbout.push_back(*it);				
			}else if(((hsv.h >= HLBIGORANGE) && (hsv.h <= BIGORANGE))){
				//ROS_INFO("Filtra Naranja claro");
				addNode(BIGORANGE, it->x, it->y, it->z);
				PCxyzrgbout.push_back(*it);
			}else if(((hsv.h >= HLBLUE) && (hsv.h <= HUBLUE))){
				//ROS_INFO("Filtra Azul");
				addNode(BLUE, it->x, it->y, it->z);
				PCxyzrgbout.push_back(*it);
			}else if(((hsv.h >= HLYELLOW) && (hsv.h <= HUYELLOW))){
				//	ROS_INFO("Filtra amarillo");
				addNode(YELLOW, it->x, it->y, it->z);
				PCxyzrgbout.push_back(*it);
			}else if(((hsv.h >= HLPINK) && (hsv.h <= HUPINK))){
				//ROS_INFO("Filtra rosa");
				addNode(PINK, it->x, it->y, it->z);
				PCxyzrgbout.push_back(*it);
			}else{
				//ROS_INFO("No filtrado***********************************************************************");
				it->r = 0;
				it->g = 0;
				it->b = 0;
			}
		}
		//Filtrado de objetos
		//for(int i = 0; i < NUM_COLORS; i++){
		//	filtrarObjetos(i);
		//}

		//Percepcion de Objetos
		
			//Pendiente

		//tansform PCxyzrgb (pcl::PointCloud<pcl::PointXYZRGB>) to Image to display in the OpenCV window
		pcl::toROSMsg(PCxyzrgb, out);
		sensor_msgs::Image image;
		cv_bridge::CvImagePtr cv_imageout;

		pcl::toROSMsg(out, image);
		cv_imageout = cv_bridge::toCvCopy(image,
				sensor_msgs::image_encodings::BGR8);

		for (int i = 0; i < NUM_COLORS; i++){
			NodeColor *auxnode = objetos[i].list;
			while(auxnode != NULL){
				ROS_INFO("CX: %f  CY: %f  CZ: %f SIZE: %d", auxnode->cx, auxnode->cy, auxnode->cz, auxnode->total);
				cv::rectangle(cv_imageout->image, cv::Point(auxnode->cx-10, auxnode->cy-10), cv::Point(auxnode->cx+10, auxnode->cy+10), cv::Scalar(0, 0, 255), 1, 8);
				auxnode = auxnode->next;
			}
		}

		cv::imshow("Imagen filtrada", cv_imageout->image);
		cv::waitKey(3);

		//std::cout << "Center of the point cloud filtered = (" << x << ", " << y
		//		<< ", " << z << ") [" << c << "]" << std::endl;

		//tranform PCxyzrgbout (pcl::PointCloud<pcl::PointXYZRGB>) to PointCloud2 in order to be published
		sensor_msgs::PointCloud2 pcout;
		pcl::toROSMsg(PCxyzrgbout, pcout);
		image_pub_.publish(pcout);
		freeList();

	}

	void initObjetos(){
		for(int i = 0; i < NUM_COLORS; i++){
			objetos[i].list = NULL;
		}
	}

	NodeColor* newNodeColor(float x, float y, float z){
		NodeColor *node = NULL;
		if((node = new NodeColor) != NULL){
			node->cx = x;
			node->cy = y;
			node->cz = z;
			node->total = 1;
			node->next = NULL;
			node->prev = NULL;
		}
		return node;
	} 
	
	void addNode(int color, float x, float y, float z){
		if((objetos[color].list == NULL)){
			objetos[color].list = newNodeColor(x, y, z);
		}else{

			NodeColor *node = objetos[color].list;
			while(node != NULL){
				if(compPixel(node)){
					float xaux, yaux, zaux;
					xaux = yaux = zaux = 0.0;
					xaux = (node->cx * (float)node->total) + x;
					yaux = (node->cy * (float)node->total) + y;
					zaux = (node->cz * (float)node->total) + z;
					node->total++;
					node->cx = xaux / (float)node->total;
					node->cy = yaux / (float)node->total;
					node->cz = zaux / (float)node->total;
					break;
				}
				node = node->next;
			}
			if(node == NULL){
				node = newNodeColor(x, y, z);
				objetos[color].list->prev = node;
				node->next = objetos[color].list;
				objetos[color].list = node;
			}
		}
	}

	int compPixel(NodeColor *node){
		if(true){
			return 1;
		}
		return 0;
	}

	NodeColor* removeNode(NodeColor* node){
		NodeColor *aux = node->next;
		delete node;
		return aux;
	}


	void freeList(){
		NodeColor *node = NULL;
		for(int i=0; i<NUM_COLORS; i++){
			node = objetos[i].list;
			while(node != NULL){
				node = removeNode(node);
			}
			objetos[i].list = node;
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter3D");
	ImageConverter3D ic;
	ros::spin();
	return 0;
}

