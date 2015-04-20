#ifndef imagetest3D_H
#define imagetest3D_H

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
#include <string>

class Imagetest3D {
private:
	enum{ORANGE, BIGORANGE, RED, BLUE, YELLOW, PINK, NUM_COLORS};

	//typedef struct NodeColor NodeColor;
	//typedef struct ListObjetos ListObjetos;
	struct NodeColor{
		float cx;
		float cy;
		float cz;
		float total;
		//pcl::PointCloud<pcl::PointXYZRGB> cloud;
		NodeColor *next;
		NodeColor *prev;
	};

	struct ListColores{
		NodeColor *list;
	};

	struct Objects{
		std::string name;
		float cx;
		float cy;
		float cz;
		int boolean;
	};

	enum{BALIZA1, BALIZA2, BALIZA3, BALIZA4, PORTERIA1, PORTERIA2, PELOTA1, PELOTA2, PELOTA3, NUM_OBJECTS};

	Objects array [NUM_OBJECTS];

	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;
	
	ListColores objetos[NUM_COLORS];

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

	int distpix;
	int sizemin;

	float minrange;
	float maxrange;
public:
	Imagetest3D();
	~Imagetest3D();
	void imageCb(const sensor_msgs::PointCloud2ConstPtr& msg);
	void initColores();
	NodeColor* newNodeColor(float x, float y, float z);
	void addNode(int color, float x, float y, float z);
	int compPixel(NodeColor *node, float x, float y, float z);
	void filtrarObjetos();
	NodeColor* removeNode(NodeColor* node);
	void freeList();
	void initObjetos();
	void addArray(NodeColor *node, int i);
	void reconnaissance();
	void searchBaliza(NodeColor *top, NodeColor *bot, int i);
	void searchPorteria(NodeColor *node, int pos);
};

#endif