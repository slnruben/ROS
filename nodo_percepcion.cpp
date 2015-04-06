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

enum{ORANGE, RED, BLUE, YELLOW, PINK, NUM_COLORS};

enum{
	HUORANGE = 255,
	HLORANGE = 0,
	SUORANGE = 255,
	SLORANGE = 0,
	VUORANGE = 255,
	VLORANGE = 0,

	HURED = 255,
	HLRED = 0,
	SURED = 255,
	SLRED = 0,
	VURED = 255,
	VLRED = 0,

	HUBLUE = 255,
	HLBLUE = 0,
	SUBLUE = 255,
	SLBLUE = 0,
	VUBLUE = 255,
	VLBLUE = 0,

	HUYELLOW = 255,
	HLYELLOW = 0,
	SUYELLOW = 255,
	SLYELLOW = 0,
	VUYELLOW = 255,
	VLYELLOW = 0,

	HUPINK = 255,
	HLPINK = 0,
	SUPINK = 255,
	SLPINK = 0,
	VUPINK = 255,
	VLPINK = 0,
};

typedef struct NodeColor NodeColor;
struct NodeColor{
	float cx;
	float cy;
	float cz;
	int total;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	NodeColor *next;
	NodeColor *prev;
};

class ImageConverter3D {
	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;
	
	NodeColor *objetos[NUM_COLORS];

public:
	ImageConverter3D() {
		//std::string topic = nh_.resolveName("point_cloud");
		image_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
				"/camera/depth_registered/points", 1,
				&ImageConverter3D::imageCb, this);
		image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_filtered", 1);

		cvNamedWindow("Imagen filtrada");
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

		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		for (it = PCxyzrgb.begin(); it != PCxyzrgb.end(); ++it) {
			pcl::PointXYZHSV hsv;
			pcl::PointXYZRGBtoXYZHSV(*it, hsv);

			//if the point is in the H range, it is added to the resulting point cloud. On the other hand, paint it black (to be displayed later)
			if (((hsv.h >= HLORANGE) && (hsv.h <= HUORANGE)) && ((hsv.s >= SLORANGE) && (hsv.s <= SUORANGE)) && ((hsv.v >= VLORANGE) && (hsv.v <= VUORANGE))){
				PCxyzrgbout.push_back(*it);
				addNode(ORANGE, it, PCxyzrgb);
			}else if(((hsv.h >= HLRED) && (hsv.h <= HURED)) && ((hsv.s >= SLRED) && (hsv.s <= SURED)) && ((hsv.v >= VLRED) && (hsv.v <= VURED))){
				PCxyzrgbout.push_back(*it);
				addNode(RED, it, PCxyzrgb);
			}else if(((hsv.h >= HLBLUE) && (hsv.h <= HUBLUE)) && ((hsv.s >= SLBLUE) && (hsv.s <= SUBLUE)) && ((hsv.v >= VLBLUE) && (hsv.v <= VUBLUE))){
				PCxyzrgbout.push_back(*it);
				addNode(BLUE, it, PCxyzrgb);
			}else if(((hsv.h >= HLYELLOW) && (hsv.h <= HUYELLOW)) && ((hsv.s >= SLYELLOW) && (hsv.s <= SUYELLOW)) && ((hsv.v >= VLYELLOW) && (hsv.v <= VUYELLOW))){
				PCxyzrgbout.push_back(*it);
				addNode(YELLOW, it, PCxyzrgb);
			}else if(((hsv.h >= HLPINK) && (hsv.h <= HUPINK)) && ((hsv.s >= SLPINK) && (hsv.s <= SUPINK)) && ((hsv.v >= VLPINK) && (hsv.v <= VUPINK))){
				PCxyzrgbout.push_back(*it);
				addNode(PINK, it, PCxyzrgb);
			}else {
				it->r = 0;
				it->g = 0;
				it->b = 0;
			}
		}

		//Filtrado de objetos
		for(int i = 0; i < NUM_COLORS; i++){
			filtrarObjetos(i);
		}

		//Percepcion de Objetos
		
			//Pendiente

		//tansform PCxyzrgb (pcl::PointCloud<pcl::PointXYZRGB>) to Image to display in the OpenCV window
		pcl::toROSMsg(PCxyzrgb, out);
		sensor_msgs::Image image;
		cv_bridge::CvImagePtr cv_imageout;

		pcl::toROSMsg(out, image);
		cv_imageout = cv_bridge::toCvCopy(image,
				sensor_msgs::image_encodings::BGR8);

		cv::imshow("Imagen filtrada", cv_imageout->image);
		cv::waitKey(3);

		//std::cout << "Center of the point cloud filtered = (" << x << ", " << y
		//		<< ", " << z << ") [" << c << "]" << std::endl;

		//tranform PCxyzrgbout (pcl::PointCloud<pcl::PointXYZRGB>) to PointCloud2 in order to be published
		sensor_msgs::PointCloud2 pcout;
		pcl::toROSMsg(PCxyzrgbout, pcout);
		image_pub_.publish(pcout);
		freeObjetos();

	}

	float calcDistanciaEuclidea(NodeColor *node, float px, float py, float pz){
		float x = (px - node->cx);
		float y = (py - node->cy);
		float z = (pz - node->cz);
		return sqrt(x*x + y*y + z*z);
	}

	//Crea un nodo nuevo
	NodeColor* newNode(pcl::PointCloud<pcl::PointXYZRGB>::iterator it, pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb){
		NodeColor *auxnode = NULL;
		if((auxnode = (NodeColor*)malloc(sizeof(NodeColor*))) != NULL){
				auxnode->cx = it->x;
				auxnode->cy = it->y;
				auxnode->cz = it->z;
				auxnode->cloud = PCxyzrgb;
				auxnode->cloud.clear();
				auxnode->cloud.push_back(*it);
				auxnode->total = 1;
				auxnode->next = NULL;	
		}
		return auxnode;
	}
	
	//Añade un nodo nuevo si el pixel no pertenece a ningun nodo de los que se encuentren en la lista
	void addNode(int color, pcl::PointCloud<pcl::PointXYZRGB>::iterator it, pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb){
		NodeColor *auxnode = NULL;
		NodeColor *auxnode2 = NULL;
		float distancia = 0.0;
		float x, y, z;
		x = y = z = 0.0;

		if( objetos[color] = NULL){
			objetos[color] = newNode(it, PCxyzrgb);				 
		}else{
			auxnode = objetos[color];
			for(;;){
				distancia = calcDistanciaEuclidea(auxnode, it->x, it->y, it->z);
				if(distancia <= 1000.0){
					x = (auxnode->cx * (float)auxnode->total) + it->x;
					y = (auxnode->cy * (float)auxnode->total) + it->y;
					z = (auxnode->cz * (float)auxnode->total) + it->z;
					auxnode->total++;
					auxnode->cx = x / (float)auxnode->total;
					auxnode->cy = y / (float)auxnode->total;
					auxnode->cz = z / (float)auxnode->total;
					auxnode->cloud.push_back(*it);
					break;					
				}
				if(auxnode->next == NULL){
					auxnode->next = newNode(it, PCxyzrgb);
					auxnode2 = auxnode->next;
					auxnode2->prev = auxnode;
					break;	
				}
			}
		}
	}

	//Elimina el nodo de la lista
	NodeColor* removeNode(int color, NodeColor *node){
		NodeColor *auxnode = NULL;
		if(node->prev == NULL){
			objetos[color] = node->next;
			free(node);
			objetos[color]->prev = NULL;
			return objetos[color];
		}
		
		auxnode = node->prev;
		auxnode->next = node->next;
		auxnode = node->next;
		if(auxnode != NULL)
			auxnode->prev = node->prev;
		free(node);
		return auxnode;
			
	}

	//Filtrado los objetos, para eliminar los objetos que no cumplen los requisitos
	void filtrarObjetos(int color){
		NodeColor *node = objetos[color];
		while(node != NULL){
			if(node->total < 22){ 				//Ajustar valor
				node = removeNode(color, node);
			}else if(node->cy < 1.0){			//Ajustar valor
				node = removeNode(color, node);
			}else if(node->cz < 2.0){			//Ajustar valor
				node = removeNode(color, node);
			}else{
				node = node->next;
			}	
		}	
	}

	void freeObjetos(){
		NodeColor *node;
		for(int i = 0; i < NUM_COLORS; i++){
			node = objetos[i];
			while(node != NULL){
				node = removeNode(i, node);
			}
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter3D");
	ImageConverter3D ic;
	ros::spin();
	return 0;
}




























