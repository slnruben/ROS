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

class Imagetest3D {
private:
	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;

	int hupper, hlower;
	int supper, slower;
	int vupper, vlower;

	float vmh, vms, vmv;
	float c;
	typedef struct ColorNode ColorNode;
	struct ColorNode {
		float cx;
		float cy;
		float cz;
		PointCloud2 p;
		int n;
		ColorNode next;
	};
	enum {NARANJAG, NARANJAP, ROJA, ROSA, AZUL, AMARILLO}
}