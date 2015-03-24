class ImageConverter3D {
	ros::NodeHandle nh_;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;

	int hupper, hlower;
	int supper, slower;
	int vupper, vlower;

	float vmh, vms, vmv;
	float c;

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
		cvCreateTrackbar("Hue Upper", "Imagen filtrada", &hupper, 360, NULL);
		cvCreateTrackbar("Hue Lower", "Imagen filtrada", &hlower, 360, NULL);
		cvCreateTrackbar("Sat Upper", "Imagen filtrada", &supper, 360, NULL);
		cvCreateTrackbar("Sat Lower", "Imagen filtrada", &slower, 360, NULL);
		cvCreateTrackbar("Val Upper", "Imagen filtrada", &vupper, 360, NULL);
		cvCreateTrackbar("Val Lower", "Imagen filtrada", &vlower, 360, NULL);
		//cvCreateButton("Save",ImageConverter::callbackButton,NULL,CV_PUSH_BUTTON,0);
		//Pelota roja clara pequeña
		//hupper = 360;
		//hlower = 319;
		//vupper = 360;
		//vlower = 300;
		//supper = 360;
		//slower = 313;
		//Pelota roja oscura pequeña
		//hupper = 350;
		//hlower = 330;
		//vupper = 359;
		//vlower = 220;
		//supper = 260;
		//slower = 120;
		//Pelota naranja grande
		//hupper = 360;
		//hlower = 340;
		//vupper = 180;
		//vlower = 120;
		//supper = 360;
		//slower = 350;
		//Baliza azul, rosa, blanca
		//hupper = 310;
		//hlower = 210;
		//vupper = 320;
		//vlower = 50;
		//supper = 360;
		//slower = 180;

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

		vmh = 0.0;
		vms = 0.0;
		vmv = 0.0;
		c = 0.0;

		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		for (it = PCxyzrgb.begin(); it != PCxyzrgb.end(); ++it) {
			pcl::PointXYZHSV hsv;
			pcl::PointXYZRGBtoXYZHSV(*it, hsv);

			//if the point is in the H range, it is added to the resulting point cloud. On the other hand, paint it black (to be displayed later)
			if ((hsv.h >= hlower) && (hsv.h <= hupper) && (hsv.s >= (float)slower/360) && (hsv.s <= (float)supper/360) && 
																(hsv.v >= (float)vlower/360) && (hsv.v <= (float)vupper/360)) { 

				PCxyzrgbout.push_back(*it);
				vmh += hsv.h;
				vms += hsv.s * 360.0;
				vmv += hsv.v * 360.0;
				c++;
			} else {
				it->r = 0;
				it->g = 0;
				it->b = 0;
			}
		}

		ROS_INFO("Valor h: %f", vmh/c);
		ROS_INFO("Valor s: %f", vms/c);
		ROS_INFO("Valor v: %f", vmv/c);

		//tansform PCxyzrgb (pcl::PointCloud<pcl::PointXYZRGB>) to Image to display in the OpenCV window
		pcl::toROSMsg(PCxyzrgb, out);
		sensor_msgs::Image image;
		cv_bridge::CvImagePtr cv_imageout;

		pcl::toROSMsg(out, image);
		cv_imageout = cv_bridge::toCvCopy(image,
				sensor_msgs::image_encodings::BGR8);

		cv::imshow("Imagen fuente", cv_imageout->image);
		cv::waitKey(3);

		//Get the mean position of the resulting point cloud

		float x, y, z;
		x = y = z = 0.0;
		int c = 0;

		for (it = PCxyzrgbout.begin(); it != PCxyzrgbout.end(); ++it) {

			if (it->x == it->x) //This seems to be the only way to detect if it is NaN
					{
				x = x + it->x;
				y = y + it->y;
				z = z + it->z;
				c++;
			}
		}

		if (c > 0) {
			x = x / (float) c;
			y = y / (float) c;
			z = z / (float) c;
		}
		std::cout << "Center of the point cloud filtered = (" << x << ", " << y
				<< ", " << z << ") [" << c << "]" << std::endl;

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
