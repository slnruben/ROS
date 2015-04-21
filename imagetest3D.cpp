#include "imagetest3D.h"

	Imagetest3D::Imagetest3D() {
		//std::string topic = nh_.resolveName("point_cloud");
		image_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
				//"/camera/depth_registered/points", 1,
				"/camera/depth/points", 1,
				&Imagetest3D::Imagetest3D::imageCb, this);
		image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_filtered", 1);
		HUORANGE = 360; //ImageConverter3D::HURANGE
		HLORANGE = 360; //336;
		SUORANGE = 360; //360;
		SLORANGE = 360; //276;
	
		HUBIGORANGE = 360; //35;
		HLBIGORANGE = 360; //0;
	
		HURED = 360; //360;
		HLRED = 360; //336;
		SURED = 360; //275;
		SLRED = 360; //180;
	
		HUBLUE = 360; //265;
		HLBLUE = 360; //150;
	
		HUYELLOW = 360; //149;
		HLYELLOW = 360; //36;
		
		HUPINK = 360; //335;
		HLPINK = 360; //266;
		distpix = 10;
		sizemin = 20;

		maxrange = 1.0;
		minrange = -1.0;

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

		cvCreateTrackbar("Distacia pixeles", "Filtrador Balizas", &distpix, 100, NULL);
		cvCreateTrackbar("Tamaño minimo", "Filtrador Balizas", &sizemin, 100, NULL);

	}

	Imagetest3D::~Imagetest3D() {
		cv::destroyWindow("Imagen filtrada");
		cv::destroyWindow("Filtrador Balizas");
		cv::destroyWindow("Filtrador Pelotas");
	}

	void Imagetest3D::imageCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
		pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb, PCxyzrgbout;

				
		sensor_msgs::PointCloud2 out;
		sensor_msgs::PointCloud2 pcl_bf;
 		pcl_ros::transformPointCloud("/base_footprint", *msg, pcl_bf, tf_listener); 

		//PointCloud2 -> pcl::PointCloud<pcl::PointXYZRGB>
		//pcl::fromROSMsg(*msg, PCxyzrgb);
		pcl::fromROSMsg(pcl_bf, PCxyzrgb);

		//Copy original point cloud to the resulting one.
		//The points in the resulting point cloud are removed completely.
		//The points of this resulting point cloud will be added later, depending on the H filtering process.
		PCxyzrgbout = PCxyzrgb;
		PCxyzrgbout.clear();
		initColores();
		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		for (it = PCxyzrgb.begin(); it != PCxyzrgb.end(); ++it) {
			pcl::PointXYZHSV hsv;
			pcl::PointXYZRGBtoXYZHSV(*it, hsv);
			
			//if the point is in the H range, it is added to the resulting point cloud. On the other hand, paint it black (to be displayed later)
			if(it->x == it->x){
				if (((hsv.h >= HLORANGE) && (hsv.h <= HUORANGE)) && ((hsv.s >= ((float)SLORANGE/360)) && (hsv.s <= ((float)SUORANGE/360)))){
					//ROS_INFO("Filtra Naranja");
					PCxyzrgbout.push_back(*it);
					addNode(ORANGE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLRED) && (hsv.h <= HURED)) && ((hsv.s >= ((float)SLRED/360)) && (hsv.s <= ((float)SURED/360)))){
					//ROS_INFO("Filtra Rojo");
					PCxyzrgbout.push_back(*it);
					addNode(RED, it->x, it->y, it->z);				
				}else if(((hsv.h >= HLBIGORANGE) && (hsv.h <= BIGORANGE))){
					//ROS_INFO("Filtra Naranja claro");
					PCxyzrgbout.push_back(*it);
					addNode(BIGORANGE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLBLUE) && (hsv.h <= HUBLUE))){
					//ROS_INFO("Filtra Azul");
					PCxyzrgbout.push_back(*it);
					addNode(BLUE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLYELLOW) && (hsv.h <= HUYELLOW))){
					//	ROS_INFO("Filtra amarillo");
					PCxyzrgbout.push_back(*it);
					addNode(YELLOW, it->x, it->y, it->z);
				}else if(((hsv.h >= HLPINK) && (hsv.h <= HUPINK))){
					//ROS_INFO("Filtra rosa");
					PCxyzrgbout.push_back(*it);
					addNode(PINK, it->x, it->y, it->z);
				}else{
					//ROS_INFO("No filtrado***********************************************************************");
					it->r = 0;
					it->g = 0;
					it->b = 0;
				}
			}else{
				it->r = 0;
				it->g = 0;
				it->b = 0;
			}
		}
		//Filtrado de objetos
		//for(int i = 0; i < NUM_COLORS; i++){
		filtrarObjetos();

		//}

		//Percepcion de Objetos
		
			//Pendiente

		//tansform PCxyzrgb (pcl::PointCloud<pcl::PointXYZRGB>) to Image to display in the OpenCV window
		//pcl::toROSMsg(PCxyzrgb, out);
		//sensor_msgs::Image image;
		//cv_bridge::CvImagePtr cv_imageout;

		//pcl::toROSMsg(out, image);
		//cv_imageout = cv_bridge::toCvCopy(image,
		//		sensor_msgs::image_encodings::BGR8);
				//sensor_msgs::image_encodings::RGB8);
		//for (int i = 0; i < NUM_COLORS; i++){
		tf::TransformBroadcaster tfB;
			NodeColor *auxnode = objetos[3].list;
			while(auxnode != NULL){
				tf::StampedTransform RB;

				RB.frame_id_ = "/base_link";
				RB.child_frame_id_ = "/ball_1";
				RB.stamp_ = ros::Time::now() + ros::Duration(0.5);

				RB.setOrigin(tf::Vector3(auxnode->cx, auxnode->cy, auxnode->cz));
				RB.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
				auxnode = auxnode->next;
			
				try
				{
				ROS_INFO("PUBLICA");
				tfB.sendTransform(RB);

				}catch(tf::TransformException & ex){
					ROS_WARN("%s",ex.what());
				}
			}
		//}*/
		

		//cv::imshow("Imagen filtrada", cv_imageout->image);
		cv::waitKey(3);

		//std::cout << "Center of the point cloud filtered = (" << x << ", " << y
		//		<< ", " << z << ") [" << c << "]" << std::endl;

		//tranform PCxyzrgbout (pcl::PointCloud<pcl::PointXYZRGB>) to PointCloud2 in order to be published
		sensor_msgs::PointCloud2 pcout;
		pcl::toROSMsg(PCxyzrgbout, pcout);
		image_pub_.publish(pcout);
	
		freeList();
	}

	void Imagetest3D::initColores() {
		for(int i = 0; i < NUM_COLORS; i++){
			objetos[i].list = NULL;
		}
	}

	Imagetest3D::NodeColor* Imagetest3D::newNodeColor(float x, float y, float z) {
		NodeColor *node = NULL;
		if((node = new NodeColor) != NULL){
			node->cx = x;
			node->cy = y;
			node->cz = z;
			node->total = 1.0;
			node->next = NULL;
			node->prev = NULL;
		}
		return node;
	} 
	
	void Imagetest3D::addNode(int color, float x, float y, float z) {
		if((objetos[color].list == NULL)){
			objetos[color].list = newNodeColor(x, y, z);
		}else{

			NodeColor *node = objetos[color].list;
			while(node != NULL){
				if(compPixel(node, x, y, z)){
					float xaux, yaux, zaux;
					xaux = yaux = zaux = 0.0;
					xaux = (node->cx * node->total) + x;
					yaux = (node->cy * node->total) + y;
					zaux = (node->cz * node->total) + z;
					node->total = node->total + 1.0;
					node->cx = xaux / node->total;
					node->cy = yaux / node->total;
					node->cz = zaux / node->total;
					//ROS_INFO("x: %f - %f, Y: %f -%f, z: %f - %f, size: %f", x,node->cx, y,node->cy, z, node->cz, node->total);
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

	int Imagetest3D::compPixel(NodeColor *node, float x, float y, float z) {
		float px = (x - node->cx);
		float py = (y - node->cy);
		float pz = (z - node->cz);
		float total = sqrt(px * px + py * py + pz * pz);
		if(total < (float)distpix/100.0){
			return 1;
		}
		return 0;
	}

	void Imagetest3D::filtrarObjetos() {
		NodeColor *node = NULL;
		for(int i = 0; i < NUM_COLORS; i++){
			node = objetos[i].list;
			while(node != NULL){
				if(node->total < (float)sizemin){
					node = removeNode(node, i);
				}else{
					//ROS_INFO("X: %d,  Y: %d \n", node->cx, node->cy);
					node = node->next;			
				}
			}
		}
	}

	Imagetest3D::NodeColor* Imagetest3D::removeNode(NodeColor* node, int color) {
		NodeColor *aux = node->next;

		if(node == objetos[color].list)
			objetos[color].list = node->next;
		aux = node->prev;
		if(aux != NULL)
			aux->next = node->next;
		aux = node->next;
		if(aux != NULL)
			aux->prev = node->prev;
		
		delete node;
		return aux;
	}


	void Imagetest3D::freeList() {
		NodeColor *node = NULL;
		for(int i = 0; i < NUM_COLORS; i++){
			node = objetos[i].list;
			while(node != NULL){
				node = removeNode(node, i);
			}
			objetos[i].list = node;
		}
	}

	void Imagetest3D::initObjetos() {
		for(int i = 0; i <= NUM_OBJECTS; i++) {
			//array[i].name = std::to_string(i);
			array[i].boolean = 0;

				/*int a = 5;
	char t[255];

	sprintf(t, "%d", a);

	std::string ms(t);
*/
		}
	}

	void Imagetest3D::addArray(NodeColor *node, int i) {
		array[i].cx = node->cx;
		array[i].cy = node->cy;
		array[i].cz = node->cz;
		array[i].boolean = 1;
	}

	void Imagetest3D::reconnaissance() {
		searchBaliza(objetos[PINK].list, objetos[BLUE].list, 0);
		searchBaliza(objetos[YELLOW].list, objetos[PINK].list, 1);
		searchBaliza(objetos[YELLOW].list, objetos[BLUE].list, 2);
		searchBaliza(objetos[BLUE].list, objetos[PINK].list, 3);
		searchPorteria(objetos[YELLOW].list, 4);
		searchPorteria(objetos[BLUE].list, 5);
	}

	void Imagetest3D::searchBaliza(NodeColor *top, NodeColor *bot, int i) {
		NodeColor *nodeTop = top;
		NodeColor *nodeBot = bot;
		int found = 0;
		float errormarginY;
		float errormarginZ;

		while(nodeTop != NULL) {
			while(nodeBot != NULL) {
				errormarginY = nodeTop->cy - nodeBot->cy;
				errormarginZ = nodeTop->cz - nodeBot->cz;
				if(errormarginY <= maxrange &&  errormarginY >= minrange &&
					errormarginZ <= maxrange && errormarginZ >= minrange) {
					found = 1;
					addArray(nodeTop, i);
					break;
				}else{
					nodeBot = nodeBot->next;
				}
			}
			if(found) {
				removeNode(nodeTop, i);
				removeNode(nodeBot, i);
				break;
			}else{
				nodeTop = nodeTop->next;
			}
		}
	}

	void Imagetest3D::searchPorteria(NodeColor *node, int pos) {
		int max;
		for(int i = 1; i < 3; i++) {
		 	NodeColor *aux = NULL;
			max = 0;
			while(node != NULL){
				if(node->total > max) {
					aux = node;
					max = node->total;
				}
				node = node->next;
			}
			addArray(aux, pos);
		}
	}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter3D");
	Imagetest3D ic;
	ros::spin();
	return 0;
}

