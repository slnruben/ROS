#include "imagetest3D.h"

	Imagetest3D::Imagetest3D() {
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

		maxrange = 0.1;
		minrange = -0.1;

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
		pcl::fromROSMsg(pcl_bf, PCxyzrgb);

		PCxyzrgbout = PCxyzrgb;
		PCxyzrgbout.clear();
		initColores();
		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

		int i = 0;
		for (it = PCxyzrgb.begin(); it != PCxyzrgb.end(); ++it) {
			pcl::PointXYZHSV hsv;
			pcl::PointXYZRGBtoXYZHSV(*it, hsv);
			
			if(it->x == it->x){
				if (((hsv.h >= HLORANGE) && (hsv.h <= HUORANGE)) && ((hsv.s >= ((float)SLORANGE/360)) && (hsv.s <= ((float)SUORANGE/360)))){
					PCxyzrgbout.push_back(*it);
					addNode(ORANGE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLRED) && (hsv.h <= HURED)) && ((hsv.s >= ((float)SLRED/360)) && (hsv.s <= ((float)SURED/360)))){
					PCxyzrgbout.push_back(*it);
					addNode(RED, it->x, it->y, it->z);				
				}else if(((hsv.h >= HLBIGORANGE) && (hsv.h <= BIGORANGE))){
					PCxyzrgbout.push_back(*it);
					addNode(BIGORANGE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLBLUE) && (hsv.h <= HUBLUE))){
					PCxyzrgbout.push_back(*it);
					addNode(BLUE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLYELLOW) && (hsv.h <= HUYELLOW))){
					PCxyzrgbout.push_back(*it);
					addNode(YELLOW, it->x, it->y, it->z);
				}else if(((hsv.h >= HLPINK) && (hsv.h <= HUPINK))){
					PCxyzrgbout.push_back(*it);
					addNode(PINK, it->x, it->y, it->z);
				}else{
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
		filtrarObjetos();
		reconnaissance();
		publishObjects();
		
		cv::waitKey(3);
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
					node = node->next;			
				}
			}
		}
	}

	Imagetest3D::NodeColor* Imagetest3D::removeNode(NodeColor* node, int color) {
		NodeColor *aux = NULL;

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
			char buffer[1];
			sprintf(buffer, "%d", i);
			std::string name(buffer);
			array[i].name = name;
			array[i].boolean = 0;
		}
	}

	void Imagetest3D::addArray(NodeColor *node, int i) {
		array[i].cx = node->cx;
		array[i].cy = node->cy;
		array[i].cz = node->cz;
		array[i].boolean = 1;
	}

	void Imagetest3D::reconnaissance() {
		initObjetos();
		searchBaliza(PINK, BLUE, 0);
		searchBaliza(YELLOW, PINK, 1);
		searchBaliza(YELLOW, BLUE, 2);
		searchBaliza(BLUE, PINK, 3);
		searchOther(objetos[YELLOW].list, 4);
		searchOther(objetos[BLUE].list, 5);
		searchOther(objetos[ORANGE].list, 6);
		searchOther(objetos[BIGORANGE].list, 7);
		searchOther(objetos[RED].list, 8);
	}

	void Imagetest3D::searchBaliza(int colortop, int colorbot, int i) {
		NodeColor *nodeTop = objetos[colortop].list;
		NodeColor *nodeBot = objetos[colorbot].list;
		int found = 0;
		float errormarginX;
		float errormarginY;
		if(nodeTop == NULL || nodeBot == NULL)
			return;

		while(nodeTop != NULL) {
			while(nodeBot != NULL) {

				errormarginX = nodeTop->cx - nodeBot->cx;
				errormarginY = nodeTop->cy - nodeBot->cy;
				if(errormarginX <= maxrange &&  errormarginX >= minrange &&
					errormarginY <= maxrange && errormarginY >= minrange && nodeTop->cz > nodeBot->cz) {
					found = 1;

					addArray(nodeTop, i);

					break;
				}else{

					nodeBot = nodeBot->next;
				}

			}
			if(found) {

				removeNode(nodeTop, colortop);
				removeNode(nodeBot, colorbot);

				break;
			}else{
				nodeTop = nodeTop->next;

				nodeBot = objetos[colorbot].list;
			}
		}
	}

	void Imagetest3D::searchOther(NodeColor *node, int pos) {
		int max;
		NodeColor *aux = NULL;
		max = 0;
		while(node != NULL){
			if(node->total > max) {
				aux = node;
				max = node->total;
			}
			node = node->next;
		}
		if(aux != NULL)
			addArray(aux, pos);
		
	}

	void Imagetest3D::publishObjects() {
		for(int i = 0; i < NUM_OBJECTS; i++) {
			if(array[i].boolean) {
				tf::StampedTransform RB;

				RB.frame_id_ = "/base_footprint";
				RB.child_frame_id_ = array[i].name;
				RB.stamp_ = ros::Time::now() + ros::Duration(0.5);

				RB.setOrigin(tf::Vector3(array[i].cx, array[i].cy, array[i].cz));
				RB.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
					
				try{
					tfB.sendTransform(RB);
				}catch(tf::TransformException & ex){
					ROS_WARN("%s",ex.what());
				}
			}
		}
	}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter3D");
	Imagetest3D ic;
	ros::spin();
	return 0;
}

