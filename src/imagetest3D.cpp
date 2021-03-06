#include "imagetest3D.h"

	Imagetest3D::Imagetest3D() {
		image_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
				//"/camera/depth/points", 1,
				"/camera/depth_registered/points", 1,
				&Imagetest3D::Imagetest3D::imageCb, this);
		image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_filtered", 1);
		HUORANGE = 360; //ImageConverter3D::HURANGE
		HLORANGE = 270; //336;
		SUORANGE = 255; //360;
		SLORANGE = 254; //276;
		VUORANGE = 255;
		VLORANGE = 254;
	
		HUBIGORANGE = 34; //35;
		HLBIGORANGE = 0; //0;
		SUBIGORANGE = 248; 
		SLBIGORANGE = 161; 
		VUBIGORANGE = 255;
		VLBIGORANGE = 132;
	
		HURED = 318; //360;
		HLRED = 260; //336;
		SURED = 255; //275;
		SLRED = 189; //180;
		VURED = 255;
		VLRED = 32;
	
		HUBLUE = 221; //265;
		HLBLUE = 206; //150;
		SUBLUE = 255; 
		SLBLUE = 295; 
		VUBLUE = 255;
		VLBLUE = 120;
	
		HUYELLOW = 74; //149;
		HLYELLOW = 33; //36;
		SUYELLOW  = 255; 
		SLYELLOW  = 139; 
		VUYELLOW  = 255;
		VLYELLOW  = 80;
		
		HUPINK = 340; //335;
		HLPINK = 272; //266;
		SUPINK = 255; 
		SLPINK = 108; 
		VUPINK = 255;
		VLPINK = 55;

		distpix = 10;
		sizemin = 20;

		maxrange = 0.1;
		minrange = -0.1;

		cvNamedWindow("Imagen filtrada");
		cvNamedWindow("Filtrador Pelotas");
		cvNamedWindow("Filtrador Balizas");

		cvCreateTrackbar("Hue Upper ORANGE", "Filtrador Pelotas", &HUORANGE, 360, NULL);
		cvCreateTrackbar("Hue Lower ORANGE", "Filtrador Pelotas", &HLORANGE, 360, NULL);
		cvCreateTrackbar("Sat Upper ORANGE", "Filtrador Pelotas", &SUORANGE, 255, NULL);
		cvCreateTrackbar("Sat Lower ORANGE", "Filtrador Pelotas", &SLORANGE, 255, NULL);
		cvCreateTrackbar("V Upper ORANGE", "Filtrador Pelotas", &VUORANGE, 255, NULL);
		cvCreateTrackbar("V Lower ORANGE", "Filtrador Pelotas", &VLORANGE, 255, NULL);

		cvCreateTrackbar("Hue Upper RED", "Filtrador Pelotas", &HURED, 360, NULL);
		cvCreateTrackbar("Hue Lower RED", "Filtrador Pelotas", &HLRED, 360, NULL);
		cvCreateTrackbar("Sat Upper RED", "Filtrador Pelotas", &SURED, 255, NULL);
		cvCreateTrackbar("Sat Lower RED", "Filtrador Pelotas", &SLRED, 255, NULL);
		cvCreateTrackbar("V Upper RED", "Filtrador Pelotas", &VURED, 255, NULL);
		cvCreateTrackbar("V Lower RED", "Filtrador Pelotas", &VLRED, 255, NULL);

		cvCreateTrackbar("Hue Upper BIGORANGE", "Filtrador Pelotas", &HUBIGORANGE, 360, NULL);
		cvCreateTrackbar("Hue Lower BIGORANGE", "Filtrador Pelotas", &HLBIGORANGE, 360, NULL);
		cvCreateTrackbar("Sat Upper BIGORANGE", "Filtrador Pelotas", &SUBIGORANGE, 255, NULL);
		cvCreateTrackbar("Sat Lower BIGORANGE", "Filtrador Pelotas", &SLBIGORANGE, 255, NULL);
		cvCreateTrackbar("V Upper BIGORANGE", "Filtrador Pelotas", &VUBIGORANGE, 255, NULL);
		cvCreateTrackbar("V Lower BIGORANGE", "Filtrador Pelotas", &VLBIGORANGE, 255, NULL);


		cvCreateTrackbar("Hue Upper BLUE", "Filtrador Balizas", &HUBLUE, 360, NULL);
		cvCreateTrackbar("Hue Lower BLUE", "Filtrador Balizas", &HLBLUE, 360, NULL);
		cvCreateTrackbar("Sat Upper BLUE", "Filtrador Balizas", &SUBLUE, 255, NULL);
		cvCreateTrackbar("Sat Lower BLUE", "Filtrador Balizas", &SLBLUE, 255, NULL);
		cvCreateTrackbar("V Upper BLUE", "Filtrador Balizas", &VUBLUE, 255, NULL);
		cvCreateTrackbar("V Lower BLUE", "Filtrador Balizas", &VLBLUE, 255, NULL);

		cvCreateTrackbar("Hue Upper PINK", "Filtrador Balizas", &HUPINK, 360, NULL);
		cvCreateTrackbar("Hue Lower PINK", "Filtrador Balizas", &HLPINK, 360, NULL);
		cvCreateTrackbar("Sat Upper PINK", "Filtrador Balizas", &SUPINK, 255, NULL);
		cvCreateTrackbar("Sat Lower PINK", "Filtrador Balizas", &SLPINK, 255, NULL);
		cvCreateTrackbar("V Upper PINK", "Filtrador Balizas", &VUPINK, 255, NULL);
		cvCreateTrackbar("V Lower PINK", "Filtrador Balizas", &VLPINK, 255, NULL);

		cvCreateTrackbar("Hue Upper YELLOW", "Filtrador Balizas", &HUYELLOW, 360, NULL);
		cvCreateTrackbar("Hue Lower YELLOW", "Filtrador Balizas", &HLYELLOW, 360, NULL);
		cvCreateTrackbar("Sat Upper YELLOW", "Filtrador Balizas", &SUYELLOW, 255, NULL);
		cvCreateTrackbar("Sat Lower YELLOW", "Filtrador Balizas", &SLYELLOW, 255, NULL);
		cvCreateTrackbar("V Upper YELLOW", "Filtrador Balizas", &VUYELLOW, 255, NULL);
		cvCreateTrackbar("V Lower YELLOW", "Filtrador Balizas", &VLYELLOW, 255, NULL);

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
		try {
 			pcl_ros::transformPointCloud("/base_link", *msg, pcl_bf, tf_listener);	
		} catch (tf::TransformException & ex) {
		ROS_WARN("%s", ex.what());
	}
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
				if (((hsv.h >= HLORANGE) && (hsv.h <= HUORANGE)) && ((hsv.s >= ((float)SLORANGE/255)) && (hsv.s <= ((float)SUORANGE/255))) 
										&& ((hsv.v >= ((float)VLORANGE/255)) && (hsv.v <= ((float)VUORANGE/255)))){
					PCxyzrgbout.push_back(*it);
					addNode(ORANGE, it->x, it->y, it->z);				
				}else if(((hsv.h >= HLBIGORANGE) && (hsv.h <= HUBIGORANGE)) && ((hsv.s >= ((float)SLBIGORANGE/255)) && (hsv.s <= ((float)SUBIGORANGE/255))) 
										&& ((hsv.v >= ((float)VLBIGORANGE/255)) && (hsv.v <= ((float)VUBIGORANGE/255)))){
					PCxyzrgbout.push_back(*it);
					addNode(BIGORANGE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLRED) && (hsv.h <= HURED)) && ((hsv.s >= ((float)SLRED/255)) && (hsv.s <= ((float)SURED/255)))
										&& ((hsv.v >= ((float)VLRED/255)) && (hsv.v <= ((float)VURED/255)))){
					PCxyzrgbout.push_back(*it);
					addNode(RED, it->x, it->y, it->z);
				}else if(((hsv.h >= HLBLUE) && (hsv.h <= HUBLUE)) && ((hsv.s >= ((float)SLBLUE/255)) && (hsv.s <= ((float)SUBLUE/255)))
										&& ((hsv.v >= ((float)VLBLUE/255)) && (hsv.v <= ((float)VUBLUE/255)))){
					PCxyzrgbout.push_back(*it);
					addNode(BLUE, it->x, it->y, it->z);
				}else if(((hsv.h >= HLYELLOW) && (hsv.h <= HUYELLOW)) && ((hsv.s >= ((float)SLYELLOW/255)) && (hsv.s <= ((float)SUYELLOW/255)))
										&& ((hsv.v >= ((float)VLYELLOW/255)) && (hsv.v <= ((float)VUYELLOW/255)))){
					PCxyzrgbout.push_back(*it);
					addNode(YELLOW, it->x, it->y, it->z);
				}else if(((hsv.h >= HLPINK) && (hsv.h <= HUPINK)) && ((hsv.s >= ((float)SLPINK/255)) && (hsv.s <= ((float)SUPINK/255)))
										&& ((hsv.v >= ((float)VLPINK/255)) && (hsv.v <= ((float)VUPINK/255)))){
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
		
		//pcl::toROSMsg(PCxyzrgb, out);
		//sensor_msgs::Image image;
		//cv_bridge::CvImagePtr cv_imageout;
	
		//pcl::toROSMsg(out, image);
		//cv_imageout = cv_bridge::toCvCopy(image,
		//	sensor_msgs::image_encodings::BGR8);
			//sensor_msgs::image_encodings::RGB16);

		//cv::imshow("Imagen filtrada", cv_imageout->image);
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
ROS_INFO(" ");
		for(int i = 0; i <= 5; i++) {
			char buffer[1];
			sprintf(buffer, "%d", i);
			std::string name(buffer);
			array[i].name = "perceived_" + name;
			array[i].boolean = 0;
		}

		for(int i = 6; i <= 8; i++) {
			char buffer[1];
			sprintf(buffer, "%d", i);
			std::string name(buffer);
			array[i].name = "pelota_" + name;
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

				RB.frame_id_ = "/base_link";
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

