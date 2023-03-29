#include "utility.h"
#include "list"


class Process{

private:
    ros::NodeHandle nh;
    ros::NodeHandle ns;
    ros::Subscriber subLaserCloud;
    ros::Subscriber subCropLidar;
    ros::Subscriber sub3DPoint;


    ros::Publisher pubGroundCloud;
    ros::Publisher pubFrontGroundCloud;

    // ros::Publisher pubTunnelFlag;
    ros::Publisher pubGroundRemovedCloud;
    // ros::Publisher pubInCameraFOVCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubPostPointCloud;
    ros::Publisher pub_polygon;
    ros::Publisher pub_postPoint;
    ros::Publisher pub_polygon_test;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud; // projected ouster raw cloud, but saved in the form of 1-D matrix

    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr FrontgroundCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr groundRemovedCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inCameraFOVCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentedCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr verticalCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr postPointCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr FromCamPoint;


    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> objectClusters;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> postPointClusters;
    jsk_recognition_msgs::PolygonArray polygon_array;
    jsk_recognition_msgs::PolygonArray pt_3d_array;
    geometry_msgs::PolygonStamped polygon_t;



    visualization_msgs::MarkerArray postPoint_array;
	
    std_msgs::Float32MultiArray tunnel_inf;

    pcl::PointXYZI nanPoint; // fill in fullCloud at each iteration

    cv::Mat labelMat; // ground matrix for ground cloud marking
    cv::Mat rangeMat; // range matrix for range image
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

    std::vector<int> px_offset;
    
public:
    Process():
        nh("~"){

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/os1_cloud_node/points", 1, &Process::cloudHandler, this);
        // subCropLidar = nh.subscribe<std_msgs::Float32MultiArray>("/lidar/crop_lidar", 1, &Process::cropHandler, this);
        sub3DPoint = nh.subscribe<std_msgs::Float32MultiArray>("/detection_front/bbox_3d", 1, &Process::pt3dHandler, this);
        
	    // pubTunnelFlag = nh.advertise<std_msgs::Float32MultiArray>("/lidar/tunnel_flag", 1);
        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/ground_cloud", 1);
        pubFrontGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/front_ground_cloud", 1);
        pubGroundRemovedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/ground_removed_cloud", 1);
        // pubInCameraFOVCloud = nh.advertise<sensor_msgs::PointCloud2> ("/in_camera_fov_cloud", 1);
        // pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/segmented_cloud", 1);
        // pubPostPointCloud = nh.advertise<sensor_msgs::PointCloud2> ("/post_point_cloud", 1);
        // pub_polygon = nh.advertise<jsk_recognition_msgs::PolygonArray>("/lidar/polygon", 1);
        pub_polygon_test = nh.advertise<geometry_msgs::PolygonStamped>("/lidar/polygon_test", 1);
        pub_postPoint = nh.advertise<visualization_msgs::MarkerArray>("/lidar/postpoint", 1);
        // nt.z = st    // nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        // nt.z = st.y = std::numeric_limits<float>::quiet_NaN();
        // nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        px_offset = get_px_offset(1024);

        allocateMemory();
        resetParameters();
    }

    ~Process(){}

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());

        fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        groundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        FrontgroundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        groundRemovedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        inCameraFOVCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        segmentedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        verticalCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        postPointCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        FromCamPoint.reset(new pcl::PointCloud<pcl::PointXYZI>());


        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        FrontgroundCloud->clear();

        groundRemovedCloud->clear();
        inCameraFOVCloud->clear();
        segmentedCloud->clear();
        verticalCloud->clear();
        postPointCloud->clear();

        FromCamPoint->clear();


        objectClusters.clear();
	    tunnel_inf.data.clear();        
	    postPointClusters.clear();
        polygon_array.polygons.clear();
        pt_3d_array.polygons.clear();
        postPoint_array.markers.clear();

        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        clock_t start = clock();

        copyPointCloud(laserCloudMsg);
        projectPointCloud();
        groundRemoval();
        //extractVertical();
        //verticalSegmentation();
        //filteringPostpoint();
        objectSegmentation();
        filteringObject();
        cal_point();
        publishResult();
        resetParameters();

        // printf("%f s\n", (float)(clock() - start) / CLOCKS_PER_SEC);
    }

    void cropHandler(std_msgs::Float32MultiArray msg){
        MAX_X = msg.data[0];
        MIN_X = msg.data[1];
        MAX_Y = msg.data[2];
        MIN_Y = msg.data[3];
    }
// pt_3d_array
    void pt3dHandler(std_msgs::Float32MultiArray msg){
        vector<cv::Point2f> points_3d;
        geometry_msgs::PolygonStamped polygon_3d_array;
        polygon_3d_array.header.frame_id = "os1_lidar";

        for (unsigned int i = 0; i < msg.data.size(); i+=2){
            geometry_msgs::Point32 point;
            point.x = msg.data[i];
            point.y = msg.data[i+1];
            polygon_3d_array.polygon.points.push_back(point);
        }

        geometry_msgs::PolygonStamped polygon_3d = polygon_3d_array;
        pt_3d_array.polygons.push_back(polygon_3d);
    }

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        cloudHeader = laserCloudMsg->header;
        //cloudHeader.stamp = ros::Time::now();

        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    }

    std::vector<int> get_px_offset(int lidar_mode) {
        auto repeat = [](int n, const std::vector<int>& v) {
            std::vector<int> res{};
            for (int i = 0; i < n; i++) res.insert(res.end(), v.begin(), v.end());
            return res;
        };

        switch (lidar_mode) {
            case 512:
                return repeat(16, {0, 3, 6, 9});
            case 1024:
                return repeat(16, {0, 6, 12, 18});
            case 2048:
                return repeat(16, {0, 12, 24, 36});
            default:
                return std::vector<int>{64, 0};
        }
    }

    void projectPointCloud(){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        pcl::PointXYZI thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (int u = 0; u < N_SCAN; u++) {
            for (int v = 0; v < Horizon_SCAN; v=v+2) {
                const size_t vv = (v + px_offset[u]) % Horizon_SCAN;     // u : 수직 채널, v : 수평 채널
                cout << vv << endl;
                const size_t i = vv * N_SCAN + u;
                cout << " i " << i << endl;

                const auto& pt = laserCloudIn->points[i];

                thisPoint.x = laserCloudIn->points[i].x;
                thisPoint.y = laserCloudIn->points[i].y;
                thisPoint.z = laserCloudIn->points[i].z;
                // find the row and column index in the iamge for this point

                rowIdn = (N_SCAN-1)-u;
                columnIdn = v;

                cout << "rowIdn" << rowIdn << endl;
                cout << "columnIdn" << columnIdn << endl;
                
                range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
                if (range < sensorMinimumRange)
                    continue;

                rangeMat.at<float>(rowIdn, columnIdn) = range;

                thisPoint.intensity = laserCloudIn->points[i].intensity;

                index = columnIdn  + rowIdn * Horizon_SCAN;

                fullCloud->points[index] = thisPoint;
            }
        }
    }

    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
	int tunnel_point = 0;
	float tunnel_distance = 0.0;
	int tunnel_flag = 0;
        
	for (size_t j = 0; j < Horizon_SCAN; ++j){   // H 1024channel
            for (size_t i = 0; i < N_SCAN; ++i){     //  64channel
                lowerInd = j + (i)*Horizon_SCAN;

                if (isnan(fullCloud->points[lowerInd].x) || isnan(fullCloud->points[lowerInd].y) || isnan(fullCloud->points[lowerInd].z) ||
                    isinf(fullCloud->points[lowerInd].x) || isinf(fullCloud->points[lowerInd].y) || isinf(fullCloud->points[lowerInd].z)){
                    labelMat.at<int8_t>(i,j) = -1; // nan
                    continue;
                }
		
		if (fullCloud->points[lowerInd].x < -10 && fullCloud->points[lowerInd].x > -15 && fullCloud->points[lowerInd].y < 2 && fullCloud->points[lowerInd].y > -2 && fullCloud->points[lowerInd].z > 2.3){
			tunnel_point++;
			if(fullCloud->points[lowerInd].x < tunnel_distance){
				tunnel_distance = fullCloud->points[lowerInd].x;
			}
		}
                if (i < groundScanInd){
                    upperInd = j + (i + 1)*Horizon_SCAN;

                    diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                    diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                    diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                    angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                    if ((abs(angle - sensorMountAngle) <= groundThreshold && fullCloud->points[lowerInd].z < -0.4) || (isnan(angle)&& fullCloud->points[lowerInd].z < -0.4)){
                        groundCloud->push_back(fullCloud->points[lowerInd]);
                        if (fullCloud->points[lowerInd].x < -3 && fullCloud->points[lowerInd].x > -50 && fullCloud->points[lowerInd].y > -3 && fullCloud->points[lowerInd].y < 3){
                            FrontgroundCloud->points.push_back(fullCloud->points[lowerInd]);
                        }
                        labelMat.at<int8_t>(i,j) = 1; // ground
                    }
                    else{
                        // compare with next next point
                        upperInd = j + (i+2)*Horizon_SCAN;
                        diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                        diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                        diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                        angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                        if((abs(angle - sensorMountAngle) <= groundThreshold && fullCloud->points[lowerInd].z < -0.4) || (isnan(angle)&& fullCloud->points[lowerInd].z < -0.4)){
                            groundCloud->push_back(fullCloud->points[lowerInd]);
                            if (fullCloud->points[lowerInd].x < -3 && fullCloud->points[lowerInd].x > -50 && fullCloud->points[lowerInd].y > -3 && fullCloud->points[lowerInd].y < 3){
                                FrontgroundCloud->points.push_back(fullCloud->points[lowerInd]);
                            }
                            labelMat.at<int8_t>(i,j) = 1; // ground
                        }
                        else{
                            labelMat.at<int8_t>(i,j) = 2; // not ground
                            if (fullCloud->points[lowerInd].z < 1.5 && fullCloud->points[lowerInd].x < MAX_X && fullCloud->points[lowerInd].x > MIN_X && fullCloud->points[lowerInd].y > MIN_Y && fullCloud->points[lowerInd].y < MAX_Y)
                                groundRemovedCloud->push_back(fullCloud->points[lowerInd]);
                            if (fullCloud->points[lowerInd].x < -4.0 && fullCloud->points[lowerInd].z < 3.0){
                                inCameraFOVCloud->points.push_back(fullCloud->points[lowerInd]);
                            }
                        }
                    }
                }
                else{
                    labelMat.at<int8_t>(i,j) = 2; // not ground
                    if (fullCloud->points[lowerInd].z < 1.5 && fullCloud->points[lowerInd].x < MAX_X && fullCloud->points[lowerInd].x > MIN_X && fullCloud->points[lowerInd].y > MIN_Y && fullCloud->points[lowerInd].y < MAX_Y)
                        groundRemovedCloud->push_back(fullCloud->points[lowerInd]);
                    if (fullCloud->points[lowerInd].x < -4.0 && fullCloud->points[lowerInd].z < 3.0){
                        inCameraFOVCloud->points.push_back(fullCloud->points[lowerInd]);
                    }
                }
            }
        }

	if(tunnel_point > 100){
	    tunnel_flag = 1;
	}
	tunnel_inf.data.push_back(tunnel_flag);
	tunnel_inf.data.push_back(-tunnel_distance);
    }

    // void extractVertical(){
    //     size_t lowerInd, upperInd;
    //     float dis;
    //     float diffX, diffY, diffZ, angle;

    //     for (size_t j = 0; j < Horizon_SCAN; ++j){
    //         int n = 0;
    //         int startInd = -1;
    //         for (size_t i = 0; i < N_SCAN-1; ++i){
    //             if (labelMat.at<int8_t>(i,j) == 2 && labelMat.at<int8_t>(i+1,j) == 2){
    //                 lowerInd = j + (i)*Horizon_SCAN;

    //                 if (fullCloud->points[lowerInd].z < -0.35)
    //                     continue;

    //                 upperInd = j + (i+1)*Horizon_SCAN;

    //                 diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
    //                 diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
    //                 diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

    //                 dis = sqrt(diffZ*diffZ + diffX*diffX + diffY*diffY);

    //                 angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

    //                 if (abs(angle) > 45 && dis < 0.2){
    //                     if (startInd == -1){
    //                         startInd = i;
    //                     }
    //                     else{
    //                         n++;
    //                     }
    //                 }
    //                 else{
    //                     if (startInd != -1 && n >= 10){
    //                         for (int k = 0; k <= n; k++){
    //                             verticalCloud->push_back(fullCloud->points[j + (startInd+k)*Horizon_SCAN]);
    //                         }
    //                     }
    //                     startInd = -1;
    //                     n = 0;
    //                 }
    //             }
    //         }
    //     }
    // }

    // void verticalSegmentation(){
    //     verticalCloud->is_dense = false;
    //     std::vector<int> indices;
    //     pcl::removeNaNFromPointCloud(*verticalCloud,*verticalCloud, indices);

    //     if (verticalCloud->points.size() > 0){
    //         pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    //         tree->setInputCloud(verticalCloud);
    //         std::vector<pcl::PointIndices> clusterIndices;
    //         pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    //         ec.setClusterTolerance(clusterTolerance);
    //         ec.setMinClusterSize(minSize);
    //         ec.setMaxClusterSize(maxSize);
    //         ec.setSearchMethod(tree);
    //         ec.setInputCloud(verticalCloud);
    //         ec.extract(clusterIndices);

    //         for(pcl::PointIndices getIndices: clusterIndices)
    //         {
    //             typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZI>);

    //             for(int index : getIndices.indices){
    //                 pcl::PointXYZI pt = verticalCloud->points[index];
    //                 cloudCluster->points.push_back(pt);
    //             }

    //             cloudCluster->width = cloudCluster->points.size();
    //             cloudCluster->height = 1;
    //             cloudCluster->is_dense = true;

    //             postPointClusters.push_back(cloudCluster);
                
    //         }
    //     }
    // }

    // void filteringPostpoint(){
    //     int j = 0;
    //     for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : postPointClusters){
    //         pcl::PointXYZI minPoint, maxPoint;
    //         pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    //         float x_size = maxPoint.x - minPoint.x;
    //         float y_size = maxPoint.y - minPoint.y;
    //         float z_size = maxPoint.z - minPoint.z;
            
    //         if (x_size < 0.5 && y_size < 0.5){
    //             for (auto pt : cluster->points){
    //                 pt.intensity = j;
    //                 postPointCloud->points.push_back(pt);
    //             }
    //             j += 1;
    //             float x = (maxPoint.x + minPoint.x) / 2.0;
    //             float y = (maxPoint.y + minPoint.y) / 2.0;
    //             if (sqrt(x*x + y*y) < 18.0){
    //                 1;
    //                 //visualization_msgs::Marker postPoint = get_post_point_msg(j, x, y);
    //                 //postPoint_array.markers.push_back(postPoint);
    //             }
    //         }
    //     }
    // }

    void objectSegmentation(){
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr downSampledCloud(new pcl::PointCloud<pcl::PointXYZI>);

        // vg.setInputCloud(groundRemovedCloud);
        // // vg.setLeafSize(0.4f, 0.4f, 0.01f); // 0.01f is 1cm
        // vg.setLeafSize(0.01f, 0.01f, 0.01f); // 0.01f is 1cm
        
        // vg.filter(*downSampledCloud);
        // groundRemovedCloud        
        
        if (groundRemovedCloud->points.size() > 0){
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(groundRemovedCloud);
            std::vector<pcl::PointIndices> clusterIndices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(clusterTolerance);
            ec.setMinClusterSize(minSize);
            ec.setMaxClusterSize(maxSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(groundRemovedCloud);
            ec.extract(clusterIndices);

            int j = 0;
            for(pcl::PointIndices getIndices: clusterIndices)
            {
                typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZI>);

                for(int index : getIndices.indices){
                    pcl::PointXYZI pt = groundRemovedCloud->points[index];
                    pt.intensity = (float)(j + 1);

                    // if(pt.y > -5.0 && pt.y < 5.0)
                    if(pt.y > -1.0 && pt.y < 1.0){
                        segmentedCloud->points.push_back(pt);
                        cloudCluster->points.push_back(pt);
                    }
                }
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                cloudCluster->is_dense = true;

                objectClusters.push_back(cloudCluster);
                j += 1;
            } 
        }
    }

    void filteringObject(){
        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : objectClusters){
            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cluster, minPoint, maxPoint);
            
            if(maxPoint.z < 0.5 && ( maxPoint.x-minPoint.x > 0.08 || maxPoint.y-minPoint.y>0.05) && maxPoint.x-minPoint.x<4 && maxPoint.y-minPoint.y<4 && (maxPoint.x-minPoint.x)*(maxPoint.y-minPoint.y)<20 && sqrt(pow(maxPoint.x,2)+pow(maxPoint.y,2))<35){
                int num = cluster->points.size();
                
                if (num >= minimumPointsNum && num <= maximumPointsNum){
                    geometry_msgs::PolygonStamped polygon = get_polygon_msg(cluster);
                    polygon_t = polygon;
                    polygon_array.polygons.push_back(polygon_t);
                   

                }
            }
        }
    }

    void cal_point(){
        // for(int i=0; i<FromCamPoint->points.size(); i++){
        //     cout << FromCamPoint->points[i] << endl;
        //     // cout << FromCamPoint->points[i].i << endl;
        // }
        
        for (int i = 0; i < polygon_array.polygons.size(); i++){
            float x_pt = -9999;
            float y_pt = 0;
            float z_pt = 9999;
            float Cam_X = 0;
            float Cam_Y = 0;
            // select min distance point

            for(int j = 0; j < polygon_array.polygons[i].polygon.points.size(); j++){
                // x_pt = x_pt + polygon_array.polygons[i].polygon.points[j].x;
                // y_pt = y_pt + polygon_array.polygons[i].polygon.points[j].y;
                // z_pt = z_pt + polygon_array.polygons[i].polygon.points[j].z;

                // if (x_pt < -1.3){
                if (x_pt < -4 ){
                // if (x_pt > 4.0){
                    x_pt  = max(x_pt, (polygon_array.polygons[i].polygon.points[j].x));
                
                }
                if (z_pt > -2.5){
                    z_pt = min(z_pt, polygon_array.polygons[i].polygon.points[j].z);
                }
                y_pt = y_pt + polygon_array.polygons[i].polygon.points[j].y;
                
                if(j == polygon_array.polygons[i].polygon.points.size()-1){
                    y_pt = y_pt/polygon_array.polygons[i].polygon.points.size();
                }
            }

            for (int k = 0; k < pt_3d_array.polygons.size(); k++){
                for(int l = 0; l < pt_3d_array.polygons[k].polygon.points.size(); l++){
                    Cam_X = pt_3d_array.polygons[k].polygon.points[l].x;
                    Cam_Y = pt_3d_array.polygons[k].polygon.points[l].y;

                    cout << x_pt << endl;
                    cout << Cam_X << endl;
                    cout <<"========="<<endl;
                    if (abs(abs(x_pt) - abs(Cam_X)) < 10 && abs(abs(y_pt) - abs(Cam_Y)) < 1 && abs(Cam_X) > abs(x_pt)) {
                        cout<< "[OOOOOO]X : " << abs(Cam_X) << ", " << abs(x_pt) <<" Y : "<< abs(Cam_Y) <<", "<< abs(y_pt) <<endl;
                        
                        visualization_msgs::Marker postPoint = get_post_point_msg(i, x_pt, y_pt, z_pt);
                        postPoint_array.markers.push_back(postPoint);
                    }
                    else{
                        cout<< "[XXXXXXX]X : " << abs(Cam_X) << ", " << abs(x_pt) <<" Y : "<< abs(Cam_Y) <<", "<< abs(y_pt) <<endl;
                        continue;
                    }
                }
            }
        }
    }

    void publishResult(){
        // Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        // ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "os1_lidar";
            pubGroundCloud.publish(laserCloudTemp);
        }

        if (pubFrontGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*FrontgroundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "os1_lidar";
            pubFrontGroundCloud.publish(laserCloudTemp);
        }

        // ground removed cloud
        if (pubGroundRemovedCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundRemovedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "os1_lidar";
            pubGroundRemovedCloud.publish(laserCloudTemp);
        }

        // obeject segmented cloud
        if (pubSegmentedCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "os1_lidar";
            // pubSegmentedCloud.publish(laserCloudTemp);
        }

        // postpoint segmented cloud
        if (pubPostPointCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*postPointCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "os1_lidar";
            // pubPostPointCloud.publish(laserCloudTemp);
        }
        
        // segmented cloud in front camera FOV
        pcl::toROSMsg(*inCameraFOVCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "os1_lidar";
        // pubInCameraFOVCloud.publish(laserCloudTemp);

        // polygon array
        // polygon_array.header.stamp = ros::Time::now(); //cloudHeader.stamp
        // polygon_array.header.frame_id = "os1_lidar";
        // pub_polygon.publish(polygon_array);

        // test
        polygon_t.header.stamp = ros::Time::now(); //cloudHeader.stamp
        polygon_t.header.frame_id = "os1_lidar";
        pub_polygon_test.publish(polygon_t);

        // NGV
        // pt_3d_array.header.stamp = ros::Time::now(); //cloudHeader.stamp
        // pt_3d_array.header.frame_id = "os1_lidar";
        // pub_polygon.publish(pt_3d_array);
        
        // post point array 
        pub_postPoint.publish(postPoint_array);

	// tunnel inf array
	// pubTunnelFlag.publish(tunnel_inf);
    }

    geometry_msgs::PolygonStamped get_polygon_msg(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
    {
      vector<cv::Point2f> points;
      geometry_msgs::PolygonStamped polygonStamped;
      polygonStamped.header.frame_id = "os1_lidar";

      float min_z = 100.0;
      float max_z = -100.0;

      for (unsigned int i = 0; i < cluster->points.size(); i++)
      {
        cv::Point2f pt;
        pt.x = cluster->points[i].x;
        pt.y = cluster->points[i].y;
        points.push_back(pt);

        if(min_z > cluster->points[i].z)
                min_z = cluster->points[i].z;
        if(max_z < cluster->points[i].z)
            max_z = cluster->points[i].z;
      }

      vector<cv::Point2f> hull;
      convexHull(points, hull);
      for (size_t i = 0; i < hull.size() + 1; i++)
      {
        geometry_msgs::Point32 point;
        point.x = hull[i % hull.size()].x ;
        point.y = hull[i % hull.size()].y ;
        point.z = min_z;
        polygonStamped.polygon.points.push_back(point);
      }

      for (size_t i = 0; i < hull.size() + 1; i++)
      {
        geometry_msgs::Point32 point;
        point.x = hull[i % hull.size()].x;
        point.y = hull[i % hull.size()].y;
        point.z = max_z;
        polygonStamped.polygon.points.push_back(point);
      }

      return polygonStamped;
    }

    visualization_msgs::Marker get_post_point_msg(int n, float x, float y, float z)
    {
        visualization_msgs::Marker marker;
        marker.type = marker.CYLINDER;
        marker.action = marker.ADD;
        marker.header.frame_id = "os1_lidar";
        marker.ns = "post_point_" + to_string(n);
        marker.id = int(n);
        marker.lifetime = ros::Duration(0.1);
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.position.x = x;
        marker .pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        return marker;
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "lidar");
    
    Process P;

    ros::spin();
    return 0;
}
