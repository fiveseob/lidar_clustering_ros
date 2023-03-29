#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

// #define PI 3.14159265

using namespace std;


// // Ouster OS1-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1024;
extern const float ang_res_x = 360.0 / float(Horizon_SCAN);
extern const float ang_res_y = 45.0 / float(N_SCAN);
extern const float ang_bottom = 22.5 + 0.1;
extern const int groundScanInd = 35;

extern const float groundThreshold = 30.0;
extern const float sensorMinimumRange = 1.0;
extern const float sensorMountAngle = 0.0;

extern const float segmentTheta = 5.0 / 180.0 * M_PI; // decrease this value may improve accuracy
extern const int segmentValidPointNum = 10;
extern const int segmentValidLineNum = 2;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

extern const float intensityThreshold = 200;
// extern const int minimumPointsNum = 5;
extern const int minimumPointsNum = 2;
extern const int maximumPointsNum = 1000;

// clustering
extern const float clusterTolerance = 1.0;
extern const int minSize = 2;
extern const int maxSize = 1000;

// Adaptive cropping
extern int MAX_X = -1.3;
extern int MIN_X = -50.0;
extern int MAX_Y = 1.5;
extern int MIN_Y = -1.5;
