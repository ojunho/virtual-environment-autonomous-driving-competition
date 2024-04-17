#include "header.h"
#include "dbscan.h"

// pcl point type
typedef pcl::PointXYZ PointT;
// cluster point type
typedef pcl::PointXYZI clusterPointT;

// ROI parameter
double xMinROI, xMaxROI, yMinROI, yMaxROI, zMinROI, zMaxROI;
double xMinBoundingBox, xMaxBoundingBox, yMinBoundingBox, yMaxBoundingBox, zMinBoundingBox, zMaxBoundingBox;
// DBScan parameter
int minPoints;
double epsilon, minClusterSize, maxClusterSize;
// VoxelGrid parameter
float leafSize;

// publisher
ros::Publisher pubROI;
ros::Publisher pubCluster;
ros::Publisher pubObstacleInfo;
ros::Publisher pubObstacleShapeArray;
ros::Publisher pubNearestObstacleInfo;

//MSG
obstacle_detection::ObstacleInfo obstacleInfoMsg;

laser_geometry::LaserProjection projector_;

void cfgCallback(obstacle_detection::hyper_parameter_dynamic_staticConfig &config, int32_t level) {
    xMinROI = config.xMinROI;
    xMaxROI = config.xMaxROI;
    yMinROI = config.yMinROI;
    yMaxROI = config.yMaxROI;
    zMinROI = config.zMinROI;
    zMaxROI = config.zMaxROI;

    minPoints = config.minPoints;
    epsilon = config.epsilon;
    minClusterSize = config.minClusterSize;
    maxClusterSize = config.maxClusterSize;

    xMinBoundingBox = config.xMinBoundingBox;
    xMaxBoundingBox = config.xMaxBoundingBox;
    yMinBoundingBox = config.yMinBoundingBox;
    yMaxBoundingBox = config.yMaxBoundingBox;
    zMinBoundingBox = config.zMinBoundingBox;
    zMaxBoundingBox = config.zMaxBoundingBox;
}

pcl::PointCloud<PointT>::Ptr ROI (const sensor_msgs::PointCloud2 input) {
    // ... do data processing
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(input, *cloud); // sensor_msgs -> PointCloud 형변환

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr *retPtr = &cloud_filtered;

    // 오브젝트 생성 
    // Z축 ROI
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud);                //입력 
    filter.setFilterFieldName("z");             //적용할 좌표 축 (eg. Z축)
    filter.setFilterLimits(zMinROI, zMaxROI);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // X축 ROI
    // pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("x");             //적용할 좌표 축 (eg. X축)
    filter.setFilterLimits(xMinROI, xMaxROI);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // Y축 ROI
    // pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("y");             //적용할 좌표 축 (eg. Y축)
    filter.setFilterLimits(yMinROI, yMaxROI);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // 포인트수 출력

    pcl::PCLPointCloud2 roiPclMsg;
    pcl::toPCLPointCloud2(*cloud_filtered, roiPclMsg);
    
    sensor_msgs::PointCloud2 roiRosMsg;
    pcl_conversions::fromPCL(roiPclMsg, roiRosMsg);
    roiRosMsg.header.frame_id = "lidar";
    
    pubROI.publish(roiRosMsg);

    return cloud_filtered;
}

void cluster(pcl::PointCloud<PointT>::Ptr input) {
    //KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    
    if (input->size() > 0) {
        tree->setInputCloud(input);
    }

    //Segmentation
    std::vector<pcl::PointIndices> cluster_indices;

    //DBSCAN with Kdtree for accelerating
    DBSCANKdtreeCluster<PointT> dc;
    dc.setCorePointMinPts(minPoints);   //Set minimum number of neighbor points
    dc.setClusterTolerance(epsilon); //Set Epsilon 
    dc.setMinClusterSize(minClusterSize);
    dc.setMaxClusterSize(maxClusterSize);
    dc.setSearchMethod(tree);
    dc.setInputCloud(input);
    dc.extract(cluster_indices);

    pcl::PointCloud<clusterPointT> totalcloud_clustered;
    int cluster_id = 0;

    //각 Cluster 접근
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) {
        pcl::PointCloud<clusterPointT> eachcloud_clustered;
        float cluster_counts = cluster_indices.size();

        //각 Cluster내 각 Point 접근
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

            clusterPointT tmp;
            tmp.x = input->points[*pit].x; 
            tmp.y = input->points[*pit].y;
            tmp.z = 0;
            tmp.intensity = cluster_id % 100; // 상수 : 예상 가능한 cluster 총 개수
            eachcloud_clustered.push_back(tmp);
            totalcloud_clustered.push_back(tmp);
        }

        //minPoint와 maxPoint 받아오기
        clusterPointT minPoint, maxPoint;
        pcl::getMinMax3D(eachcloud_clustered, minPoint, maxPoint);

        obstacleInfoMsg.lengthX[cluster_id] = maxPoint.x - minPoint.x; // 
        obstacleInfoMsg.lengthY[cluster_id] = maxPoint.y - minPoint.y; // 
        obstacleInfoMsg.lengthZ[cluster_id] = maxPoint.z - minPoint.z; // 
        obstacleInfoMsg.centerX[cluster_id] = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
        obstacleInfoMsg.centerY[cluster_id] = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
        obstacleInfoMsg.centerZ[cluster_id] = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표
        
    }
    obstacleInfoMsg.obstacleCounts = cluster_id;



    pubObstacleInfo.publish(obstacleInfoMsg);

    sensor_msgs::PointCloud2 cluster_point;
    pcl::toROSMsg(totalcloud_clustered, cluster_point);
    cluster_point.header.frame_id = "lidar";
    pubCluster.publish(cluster_point);
}

void visualizeObstacle() {
    visualization_msgs::MarkerArray obstacleShapeArray;
    visualization_msgs::Marker obstacleShape;
    uint32_t shape = visualization_msgs::Marker::CYLINDER; // Set our initial shape type to be a cube

    obstacleShape.header.frame_id = "lidar"; 
    obstacleShape.ns = "obstacle_shape";
    obstacleShape.type = shape;
    // Set the marker action.  Options are ADD and DELETE
    obstacleShape.action = visualization_msgs::Marker::ADD;
    for (int i = 0; i < obstacleInfoMsg.obstacleCounts; i++) {
        if (xMinBoundingBox <= obstacleInfoMsg.lengthX[i] && obstacleInfoMsg.lengthX[i] <= xMaxBoundingBox &&
            yMinBoundingBox <= obstacleInfoMsg.lengthY[i] && obstacleInfoMsg.lengthY[i] <= yMaxBoundingBox &&
            zMinBoundingBox <= obstacleInfoMsg.lengthZ[i] && obstacleInfoMsg.lengthZ[i] <= zMaxBoundingBox) {

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            obstacleShape.header.stamp = ros::Time::now();
            obstacleShape.id = 100+i; // 

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            obstacleShape.pose.position.x = obstacleInfoMsg.centerX[i];
            obstacleShape.pose.position.y = obstacleInfoMsg.centerY[i];
            obstacleShape.pose.position.z = obstacleInfoMsg.centerZ[i];

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            // obstacleShape.scale.x = obstacleInfoMsg.lengthX[i];
            // obstacleShape.scale.y = obstacleInfoMsg.lengthY[i];
            // obstacleShape.scale.z = obstacleInfoMsg.lengthZ[i];

            obstacleShape.scale.x = 0.3;
            obstacleShape.scale.y = 0.3;
            obstacleShape.scale.z = 0.05;
            
            // Set the color -- be sure to set alpha to something non-zero!
            obstacleShape.color.r = 1.0;
            obstacleShape.color.g = 0.0;
            obstacleShape.color.b = 0.0;
            obstacleShape.color.a = 0.8;

            obstacleShape.lifetime = ros::Duration(0.1);
            obstacleShapeArray.markers.emplace_back(obstacleShape);
        }
    }

    // Publish the marker
    pubObstacleShapeArray.publish(obstacleShapeArray);

    // Look for nearest obstacle
    std::vector< std::vector<float> > obstacleVector;
    obstacle_detection::NearestObstacleInfo NearestObstaclePosition;

    if (obstacleShapeArray.markers.size() > 0) {
        for (int i = 0; i < obstacleShapeArray.markers.size(); i++) {
            std::vector<float> obstacle;
            obstacle.emplace_back(obstacleShapeArray.markers[i].pose.position.x);
            obstacle.emplace_back(obstacleShapeArray.markers[i].pose.position.y);
            obstacleVector.emplace_back(obstacle);
        }

        sort(obstacleVector.begin(), obstacleVector.end());

        NearestObstaclePosition.x = obstacleVector[0][0];
        NearestObstaclePosition.y = obstacleVector[0][1];
    }
    else {
        NearestObstaclePosition.x = 0;
        NearestObstaclePosition.y = 0;
    }

    pubNearestObstacleInfo.publish(NearestObstaclePosition);
}

void mainCallback(const sensor_msgs::PointCloud2 input) {
    pcl::PointCloud<PointT>::Ptr cloudPtr;

    // main process method
    cloudPtr = ROI(input);
    cluster(cloudPtr);

    // visualize method
    visualizeObstacle();
}

void laser2cloudmsg (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    sensor_msgs::PointCloud2 pc2_dst;
    projector_.projectLaser(*scan_in, pc2_dst);

    mainCallback(pc2_dst);
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "obstacle_detection_dynamic_static");
    ros::NodeHandle nh;
    
    dynamic_reconfigure::Server<obstacle_detection::hyper_parameter_dynamic_staticConfig> server;
    dynamic_reconfigure::Server<obstacle_detection::hyper_parameter_dynamic_staticConfig>::CallbackType f;

    f = boost::bind(&cfgCallback, _1, _2);
    server.setCallback(f);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/scan", 1, laser2cloudmsg);

    // Create a ROS publisher for the output point cloud
    pubROI = nh.advertise<sensor_msgs::PointCloud2> ("/roi_raw_dynamic_static", 1);
    pubCluster = nh.advertise<sensor_msgs::PointCloud2>("/cluste_dynamic_staticr", 1);
    pubObstacleInfo = nh.advertise<obstacle_detection::ObstacleInfo>("/obstacle_info_dynamic_static", 1);
    pubObstacleShapeArray = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_marker_dynamic_static", 1);
    pubNearestObstacleInfo = nh.advertise<obstacle_detection::NearestObstacleInfo>("/nearest_obstacle_info_dynamic_static", 1);

    // Spin
    ros::spin();
}