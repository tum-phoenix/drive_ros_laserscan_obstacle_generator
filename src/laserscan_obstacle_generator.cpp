#include <laserscan_obstacle_generator/laserscan_obstacle_generator.h>
#include <drive_ros_msgs/Obstacle.h>
#include <drive_ros_msgs/ObstacleArray.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

LaserscanObstacleGenerator::LaserscanObstacleGenerator(const ros::NodeHandle& pnh):
  nh_(),
  pnh_(pnh),
  obstacle_width_(0.8),
  obstacle_init_trust_(0.3),
  minBlobElements_(5),
  maxBlobElements_(600),
  blobMaxDistance_(0.1)
{
  obstacle_pub_ = nh_.advertise<drive_ros_msgs::ObstacleArray>("obstacles_out",1);
}

LaserscanObstacleGenerator::~LaserscanObstacleGenerator() {
  return;
}

bool LaserscanObstacleGenerator::init() {
  scan_sub_ = nh_.subscribe("pointcloud_in", 10, &LaserscanObstacleGenerator::laserscanCallback, this);

  pnh_.getParam("obstacleInitTrust", obstacle_init_trust_);
  pnh_.getParam("obstacleWidth", obstacle_width_);
  pnh_.getParam("minBlobElements", minBlobElements_);
  pnh_.getParam("maxBlobElements", maxBlobElements_);
  pnh_.getParam("blobMaxDistance", blobMaxDistance_);
  return true;
}

void LaserscanObstacleGenerator::laserscanCallback(const sensor_msgs::PointCloud2ConstPtr& scan_in) {

  sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*scan_in,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*input_cloud);
  /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  /* Here we are creating a vector of PointIndices, which contains the actual index
  * information in a vector<int>. The indices of each detected cluster are saved here.
  * Cluster_indices is a vector containing one instance of PointIndices for each detected
  * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
  */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (blobMaxDistance_);
  ec.setMinClusterSize (minBlobElements_);
  ec.setMaxClusterSize (maxBlobElements_);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);

  /* To separate each cluster out of the vector<PointIndices> we have to
  * iterate through cluster_indices, create a new PointCloud for each
  * entry and write all points of the current cluster in the PointCloud.
  */
  drive_ros_msgs::Obstacle temp_obstacle;
  drive_ros_msgs::ObstacleArray obstacle_out;
  temp_obstacle.header = scan_in->header;

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  geometry_msgs::Point32 point;
  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
      // messages doesnt have a constructor apparently
      point.x = input_cloud->points[*pit].x;
      point.y = input_cloud->points[*pit].y;
      point.z = input_cloud->points[*pit].z;
      temp_obstacle.polygon.points.push_back(point);
    }
    temp_obstacle.trust = obstacle_init_trust_;
    // todo: fuse into trajectory instead of assuming blocking width
    temp_obstacle.width = obstacle_width_;
    temp_obstacle.obstacle_type = drive_ros_msgs::Obstacle::TYPE_LIDAR;
    obstacle_out.obstacles.push_back(temp_obstacle);
  }

  ROS_DEBUG_STREAM("Found "<<obstacle_out.obstacles.size()<<" obstacles in laser scanner pointcloud");
  obstacle_pub_.publish(obstacle_out);
}
