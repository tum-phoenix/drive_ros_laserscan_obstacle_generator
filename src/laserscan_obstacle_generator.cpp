#include <drive_ros_laserscan_obstacle_generator/laserscan_obstacle_generator.h>
#include <drive_ros_msgs/Obstacle.h>
#include <drive_ros_msgs/ObstacleArray.h>

#include <limits>
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
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>

LaserscanObstacleGenerator::LaserscanObstacleGenerator(const ros::NodeHandle& pnh):
  nh_(),
  pnh_(pnh),
  minWidth_(0.0),
  maxWidth_(1.0),
  minLength_(0.0),
  maxLength_(1.0),
  obstacle_init_trust_(0.3),
  minBlobElements_(5),
  maxBlobElements_(600),
  blobMaxDistance_(0.1),
  projector_()
{
  obstacle_pub_ = nh_.advertise<drive_ros_msgs::ObstacleArray>("obstacles_out",1);
}

LaserscanObstacleGenerator::~LaserscanObstacleGenerator() {
  return;
}

bool LaserscanObstacleGenerator::init() {
  scan_sub_ = nh_.subscribe("scan_in", 10, &LaserscanObstacleGenerator::laserscanCallback, this);

  std::vector<float> pose_cov_vec, twist_cov_vec;

  bool ret = true;
  ret &= pnh_.getParam("obstacleInitTrust", obstacle_init_trust_);
  ret &= pnh_.getParam("minWidth",          minWidth_);
  ret &= pnh_.getParam("maxWidth",          maxWidth_);
  ret &= pnh_.getParam("minLength",         minLength_);
  ret &= pnh_.getParam("maxLength",         maxLength_);
  ret &= pnh_.getParam("minBlobElements",   minBlobElements_);
  ret &= pnh_.getParam("maxBlobElements",   maxBlobElements_);
  ret &= pnh_.getParam("blobMaxDistance",   blobMaxDistance_);
  ret &= pnh_.getParam("pose_covariance",   pose_cov_vec);
  ret &= pnh_.getParam("twist_covariance",  twist_cov_vec);

  std::copy(pose_cov_vec.begin(), pose_cov_vec.begin() + 36, pose_covariance.begin());
  std::copy(twist_cov_vec.begin(), twist_cov_vec.begin() + 36, twist_covariance.begin());

  ROS_ASSERT(36 == pose_cov_vec.size());
  ROS_ASSERT(36 == twist_cov_vec.size());

  return ret;
}

void LaserscanObstacleGenerator::laserscanCallback(const sensor_msgs::LaserScanConstPtr& scan_in) {

  sensor_msgs::PointCloud2 incoming_pointcloud;
  projector_.projectLaser(*scan_in, incoming_pointcloud);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(incoming_pointcloud, pcl_pc2);
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
  ec.setClusterTolerance(blobMaxDistance_);
  ec.setMinClusterSize(minBlobElements_);
  ec.setMaxClusterSize(maxBlobElements_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract(cluster_indices);

  drive_ros_msgs::ObstacleArray obstacle_out;
  /* To separate each cluster out of the vector<PointIndices> we have to
  * iterate through cluster_indices, create a new PointCloud for each
  * entry and write all points of the current cluster in the PointCloud.
  */
  for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

    drive_ros_msgs::Obstacle temp_obstacle;
    pcl::CentroidPoint<pcl::PointXYZ> centroid;

    float max_x=FLT_MIN, max_y=FLT_MIN, max_z=FLT_MIN,
          min_x=FLT_MAX, min_y=FLT_MAX, min_z=FLT_MAX;

    for(auto pit = it->indices.begin(); pit != it->indices.end(); pit++) {
      // messages doesnt have a constructor apparently
      geometry_msgs::Point32 point;
      point.x = input_cloud->points[*pit].x;
      point.y = input_cloud->points[*pit].y;
      point.z = input_cloud->points[*pit].z;
      temp_obstacle.polygon.points.push_back(point);

      // get min/max values
      max_x = std::max(max_x, input_cloud->points[*pit].x);
      max_y = std::max(max_y, input_cloud->points[*pit].y);
      max_z = std::max(max_z, input_cloud->points[*pit].z);

      min_x = std::min(min_x, input_cloud->points[*pit].x);
      min_y = std::min(min_y, input_cloud->points[*pit].y);
      min_z = std::min(min_z, input_cloud->points[*pit].z);

      // add point to centroid
      centroid.add(input_cloud->points[*pit]);
    }

    // currently no rotation is supported :(
    geometry_msgs::Quaternion q;
    q.x=0; q.y=0; q.z=0; q.w=0;
    temp_obstacle.centroid_pose.pose.orientation = q;

    temp_obstacle.length = std::max(max_x - min_x, float(0.0));
    temp_obstacle.width =  std::max(max_y - min_y, float(0.0));
    temp_obstacle.height = std::max(max_z - min_z, float(0.0));

    if(temp_obstacle.length > maxLength_ ||
       temp_obstacle.length < minLength_ ||
       temp_obstacle.width  > maxWidth_  ||
       temp_obstacle.width  < minWidth_  )
    {
        ROS_DEBUG("Rejecting Object.");
        continue;
    }


    // get centroid point
    geometry_msgs::Point gm_centroid_pt;
    pcl::PointXYZ pcl_centroid_pt;
    centroid.get(pcl_centroid_pt);
    gm_centroid_pt.x = pcl_centroid_pt.x;
    gm_centroid_pt.y = pcl_centroid_pt.y;
    gm_centroid_pt.z = pcl_centroid_pt.z;
    temp_obstacle.centroid_pose.pose.position = gm_centroid_pt;

    temp_obstacle.header.stamp = scan_in->header.stamp;
    temp_obstacle.header.frame_id = scan_in->header.frame_id;

    // set trust and type
    temp_obstacle.trust = obstacle_init_trust_;
    temp_obstacle.obstacle_type = drive_ros_msgs::Obstacle::TYPE_LIDAR;

    // set covarinaces
    temp_obstacle.centroid_pose.covariance = pose_covariance;
    temp_obstacle.centroid_twist.covariance = twist_covariance;

    obstacle_out.obstacles.push_back(temp_obstacle);


  }

  ROS_DEBUG_STREAM("Found "<<obstacle_out.obstacles.size()<<" obstacles in laser scanner pointcloud");
  obstacle_pub_.publish(obstacle_out);
}
