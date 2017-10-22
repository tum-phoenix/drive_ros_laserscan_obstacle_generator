#ifndef LASERSCAN_OBSTACLE_GENERATOR_H
#define LASERSCAN_OBSTACLE_GENERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class LaserscanObstacleGenerator {
public:
    LaserscanObstacleGenerator(const ros::NodeHandle& pnh);
    ~LaserscanObstacleGenerator();
    bool init();
private:
    void laserscanCallback(const sensor_msgs::PointCloud2ConstPtr& scan_in);
    ros::Subscriber scan_sub_;
    ros::Publisher obstacle_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    float obstacle_width_;
    float obstacle_init_trust_;
    int minBlobElements_;
    int maxBlobElements_;
    float blobMaxDistance_;
};

#endif // LASERSCAN_OBSTACLE_GENERATOR_H
