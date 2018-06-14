#ifndef LASERSCAN_OBSTACLE_GENERATOR_H
#define LASERSCAN_OBSTACLE_GENERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class LaserscanObstacleGenerator {
public:
    LaserscanObstacleGenerator(const ros::NodeHandle& pnh);
    ~LaserscanObstacleGenerator();
    bool init();
private:
    void laserscanCallback(const sensor_msgs::LaserScanConstPtr& scan_in);
    ros::Subscriber scan_sub_;
    ros::Publisher obstacle_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    float minWidth_;
    float maxWidth_;
    float minLength_;
    float maxLength_;
    float obstacle_init_trust_;
    int minBlobElements_;
    int maxBlobElements_;
    float blobMaxDistance_;

    laser_geometry::LaserProjection projector_;
};

#endif // LASERSCAN_OBSTACLE_GENERATOR_H
