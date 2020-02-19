#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



class bresenham2D{
    ros::Subscriber costmap_subscriber, footprint , amcl_pose;
    ros::NodeHandle nh_;
    int height;
    bool costmap_ = false;
    bool footprint_ = false;
    int width;
    double resolution;
    int8_t data[64000];
    int8_t costmap_data[64000];
    double originX;
    double originY;
    double goalX = 15;
    double goalY = 2;
    double curr_pose_x;
    double curr_pose_y;
    bool kill =false;

    public:
    bresenham2D();
    void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);
    void find_line(double x1, double y1, double x2, double y2);
    void put_point(double x ,double y);
    void refresh_data();
    void footprintCallback(geometry_msgs::PolygonStampedConstPtr footprint);
    bool dot_product();
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap );
    bool check_robot_path(double goalX, double goalY);
};