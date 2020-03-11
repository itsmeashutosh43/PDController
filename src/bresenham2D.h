#include <ros/ros.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>

#include <chrono>
#include <ctime> 
#include <mutex>
#include <future>
#include <thread>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "pd_controller/prob.h"
#include "pd_controller/ray.h"





class bresenham2D{
    public:
    ros::Subscriber costmap_subscriber, footprint , amcl_pose;
    ros::NodeHandle nh_;
    ros::Publisher pb;

    int height;
    int width;
    int costmap_count = 0;
    int *costmap_count_ = &costmap_count;
    int sem;
    int *sem_ = &sem;
    
    double PI  =3.141592653589793238463;
    double prev = 0;
    double *yaw;
    double resolution_ = 0.025;
    double *resolution = &(resolution_);
    double* length;
    double originX_;
    double *originX = &originX_;
    double originY_;
    double *originY = &originY_;
    double curr_pose_x;
    double curr_pose_y;

    float data[80][80];
    float filter[40][40];
    float costmap_data[6400];

    bool value = false;
    bool *costmap_ = &(value); 
    
    cv::Mat dest;
    std::mutex m;
        
    bresenham2D(double length);

    void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap );

    bool check_robot_path(double goalX, double goalY);
    void refresh_data();
    void refresh_data_filter();
    int compute(int *x,double goalX,double goalY);
    bool convolve();
    void find_line(double x1, double y1, double x2, double y2);
    void put_point(double x ,double y);
    bool distance(int x1, int y1 , int x0 , int y0);
    
};