#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "pd_controller/prob.h"
#include "pd_controller/ray.h"

#include <chrono>
#include <ctime> 
#include <mutex>
#include <future>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>



class bresenham2D{
    public:
    ros::Subscriber costmap_subscriber, footprint , amcl_pose;
    ros::NodeHandle nh_;
    int height;
    double *yaw;
    cv::Mat dest;
    int costmap_count = 0;
    int *costmap_count_ = &costmap_count;
    ros::Publisher pb;
    bool value = false;
    bool *costmap_ = &(value); 
    bool *semaphore = &(value);
    std::mutex m;
    bool footprint_ = false;
    boost::recursive_mutex planner_mutex_;
    int width;
    double PI  =3.141592653589793238463;
    double prev = 0;
    
    float data[120][120];
    float filter[40][40];
    int8_t costmap_data[64000];

    double resolution_ = 0.025;
    double *resolution = &(resolution_);

    int sem;
    int *sem_ = &sem;

    double* length;

    double originX_;
    double *originX = &originX_;
    
    double originY_;
    double *originY = &originY_;

    int* offset;

    double curr_pose_x;
    double curr_pose_y;
    bool kill =false;
    bool* kill_p = &kill;
    costmap_2d::Costmap2DROS *controller_costmap_ros_;

    
    bresenham2D(double length);
    int compute(int *x,double goalX,double goalY);
    void print_corresponding_coordinates(int x);
    cv::Mat* convolve();
    void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);
    void find_line(double x1, double y1, double x2, double y2);
    void put_point(double x ,double y);
    void refresh_data();
    void refresh_data_filter();
    //void put_points(double x, double y);
   
    bool dot_product(cv::Mat* output);
    //void points_inside_ellipse(double x , double y);
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap );
    bool check_robot_path(double goalX, double goalY);
    bool distance(int x1, int y1 , int x0 , int y0)
    {

        float distance = std::sqrt(std::pow(x1 - x0,2) + std::pow(y1 - y0,2));

        if (distance > 20)
        {
            return false;
        }

        return true;

    }
};