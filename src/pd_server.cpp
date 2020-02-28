#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include <math.h>
#include <pd_controller/pdAction.h>
#include "pid_implementation.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include "bresenham2D.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "velocity_smoother.h"


/*

Actionlib server is implemented here.
This class/ node is comparable to move_base node.
*/

class ControllerServer{
    protected:

    ros::NodeHandle nh_;
    VelocitySmoother vs;
    bool getting_goal = false;
    actionlib::SimpleActionServer<pd_controller::pdAction> as_;
    pd_controller::pdResult result_;
    std::string action_name;
    ros::Publisher vel_publisher;
    ros::Subscriber odom_callback , laser_callback;
    double omega = 0.0;
    geometry_msgs::Twist vel_msg;
    nav_msgs::Odometry curr_pose;
    bool success = false;
    bresenham2D* find_obstacle;
    costmap_2d::Costmap2DROS* costmap2d;
    costmap_2d::Costmap2DROS *controller_costmap_ros_;
    tf2_ros::Buffer& tf_;
   
    public:
    ControllerServer(std::string name , tf2_ros::Buffer& tf):
    as_(nh_, name, boost::bind(&ControllerServer::executeCB, this, _1),false),
    action_name(name),
    controller_costmap_ros_(NULL),
    tf_(tf),
    vs(0.0, 0.0, 0.0)
    {

        vs = VelocitySmoother(0.4 , 0.25 , 0.4);
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->start();
       
        odom_callback = nh_.subscribe("/odom",1,&ControllerServer::odomCallback,this);
        vel_publisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
        find_obstacle = new bresenham2D();
        
        as_.start();
    }



    void sendZeroVel()
    {
        geometry_msgs::Twist command = geometry_msgs::Twist();
        command.linear.x = 0;
        command.angular.z = 0;
        vel_publisher.publish(command);    
    }

    void odomCallback(nav_msgs::Odometry pose)
    {
        curr_pose = pose;
    }

    ~ControllerServer(void)
    {
    }

    void executeCB(const pd_controller::pdGoalConstPtr &goal)
    {   

        geometry_msgs::PoseStamped pose = goal->position;


        ROS_INFO("got goal for %f and %f", pose.pose.position.x, pose.pose.position.y);
        
        while (true){

            controller_costmap_ros_->updateMap();

            bool a = find_obstacle->check_robot_path(pose.pose.position.x, pose.pose.position.y);
            if (a){
                sendZeroVel();
                ROS_WARN("Detected something in robot's path, please confirm. Sleeping for 2 secs.");
                ros::Duration(3).sleep();
                
                continue;}


            float desired_phi;            


            if (true)
            {    
            desired_phi =  atan2((pose.pose.position.y - curr_pose.pose.pose.position.y),
                                (pose.pose.position.x - curr_pose.pose.pose.position.x));
            }

            else{
            desired_phi =atan2((pose.pose.position.y - curr_pose.pose.pose.position.y),
                                (pose.pose.position.x - curr_pose.pose.pose.position.x));
                                
            }

            tf2::Quaternion q(
            curr_pose.pose.pose.orientation.x,
            curr_pose.pose.pose.orientation.y,
            curr_pose.pose.pose.orientation.z,
            curr_pose.pose.pose.orientation.w);            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;

            
            m.getRPY(roll, pitch, yaw);

            ROS_INFO("yaw is %f", yaw);

            double e = desired_phi - yaw;
            double e_ = atan2(sin(e),cos(e));

            PD pid = PD(0.1, 3, 0.05 ,0);
            double desired_rotate = pid.calculate(e_);

            double forward_vel = vs.smooth_velocity(e_);

            geometry_msgs::Twist command = geometry_msgs::Twist();
            command.angular.z = desired_rotate;

            command.linear.x = forward_vel;


            float distance_error = sqrt(pow((pose.pose.position.y - curr_pose.pose.pose.position.y),2) +
                                    pow((pose.pose.position.x - curr_pose.pose.pose.position.x),2));
            
            vel_publisher.publish(command);
            ros::Duration(0.5).sleep();

            if (distance_error < 0.5)
            {
                sendZeroVel();
                ROS_INFO("Close enough to goal, you may stop");
                success = true;
                as_.setSucceeded(result_);
                break;                
            }
        }           
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pd_server");

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    
    ControllerServer controller("pd_server", buffer);
    ros::spin();
    return 0;
}