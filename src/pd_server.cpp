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

class ControllerServer{
    protected:
    ros::NodeHandle nh_;
    bool interrupt = false;
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
    bresenham2D find_obstacle;


    public:
    ControllerServer(std::string name):
    as_(nh_, name, boost::bind(&ControllerServer::executeCB, this, _1),false),
    action_name(name)
    {
       
        odom_callback = nh_.subscribe("/odom",1,&ControllerServer::odomCallback,this);
        vel_publisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
        find_obstacle = bresenham2D();
        
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


        while (true){

            if (find_obstacle.check_robot_path(pose.pose.position.x, pose.pose.position.y)){
                ros::Duration(1).sleep();
                continue;}

        
                
            float desired_phi = atan((pose.pose.position.y - curr_pose.pose.pose.position.y)/
                                (pose.pose.position.x - curr_pose.pose.pose.position.x));


            tf2::Quaternion q(
            curr_pose.pose.pose.orientation.x,
            curr_pose.pose.pose.orientation.y,
            curr_pose.pose.pose.orientation.z,
            curr_pose.pose.pose.orientation.w);
            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            
            m.getRPY(roll, pitch, yaw);
            
            double e = desired_phi - yaw;
            double e_ = atan2(sin(e),cos(e));

            PD pid = PD(0, 1.5, 0 ,0);

            double desired_rotate = pid.calculate(e_);

            geometry_msgs::Twist command = geometry_msgs::Twist();
            command.angular.z = desired_rotate;

            if (pose.pose.position.x > 0){command.linear.x = 0.25;}
            else{command.linear.x = -0.25;}


            float distance_error = sqrt(pow((pose.pose.position.y - curr_pose.pose.pose.position.y),2) +
                                    pow((pose.pose.position.x - curr_pose.pose.pose.position.x),2));
            

            vel_publisher.publish(command);
            ros::Duration(1).sleep();

            ROS_INFO("%f" , abs(distance_error));

            if (distance_error < 0.5)
            {
                sendZeroVel();
                ROS_INFO("Close enough to goal, you may stop");
                success = true;
                break;                
            }
        }   


        if(success)
        {
                
                // set the action state to succeeded
                as_.setSucceeded(result_);
        }

        
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pd_server");
    ControllerServer controller("pd_server");
    ros::spin();
    return 0;
}