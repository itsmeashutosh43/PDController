#include "pd_controller.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(pd_controller::PDController, nav_core::BaseLocalPlanner)

namespace pd_controller
{
    PDController::PDController(){

    }

    PDController::~PDController()
    {

    }

    void PDController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;

        collision_planner_.initialize(name + "/collision_planner", tf_, costmap_ros_);
        ros::NodeHandle node;

        ROS_INFO("Initialised custom local planner");
    }

    bool PDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        geometry_msgs::PoseStamped robot_pose;

        if (!costmap_ros_->getRobotPose(robot_pose))
        {
            ROS_ERROR("Can't get robot pose");
            geometry_msgs::Twist empty_twist;
            cmd_vel = empty_twist;
            return false;
        }


        ROS_INFO("Current pose of the robot is %f %f",robot_pose.pose.position.x, robot_pose.pose.position.y);


        float desired_phi =  atan2((goal.pose.position.y - robot_pose.pose.position.y),
                                (goal.pose.position.x - robot_pose.pose.position.x));

        tf2::Quaternion q(
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w);            
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;

            
        m.getRPY(roll, pitch, yaw);

        double e = desired_phi - yaw;
        double e_ = atan2(sin(e),cos(e));


        PD pid = PD(0.1, 3, 0.05 ,0);

        double desired_rotate = pid.calculate(e_);

        double forward_vel = 0.2;

        float distance_error = sqrt(pow((goal.pose.position.y - robot_pose.pose.position.y),2) +
                                    pow((goal.pose.position.x - robot_pose.pose.position.x),2)); 


        if (distance_error < 0.5)
        {
            ROS_INFO("Reached tolerance level ");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            goal_reached = true;
            return true;
        }

        cmd_vel.linear.x = forward_vel;
        cmd_vel.angular.z = desired_rotate;
        
        return true;

    }

    bool PDController::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
        // if this does not work, subscribe to move_base goals
        goal = global_plan.back();

        ROS_INFO("Goal is %f %f",goal.pose.position.x, goal.pose.position.y);

        return true;
    }

    bool PDController::isGoalReached(){

        ROS_INFO("Is goal reached %d", goal_reached);
        return goal_reached;

    }    

};


