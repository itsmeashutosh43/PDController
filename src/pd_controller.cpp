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
    

    void PDController::reconfigureCB(pd_controller::PDControllerConfig &config , uint32_t level){

        ROS_INFO("Updated velocities %f %f", config.vel_lin, config.vel_rot);

        vel_forward = config.vel_lin;
        vel_rot = config.vel_rot;
        collision_flag = collision_flag;
    }
    

    void PDController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {


        tf_ = tf;
        costmap_ros_ = costmap_ros;
        vs = _Smoother();
        collision_planner_.initialize(name + "/collision_planner", tf_, costmap_ros_);
    
        ros::NodeHandle private_nh("~/" + name);

        ROS_INFO("Initialised custom local planner");

        dsrv_ = new dynamic_reconfigure::Server<pd_controller::PDControllerConfig>(ros::NodeHandle(private_nh));

        dynamic_reconfigure::Server<pd_controller::PDControllerConfig>::CallbackType cb = boost::bind(&PDController::reconfigureCB, this, _1,_2);

        dsrv_->setCallback(cb);
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

        double desired_rotate = pid.calculate(e_,vel_rot);

        double forward_vel = vs.smooth_velocity(vel_forward , 25 , 0.4, e_);

        float distance_error = sqrt(pow((goal.pose.position.y - robot_pose.pose.position.y),2) +
                                    pow((goal.pose.position.x - robot_pose.pose.position.x),2)); 


        if (distance_error < 0.5)
        {
            ROS_INFO("Reached tolerance level ");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            stopped = true;
            return true;
        }

        stopped = false;


        bool legal_traj = collision_planner_.checkTrajectory(forward_vel, 0 , desired_rotate, true);

        if (legal_traj)
        {
            cmd_vel.linear.x = forward_vel;
            cmd_vel.angular.z = desired_rotate;
        }
        
        else{
            ROS_INFO("The computed trajectory is not a legal one. Sending zero velocity");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        }
        return true;

    }

    bool PDController::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {

        goal = global_plan.back();
        return true;
    }

    bool PDController::isGoalReached(){

        if (stopped){
            stopped = false;
            return true;
        }

        return false;

    }    

};


