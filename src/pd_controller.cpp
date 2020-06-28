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
        collision_flag = config.collision_flag;
        rotate_to_goal = config.rotate_to_goal;
        angular_tolerance = config.angular_tolerance;
        linear_tolerance = config.linear_tolerance;
        k_p = config.k_p;
        k_i = config.k_i;
        k_d = config.k_d;
    }
    

    void PDController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {


        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ros_->getRobotPose(robot_pose);

        ROS_INFO("Frame is in %s", robot_pose.header.frame_id.c_str());

        geometry_msgs::PoseStamped global_pose;
        tf_->transform(robot_pose , global_pose,  "map");

        ROS_INFO("x and y initial amcl_pose is %f %f", global_pose.pose.position.x, global_pose.pose.position.y);
        

        ROS_INFO("Frame is in %s", global_pose.header.frame_id.c_str());

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
        

        if (!costmap_ros_->getRobotPose(robot_pose))
        {
            ROS_ERROR("Can't get robot pose");
            geometry_msgs::Twist empty_twist;
            cmd_vel = empty_twist;
            return false;
        }


        geometry_msgs::PoseStamped global_pose;
        tf_->transform(robot_pose , global_pose,  "map");


        float desired_phi =  atan2((goal.pose.position.y - global_pose.pose.position.y),
                                (goal.pose.position.x - global_pose.pose.position.x));

        float yaw = check_yaw(global_pose);

        double desired_rotate = check_desirable_rotation(desired_phi,yaw);

        
        double forward_vel = vs.smooth_velocity(vel_forward , 0.2 , 0.4, desired_rotate);

        float distance_error = sqrt(pow((goal.pose.position.y - global_pose.pose.position.y),2) +
                                    pow((goal.pose.position.x - global_pose.pose.position.x),2));

        


        

        if (distance_error < linear_tolerance && rotate_to_goal)
        {
            double desired_yaw = check_yaw(goal);
            float desired_rotate = check_desirable_rotation(desired_yaw , yaw);

            double e = desired_yaw - yaw;
            double e_ = atan2(sin(e),cos(e));

            ROS_INFO("error %f vel_rot %f", abs(e_),vel_rot);

            if (abs(e_) >= angular_tolerance)
            {
                ROS_INFO("Rotate to goal executing");
                send_command_vel(cmd_vel, 0 , desired_rotate);
                return true;
            }

            else{
                ROS_INFO("Reached Tolerance level : angular distance");
                stopped = true;
                return true;
            } 
        }


        else if (distance_error < linear_tolerance)
        {
            ROS_INFO("Reached tolerance level : linear distance");
            stopped = true;
            return true;
        }

        stopped = false;

        if (!collision_flag)
        {
            ROS_INFO("Flag not checking for collision. This is dangerous. Set collision_flag to TRUE");
            send_command_vel(cmd_vel, forward_vel,desired_rotate);
            return true;
        }

        
        bool legal_traj = collision_planner_.checkTrajectory(forward_vel, 0 , desired_rotate, true);

        if (legal_traj)
        {
            send_command_vel(cmd_vel, forward_vel ,desired_rotate);
        }
        
        else{
            ROS_INFO("The computed trajectory is not a legal one. Sending zero velocity");
            send_command_vel(cmd_vel, 0 ,0);
        }
        return true;

    }


    void PDController::send_command_vel(geometry_msgs::Twist& cmd_vel, double f_vel ,double rot_vel){
        cmd_vel.linear.x = f_vel;
        cmd_vel.angular.z = rot_vel;
    }

    double PDController::check_desirable_rotation(double desired_yaw ,double yaw)
    {
        double e = desired_yaw - yaw;
        double e_ = atan2(sin(e),cos(e));
        PD pid = PD(0.1, k_p, k_d ,k_i);
        double desired_rotate = pid.calculate(e_,vel_rot);
        return desired_rotate;

    }

    double PDController::check_yaw(geometry_msgs::PoseStamped pose)
    {
        tf2::Quaternion q(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w);            
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;

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


