#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "pid_implementation.h"
#include "smoother.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/trajectory_planner_ros.h>



namespace pd_controller{
    class PDController : public nav_core::BaseLocalPlanner{
        public:
        PDController();
        ~PDController();
        void initialize(std::string name, tf2_ros::Buffer* tf , costmap_2d::Costmap2DROS* costmap_ros);
        bool isGoalReached();
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


        private:
         tf2_ros::Buffer* tf_;
         costmap_2d::Costmap2DROS *costmap_ros_;
         base_local_planner::TrajectoryPlannerROS collision_planner_;
         geometry_msgs::PoseStamped goal;
         bool goal_reached = false;


    };
};