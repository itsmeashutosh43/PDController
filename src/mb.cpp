#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<iostream>
#include "geometry_msgs/Pose.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class PD_controller
{
    public:
    PD_controller(std::string name):ac("/move_base", true)
    {
        boost::shared_ptr<nav_msgs::Path const> sharedEdge;

        ROS_INFO("%s Waiting For Server...", name.c_str());
        ac.waitForServer();
        ROS_INFO("Got a server ...");
        

        
        sharedEdge = ros::topic::waitForMessage<nav_msgs::Path>("/move_base/GlobalPlanner/plan");
        bool got_plan = false;
        if (sharedEdge != NULL)
        {
            ROS_INFO("Initial Plan recorded");
            init_global_plan = *sharedEdge;
            got_plan = true;
            
        }
    

        

        while (!got_plan)
        {
            ROS_INFO("Not got a plan yet");
            ros::Duration(2).sleep();
        }
        get_intermediate_goal(&init_global_plan);
        
    }

    
    void doneCb(const actionlib::SimpleClientGoalState& state,
        int status){
        //ROS_INFO("Finished in state [%s]", state.toString().c_str());
        //ROS_INFO("Result: %i", result->ok);
    };

    void get_intermediate_goal(nav_msgs::Path* global_plan)
    {
        int length = sizeof(global_plan->poses)/ sizeof(global_plan->poses[0]);
        for(int i = 0; i< length; i++)
        {
            goal.target_pose = global_plan->poses[i];
            ac.sendGoal(goal);
            //, boost::bind(&PD_controller::doneCb, this, _1,_2),
            //MoveBaseClient::SimpleActiveCallback(),MoveBaseClient::SimpleFeedbackCallback());
        }
    }

    private:
    nav_msgs::Path init_global_plan;
    MoveBaseClient ac;
    move_base_msgs::MoveBaseGoal goal;
};



int main (int argc, char** argv){
    ros::init(argc,argv,"PD_controller");
    ros::NodeHandle nh;
    std::string filename;
    ROS_INFO("Started now ");
    PD_controller p(ros::this_node::getName());
    ros::spin();
}