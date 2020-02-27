#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include <pd_controller/pdAction.h>
#include <actionlib/client/simple_action_client.h>
#include<iostream>
#include "geometry_msgs/Pose.h"

#include "pd_controller/coordinate.h"
#include "pd_controller/goal.h"

/*

Client side for pd controller
desc:
send goals from here
implementation:
Inherit SimpleActionClient in PD_Controller class which provides interface to all]
functionalities.
Read initial global plan and send them sequentially to pd_server.cpp


*/

using namespace pd_controller;
typedef actionlib::SimpleActionClient<pdAction> Client;

class PD_controller
{
    public:




    PD_controller(std::string name):ac(name, true)
    {
        
        //goal = n.subscribe("move_base/goal",1,&Eval::goalCallback,this);

        pb = nh.subscribe("/given_goal", 1 , &PD_controller::goalCallback,this);

        ROS_INFO("%s Waiting For Server...", name.c_str());
        ac.waitForServer();
        ROS_INFO("Got a server ...");


       ROS_INFO("Waiting for everything to go up");
       ros::Duration(1).sleep();
       //get_intermediate_goal();
        
    }


    void goalCallback(const pd_controller::goal goal)
    {
        double x = goal.goal.x;
        double y = goal.goal.y;


        ROS_INFO("Got goals for coordinate %f and %f",x,y);    
        get_intermediate_goal(x,y);
    }

    
    void doneCb(const actionlib::SimpleClientGoalState& state,
        const pd_controller::pdResultConstPtr& result){
        
    };

    void get_intermediate_goal(double x, double y)
    {
        
        geometry_msgs::PoseStamped goal_;
        
        goal_.pose.orientation.w = 1;
        goal_.pose.position.x = x;
        goal_.pose.position.y = y;

        goal.position = goal_;
        
        ac.sendGoal(goal, boost::bind(&PD_controller::doneCb, this, _1,_2),
            Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());
        
    }

    private:

    ros::NodeHandle nh;
    ros::Subscriber pb;
    nav_msgs::Path init_global_plan;
    Client ac;
    pd_controller::pdGoal goal;
};



int main (int argc, char** argv){
    ros::init(argc,argv,"PD_controller");
    
    ros::Subscriber goal;
    std::string filename;
    ROS_INFO("Started now ");
    PD_controller p("pd_server");
    ros::spin();
}
