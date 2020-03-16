/*
Detects the possible obstacle and notifies pid_controller class in case of any.
*/



#include "bresenham2D.h"


bresenham2D::bresenham2D(double length_, _Smoother sm)
{    
  costmap_subscriber = nh_.subscribe("/move_base/local_costmap/costmap",1,&bresenham2D::costmapCallback,this);  
  amcl_pose = nh_.subscribe("/amcl_pose", 1 , &bresenham2D::amcl_callback, this);
  length = &length_;
  refresh_data();
  refresh_data_filter(sm);
}



void bresenham2D::amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose)
{
    curr_pose_x = pose->pose.pose.position.x;
    curr_pose_y = pose->pose.pose.position.y;
}




void bresenham2D::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap )
{

    if (m.try_lock())
    {
        height = costmap->info.height;
        width = costmap->info.width;
        resolution_ = costmap->info.resolution;  
        originX_ = costmap->info.origin.position.x;
        originY_ = costmap->info.origin.position.y;
            
        *costmap_ = true;
    
        for (auto i = 0 ; i< (height*width) ; i++)
        {
                costmap_data[i] = costmap->data[i];
        }
        m.unlock();
    }
        
        
    else{
            ROS_ERROR("lock not recieved for costmap update");
    }

}



bool bresenham2D::check_robot_path(double goalX, double goalY)
{
    if ((!(*costmap_)) ){
        ROS_INFO("Either there's no footprint or costmap here!");
        ros::Duration(1).sleep();
        return true;}

    std::thread t1(&bresenham2D::compute, this, &sem , goalX, goalY) ;
    t1.join();

    if (sem == 1)
    {   return true;
    }
    return false;
}


int bresenham2D::compute(int *x, double goalX,double goalY)
{
    if (m.try_lock())
    {
        refresh_data();
        find_line(curr_pose_x,curr_pose_y,goalX,goalY);
        
    if (convolve())
    {
        *x = 1;
        m.unlock();
        return 0;
    }
    
    }

    else{
            ROS_ERROR("lock not recieved");
    }
    m.unlock();
    *x = 0;
}



bool bresenham2D::convolve()
{
    cv::Mat kernel = cv::Mat(40,40, CV_32F, &filter);
    cv::Mat src = cv::Mat((80) ,(80) , CV_32F, &data);
    cv::filter2D(src,dest,-1,kernel);
    cv::imshow("",dest);
    cv::waitKey(1);
    dest = dest.reshape(1, dest.rows * dest.cols);
    cv::Mat costmap_data_mat = cv::Mat(6400,1,CV_32F,&costmap_data);

    double dot_sum = dest.dot(costmap_data_mat);

    if (dot_sum > 10)
    {
        ROS_INFO("Dot product is %f", dot_sum);
        return true;
    }

    return false;    
}


void bresenham2D::find_line( double x,double y,double x2, double y2){


    double w = (x2 - x)/0.025 ;
    double h = (y2 - y)/0.025 ;
    double dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0 ;
    if (w<0) dx1 = -0.025 ; else if (w>0) dx1 = 0.025 ;
    if (h<0) dy1 = -0.025 ; else if (h>0) dy1 = 0.025 ;
    if (w<0) dx2 = -0.025 ; else if (w>0) dx2 = 0.025 ;
    int longest = abs(w) ;
    int shortest = abs(h) ;
    if (!(longest>shortest)) {
        longest = abs(h) ;
        shortest = abs(w) ;
        if (h<0) dy2 = -0.025 ; else if (h>0) dy2 = 0.025 ;
        dx2 = 0 ;            
    }
    int numerator = longest >> 1 ;
    for (int i=0;i<=longest;i++) {

        world_to_map(x,y);
        numerator += shortest ;
        if (!(numerator<longest)) {
            numerator -= longest ;
            x += dx1 ;
            y += dy1 ;
        } else {
            x += dx2 ;
            y += dy2 ;
        }
    }
 
}

void bresenham2D::world_to_map(double x ,double y)
{
    if (x < *originX || y < *originY)
    {
        return;
    }

    int mx = (x - *originX)/ *resolution;
    int my = (y - *originY)/ *resolution;
    int index = my * 80 + mx;

    if ( mx < 80 && my < 80)
    {
        data[index] = 1; 
        }
}






void bresenham2D::refresh_data()
{
    for (int i = 0; i< 80*80 ; i++)
    {
        data[i] = 0;
    }
}



void bresenham2D::refresh_data_filter(_Smoother sm)
{
    for(int i = 0; i< 40 ;i++)
    {
        for (int j =0 ; j < 40 ; j++)
        {
            if (within_distance(i ,j , 20 ,20))
            {
                filter[i][j] = sm.smoother_funct(2, 0 , 0.09,distance(i,j,20,20));
            }
        }
    }
}


bool bresenham2D::within_distance(int x1, int y1 , int x0 , int y0)
    {

        float distance_ = distance(x1, y1, x0 , y0);

        if (distance_ > 20)
        {
            return false;
        }

        return true;

    }

float bresenham2D::distance(int x1, int y1 , int x0 ,int y0)
{
    return std::sqrt(std::pow(x1 - x0,2) + std::pow(y1 - y0,2));
}




