/*
Detects the possible obstacle and notifies pid_controller class in case of any.
*/



#include "bresenham2D.h"
#include "costmap_2d/costmap_2d_ros.h"
#include <random>
#include <thread>
#include <future>


bresenham2D::bresenham2D(double length_)
{    
  ROS_ERROR("bresenham started");


  

  //put_points(0.8,1.5);
  costmap_subscriber = nh_.subscribe("/move_base/local_costmap/costmap",1,&bresenham2D::costmapCallback,this);  
  amcl_pose = nh_.subscribe("/amcl_pose", 1 , &bresenham2D::amcl_callback, this);

  ROS_INFO("Waiting for costmap to come up.");
  

  length = &length_;


  refresh_data();
  refresh_data_filter();

  //controller_costmap_ros_ = controller_costmap_ros_;

  ros::Duration(2).sleep();
}


cv::Mat* bresenham2D::convolve()
{
    


    cv::Mat kernel = cv::Mat(40,40, CV_32F, &filter);
    cv::Mat src = cv::Mat((int)(120) ,(int)(120) , CV_32F, &data);

    
    cv::filter2D(src,dest,-1,kernel);

    dest = dest.reshape(1, dest.rows * dest.cols);

    return &dest;

    //cv::imshow("_show_",dest);
    //cv::waitKey(0);
}


int bresenham2D::compute(int *x, double goalX,double goalY)
{

    
    if (m.try_lock())
    {
        refresh_data();
        find_line(curr_pose_x,curr_pose_y,goalX,goalY);
        cv::Mat* output = convolve();
    if (dot_product(output))
    {
        *x = 1;
        //refresh_data();
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



void bresenham2D::amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose)
{
    
    curr_pose_x = pose->pose.pose.position.x;
    curr_pose_y = pose->pose.pose.position.y;

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
    {
        return true;
    }

    return false;


}


/*
void bresenham2D::find_line(double x1, double y1, double x2, double y2)
{
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double P = 2 * dy - dx;

    double x = x1;
    double y = y1;

    while (x <= x2){
        if (*kill_p)
        {
            return;
        }
        put_point(x , y);
        //points_inside_ellipse(x , y);

        x += *resolution;

        if (P < 0){ P = P + 2 * dy;}

        else{
            P = P +2 * dy - 2*dx;
            y += *resolution;
        }
    }
}

*/

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
        //putpixel(x,y,color) ;

        put_point(x,y);
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
 


/*
void bresenham2D::points_inside_ellipse( double x_0 , double y_0)
{

    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<> rho(0 , 1);
    std::uniform_real_distribution<> phi(0 , PI);

    for (int  i = 0 ; i < 1000 ; i++)
    {
        double x = sqrt(rho(rng)) * cos(phi(rng));
        double y = sqrt(rho(rng)) * sin(phi(rng));


        x = x_0 + x * 1.5;
        y = y_0 + y * 1.5;

        put_point(x,y);

    }

    *semaphore = true;
}
*/



void bresenham2D::put_point(double x ,double y)
{

    int rows = (x- *originX)/ *resolution;
    int columns = (y- *originY)/ *resolution;
    int index = rows * 80 + columns;
    if (index < 0)
    {
        return;
    }
    if (index < (width*height))
    {            
        data[rows + 20][columns + 20 ] = 1;
    }

    else{
        //kill = true;
    }
    
}


/*

void bresenham2D::put_points(double c_width, double c_length)
{

    ROS_INFO("%f %f", c_width,c_length);

    int iter_x = (int)(c_width / *resolution);
    int iter_y = (int)(c_length/ *resolution);

    ROS_INFO("%d %d", iter_x , iter_y);


    int start_x = 40 - (iter_x);

    int end_x = 40 + (iter_x);


    for (auto rows = 40 ; rows <= 80 ; rows++)
    {
        for (auto columns = start_x; columns<= end_x ; columns++)
        {
            int index = rows * 80 + columns;
            
            print_corresponding_coordinates(index);
            data[index] = 100;
        }
    }

}


*/

void bresenham2D::refresh_data()
{
    for (int i = 0; i< 120 ; i++)
    {
        for (int j = 0; j< 120 ; j++)
        {
            data[i][j] = 0;
        }
    }
}





void bresenham2D::refresh_data_filter()
{
    for(int i = 0; i< 40 ;i++)
    {
        for (int j =0 ; j < 40 ; j++)
        {
            if (distance(i ,j , 20 ,20))
            {
                filter[i][j] = 10;
            }
        }
    }
}


bool bresenham2D::dot_product(cv::Mat* output)
{
    double c = 0;
    for (auto i = 0; i< (width*height); i++)
    {
        //ROS_INFO("%d %d", data[i], costmap_data[i]);
        c += (output->at<int>(i))*costmap_data[i];
    }

    if (c > 0 ){
        
    ROS_INFO("dot product %f" , c);
    return true;
    }

    return false;
}


void bresenham2D::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap )
{

        if (m.try_lock())
        {


        bool flag = false;
        if (*costmap_count_ == 0)
        {
            
            flag = true;
            prev = ros::Time::now().toSec();
        }

        (*costmap_count_)+=1;

        if (true){
            height = costmap->info.height;
            width = costmap->info.width;
            resolution_ = costmap->info.resolution;
            
            originX_ = costmap->info.origin.position.x;
            
            originY_ = costmap->info.origin.position.y;
            
            *costmap_ = true;

            prev = ros::Time::now().toSec();
        

            for (auto i = 0 ; i< (height*width) ; i++)
            {
                costmap_data[i] = costmap->data[i];
                    //ROS_INFO("%d", i);

            }

        }
        m.unlock();
        }


        else{
            ROS_ERROR("lock not recieved");
        }

        

}

    
    
    void bresenham2D::print_corresponding_coordinates(int x)
    {
        
        int rows = x / 80 ;
        int columns = x % 80;

        double newX = *originX + *resolution*(columns);
        double newY = *originY + *resolution* (rows);
        ROS_INFO("%f %f", newX, newY);
    }

    /*
    
int main (int argc, char** argv){
    ros::init(argc,argv,"bresenham");
    ros::NodeHandle nh;
    ros::Subscriber goal;
    
    
    bresenham2D b;
    ros::spin();
}
*/
