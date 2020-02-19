#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
class bresenham2D
{
    
    ros::Subscriber costmap_subscriber, footprint , amcl_pose;
    ros::NodeHandle nh_;
    int height;
    bool costmap_ = false;
    int width;
    double resolution;
    int8_t data[64000];
    int8_t costmap_data[64000];
    double originX;
    double originY;
    double goalX = 15;
    double goalY = 2;
    double curr_pose_x;
    double curr_pose_y;
    bool kill =false;

    public:
    bresenham2D()
    {

    
       refresh_data();
      
      costmap_subscriber = nh_.subscribe("/move_base/local_costmap/costmap",1,&bresenham2D::costmapCallback,this);  
      amcl_pose = nh_.subscribe("/amcl_pose", 1 , &bresenham2D::amcl_callback, this);
      footprint = nh_.subscribe("/move_base/local_costmap/footprint", 1 ,&bresenham2D::footprintCallback,this);
    }

    void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose)
    {
        curr_pose_x = pose->pose.pose.position.x;
        curr_pose_y = pose->pose.pose.position.y;

        if (costmap_){find_line(curr_pose_x,curr_pose_y,goalX,goalY);}

    }

    void find_line(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2-y1;
        double P = 2*dy - dx;

        double x = x1;
        double y = y1;

        while (x <= x2){
            if (kill){ return; };
            
            put_point(x , y);

            x += resolution;

            if (P < 0){ P = P +2 * dy;}

            else{
                P = P +2 * dy - 2*dx;
                y += resolution;
            }
        }
    }

    void put_point(double x ,double y)
    {

        int columns = (x- originX)/resolution;
        int rows = (y- originY)/resolution;

        int index = rows * width + columns;

        if (index < (width*height))
        {
            
            data[index] = 100;
        }
        else{
            kill = true;
        }

    }

    void refresh_data()
    {
        for (auto i = 0 ; i< (height*width) ; i++)
        {
            data[i] = 0;
        }
    }


    void footprintCallback(geometry_msgs::PolygonStampedConstPtr footprint)
    {

        //refresh_data();
        
        for (auto i = footprint->polygon.points.begin(); i< footprint->polygon.points.end(); i++)
        {
            for (int k = 0 ; k<= 10 ; k ++)
            {find_line((i)->x - 5 + k, i->y - 5 + k , goalX + i->x - 5 + k , goalY + i->y - 5 + k);}
        }


        
        dot_product();
    }

    void dot_product()
    {
        double c = 0;
        for (auto i = 0; i< (width*height); i++)
        {
            c += data[i]*costmap_data[i];
        }

        if (c > 0 ){ROS_INFO("dot product %f" , c);}
    }


    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap )
    {
        height = costmap->info.height;
        width = costmap->info.width;
        resolution = costmap->info.resolution;
        originX = costmap->info.origin.position.x;
        originY = costmap->info.origin.position.y;
        costmap_ = true;

        

        for (auto i = 0 ; i< (height*width) ; i++)
        {
            costmap_data[i] = costmap->data[i];
            //ROS_INFO("%d", i);

            if (costmap->data[i] > 50){
                //ROS_INFO("obstacle at %d", i);
            //    //print_corresponding_coordinates(i);
            //}
            }
        }
        

    }

    
    /*
    void print_corresponding_coordinates(int x)
    {
        
        int rows = x / width ;
        int columns = x % height;

        double newX = originX + resolution*(columns);
        double newY = originY + resolution* (rows);
        ROS_INFO("%f %f", newX, newY);
    }
    */



};

int main (int argc, char** argv){
    ros::init(argc,argv,"bresenham");
    ros::NodeHandle nh;
    ros::Subscriber goal;
    
    
    bresenham2D b;
    ros::spin();
}
