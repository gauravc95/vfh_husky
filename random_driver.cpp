#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>      
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"
#include <angles/angles.h>
#define PI 3.14159265



//creatindg the matrix
std::vector<std::vector<double> > matrix(20, std::vector<double>(20, 0));   //20 rows and 20 columns initialized to 0
double curr_x=10,curr_y=10;


//if  distance is not between 29-31 then it has obstacle



void maptomatrix(const sensor_msgs::LaserScan::ConstPtr& scan)  //to map value from polar to matrix
{
        double x1=0,y1=0;


        for(int i = 0 ; i < scan->ranges.size(); i++){
            ROS_INFO("Value at angle %f is : %f", (i*scan->angle_increment+scan->angle_min)*180/PI, scan->ranges[i]);

            if (scan->ranges[i]>29 && scan->ranges[i]<31){
                    //not an obstacle
                }

            else{
                //this is obstacle
                //find x and y coordinate for every range
                //currrently at origin
                    double x=scan->ranges[i]*cos((135*PI/180)-(i*scan->angle_increment));
                    double y=scan->ranges[i]*sin((135*PI/180)-(i*scan->angle_increment));   
                    std::cout<<"i :"<<i<<" scan range : "<<scan->ranges[i]<<" x :"<<x<<"\n";
                    std::cout<<"y :"<<y<<"\n";

                    if(x>0 && y>0){
                        x1=curr_x-y;
                        y1=x+curr_y;
                    }

                    else if(x<0 && y<0){
                        x1=(-1)*y+curr_x;
                        y1=(-1)*x+curr_y;
                    }

                    else if(x>0 && y<0){
                        x1=(-1)*y+curr_x;
                        y1=x+curr_y;
                    }
                    
                    else if(x<0 && y>0){
                        y1=curr_y-x;
                        x1=curr_x-y;
                    }
                    // y1 = curr_x + x;
                    // x1 = curr_y - y;

                    // std::cout<<"Now :"<<i<<" scan range : "<<scan->ranges[i]<<" x :"<<x1<<"\n";
                    // std::cout<<"y :"<<y1<<"\n";

                    matrix[abs(x1)][abs(y1)]=matrix[abs(x1)][abs(y1)]+1;

            }
    
        }



}

void displaygrid() //display the matrix[20][20]
{
     //Now the matrix will be

        std::cout<<"===========================================\n";
        for(int i=0;i<20;i++){
            for (int j = 0; j < 20; j++)
            {
                /* code */
                std::cout<<matrix[i][j]<<" ";
                if(j%9==0 && j!=0 && j!=18){
                    std::cout<<"||";
                }

            }
                std::cout<<"\n";
                if(i%9==0 && i!=0 && i!=18){
                            std::cout<<"===========================================\n";

                }
        }
    
        std::cout<<"===========================================\n";


}


void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
    //Ranges is the unbounded array i.e. vector and it has size of 720
        ROS_INFO("Printig....%lu", scan->ranges.size());
        //const nav_msgs :: Odometry::ConstPtr& odom;

        maptomatrix(scan);
        displaygrid();
}



   //Initializing the positions and z axis orientation
   double x=0.0;
   double y=0.0;
   double theta=0.0;
   
void odomCallback(const nav_msgs :: Odometry::ConstPtr& odom){
    //rostopic show Odometry

    x=odom->pose.pose.position.x;
    y=odom->pose.pose.position.y;


    //curr_x=x;
    //curr_y=y;


    double quat_x=odom->pose.pose.orientation.x;
    double quat_y=odom->pose.pose.orientation.y;
    double quat_z=odom->pose.pose.orientation.z;
    double quat_w=odom->pose.pose.orientation.w;

    tf::Quaternion q(quat_x, quat_y, quat_z, quat_w);       //Quaternion gives the rotation 
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);                             //Changing queternion to Euler which gives 3 values 
                                                            //yaw is rotation around z axis
    theta=yaw;
    std::cout<<"Rotation around z axis is"<<theta*180/PI;


    //Now we will make a    
}


int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "random_husky_commands");
    ros::NodeHandle nh;
    ros::Subscriber scanSub;


    //Ceates the publisher, and tells it to publish
    //to the husky_velocity_controller/cmd_vel topic, with a queue size of 100
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 100);
    scanSub=nh.subscribe<sensor_msgs::LaserScan>("scan",10,processLaserScan);
        
    ros::Subscriber sub = nh.subscribe("/odometry/filtered", 10, odomCallback);


    geometry_msgs::Point goal;              //rostopic show Odometry
    //say goal is (5,5)
    goal.x=5;
    goal.y=5;

    geometry_msgs::Twist deep;              ///cmd_vel

    //Sets up the random number generator
    srand(time(0));

    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(10);

      while(ros::ok()) {

            double inc_x=goal.x-x;
            double inc_y=goal.y-y;
            
            
            double angle_to_goal=atan2(inc_y,inc_x);        //atan2 returns in radians and it is inverse tan.

            if (abs(angle_to_goal - theta) > 0.1){          //if the vehicle is not facing to goal then change angle
                deep.linear.x=0.0;
                deep.angular.z=0.3;
            }

            else{                                           //if vehicle is facing to goal then go straight and dont change angle
                deep.linear.x=0.5;
                deep.angular.z=0.0;

            }

            //Publish the message
            pub.publish(deep);
            //Delays until it is time to send another message
            rate.sleep();
            ros::spinOnce();


        }


}
