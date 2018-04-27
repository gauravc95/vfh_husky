#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>      
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"
#include <angles/angles.h>

#define PI 3.14159265
#define DMAX 10*sqrt(2)
#define A 8
#define B 0.5
#define ALPHA 10
#define SMAX 5



//creatindg the matrix
std::vector<std::vector<double> > histogramGrid(20, std::vector<double>(20, 0));  //20 rows and 20 columns initialized to 0
std::vector<std::vector<double> > direction(20, std::vector<double>(20, 0));
std::vector<std::vector<double> > magnitude(20, std::vector<double>(20, 0));


double curr_x=10,curr_y=10;
   double x=0.0;
   double y=0.0;
   double theta=0.0;


//if  distance is not between 29-31 then it has obstacle
double bot_position_x=10,bot_position_y=10;
double findx(double x,double y)
{
          double x1=0,y1=0;

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

    return x1;
}

double findy(double x,double y)
{   double x1=0,y1=0;

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

  return y1;
}
//update
void maptomatrix(const sensor_msgs::LaserScan::ConstPtr& scan)  //to map value from polar to matrix
{
        double x1=0,y1=0;
        double sox=0;
        double soy=0;


        for(int i = 0 ; i < scan->ranges.size(); i++){
            ROS_INFO("Value at angle %f is : %f", (i*scan->angle_increment+scan->angle_min)*180/PI, scan->ranges[i]);
            std::cout<<"robot ---x"<<x<<"robot --y"<<y;

            std::cout<<"robot x"<<findx(x,y)<<"robot y"<<findx(x,y);
/*
            if(findx(x,y)==10 && findy(x,y)==10)
            {
                sox=0;
                soy=0;
            }
            else
            {
                sox=10;
                soy=10;
            }*/

            if (scan->ranges[i]>29 && scan->ranges[i]<31){
                    //not an obstacle
                }

            else{
                //this is obstacle
                //find x and y coordinate for every range
                //currrently at origin
                    double xi=scan->ranges[i]*cos((135*PI/180)-(i*scan->angle_increment)+theta);
                    double yi=scan->ranges[i]*sin((135*PI/180)-(i*scan->angle_increment)+theta);   
                    std::cout<<"Now before mapping with theta :"<<" x :"<<xi<<"y :"<<yi<<   "\n";


                    x1=findx(xi+x,yi+y);
                    y1=findy(xi+x,yi+y);

                    
                    // y1 = curr_x + x;
                    // x1 = curr_y - y;

                     std::cout<<"Now after mapping :"<<" x :"<<x1<<"y :"<<y1<<"\n";

                    histogramGrid[abs(x1)][abs(y1)]=histogramGrid[abs(x1)][abs(y1)]+1;

            }
    
        }



}



void directionMatrix(){

for(int i=0;i<20;i++){
  for(int j=0;j<20;j++){
    direction[i][j]=atan((j-bot_position_y)/(i-bot_position_x))*180/PI; 
  }
  
}


}

double distance(double x1 ,double y1 ){
  return sqrt((x1-bot_position_x)*(x1-bot_position_x)+(y1-bot_position_y)*(y1-bot_position_y));
}

void magnitudeMatrix(){
  for(int i=0;i<20;i++){
    for(int j=0;j<20;j++){
      magnitude[i][j]=(histogramGrid[i][j])*(histogramGrid[i][j])*(A-B*distance(i,j));  
    }
  }
}


void printMatrix(){

       //Now the matrix will be

      std::cout<<"===========================================\n";
      for(int i=0;i<20;i++){
        for (int j = 0; j < 20; j++)
        {
          /* code */
          std::cout<<histogramGrid[i][j]<<" ";
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


void printDirectionMatrix(){

       //Now the matrix will be

      std::cout<<"===========================================\n";
      for(int i=0;i<20;i++){
        for (int j = 0; j < 20; j++)
        {
          /* code */
          std::cout<<direction[i][j]<<" ";
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


void printMagnitudeMatrix(){

       //Now the matrix will be

      std::cout<<"===========================================\n";
      for(int i=0;i<20;i++){
        for (int j = 0; j < 20; j++)
        {
          /* code */
          std::cout<<magnitude[i][j]<<" ";
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
      printMatrix();

}



   //Initializing the positions and z axis orientation

   
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
