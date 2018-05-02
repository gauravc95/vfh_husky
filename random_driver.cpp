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
#define NO_SECTORS 360/ALPHA
#define THRESHOLD 10
#define SAFE_SECTOR_CNT 5


    geometry_msgs::Twist deep;              ///cmd_vel
int cnt=0;
//creatindg the matrix
std::vector<std::vector<double> > histogramGrid(20, std::vector<double>(20, 0));  //20 rows and 20 columns initialized to 0
std::vector<std::vector<double> > direction(20, std::vector<double>(20, 0));
std::vector<std::vector<double> > magnitude(20, std::vector<double>(20, 0));
std::vector<double> h(NO_SECTORS);
std::vector<double> candidateSectors;
double curr_x=10,curr_y=10;
double dest_x=5,dest_y=5;

//if  distance is not between 29-31 then it has obstacle
double bot_position_x,bot_position_y;
 //Initializing the positions and z axis orientation
   double x;
   double y;
   double theta=0.0;
   

//if  distance is not between 29-31 then it has obstacle
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
           // std::cout<<"robot ---x"<<x<<"robot --y"<<y;

           // std::cout<<"robot x"<<findx(x,y)<<"robot y"<<findx(x,y);
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

                   //std::cout<<"Now after mapping :"<<" x :"<<x1<<"y :"<<y1<<"\n";
                     if(x1>=0 && x1<20 && y1>=0 && y1<20 && histogramGrid[abs(x1)][abs(y1)] < 5 
                      && abs(x1) != bot_position_x && abs(y1) != bot_position_y)
                      { histogramGrid[abs(x1)][abs(y1)]=histogramGrid[abs(x1)][abs(y1)]+1;
                      }

            }
    
        }



}

void findPolarObstacleDensity(){
  for(int i = 0 ; i < NO_SECTORS ; i++){
    h[i] = 0.0;
  }
  for(int i = 0 ; i < 20 ; i++){
    for(int j = 0 ; j < 20 ; j++){
      h[ (direction[i][j]/ALPHA) ] += magnitude[i][j] ;
    }
  }
}

double findTheta(){
  int k_targ = direction[dest_x][dest_y] / ALPHA;
  std::cout<<" ktarg="<<k_targ<<"\n";
  if(h[k_targ] == 0) return (k_targ*ALPHA + (ALPHA/2));             //simply go straight


  int k_f_l = 0, k_f_r = 0;
  int k_n_l = k_targ, k_n_r  = k_targ;
  int cnt = 0 ;

  //calculate start sectors
  while(h[k_n_l] > THRESHOLD && cnt++ < NO_SECTORS){ 
    k_n_l = (NO_SECTORS+k_n_l+1)%(NO_SECTORS);
  }
  cnt = 0;
  while(h[k_n_r] > THRESHOLD  && cnt++ < NO_SECTORS){ 
    k_n_r = (NO_SECTORS+k_n_r-1)%(NO_SECTORS);
  }
  k_n_l = (NO_SECTORS+k_n_l-1)%(NO_SECTORS);
  k_n_r = (NO_SECTORS+k_n_r+1)%(NO_SECTORS);


  std::cout<<"k_n_l="<<k_n_l<<"  && k_n_r="<<k_n_r<<"\n";
  //calculate end sectors
  cnt = 0;
  for(int i = 1 ; h[(NO_SECTORS + k_n_l+i)%(NO_SECTORS)] < THRESHOLD && cnt++ < SAFE_SECTOR_CNT ; i++){
    k_f_l++;
  }
  cnt=0;
  for(int i = 1 ; h[(NO_SECTORS + k_n_r-i)%(NO_SECTORS)] < THRESHOLD && cnt++ < SAFE_SECTOR_CNT ; i++){
    k_f_r++;
  }

  std::cout<<"k_f_l="<<k_f_l<<"  && k_f_r="<<k_f_r<<"\n";
  if(std::max(k_f_l, k_f_r)*ALPHA < SMAX){
      std::cout<<"CANNOT TURN!!!"<<std::endl;   
      return -1;
    }

  if(k_f_l < k_f_r && k_f_l >= SAFE_SECTOR_CNT){
    return (k_n_l*ALPHA) + (((k_f_l*ALPHA) + SMAX)/2);
  }
  else
    return (k_n_r*ALPHA) - (((k_f_r*ALPHA) + SMAX)/2);
}

void printPolarObstacleDensity(){
  std::cout<<"-------------------------------------------\n";
  for(int i = 0 ; i < NO_SECTORS ; i++){
    std::cout<<i<<" => "<<h[i]<<std::endl;
  }
  std::cout<<"-------------------------------------------\n";
}




void directionMatrix(){


// for(int i=0;i<20;i++){
//   for(int j=0;j<20;j++){

//     direction[i][j]=atan((j-bot_position_y)/(i-bot_position_x))*180/PI; 
//      }
  
//  }

    float X, Y, ang;
  for(int i=0;i<20;i++){
    for(int j=0;j<20;j++){
      Y = i - bot_position_x;
      X = j - bot_position_y;
      if(Y == 0)
        ang = 0;                //handle 0/0 types
      else
        ang=abs(atan(Y/X)*180/PI); 
      
      if(X < 0){
        if(Y < 0) 
          ang = 180 - ang;       //2nd quadrant
        else 
          ang = 180 + ang;       //3rd quadrant
      }else{
        if(Y > 0) 
          ang = 360 - ang;       //4th quadrant
      }
      direction[i][j] = ang;
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
      directionMatrix();
      //printDirectionMatrix();
      magnitudeMatrix();
      //printMagnitudeMatrix();
      findPolarObstacleDensity();
      //printPolarObstacleDensity();

}



   //Initializing the positions and z axis orientation

   
void odomCallback(const nav_msgs :: Odometry::ConstPtr& odom){
    //rostopic show Odometry

  x=odom->pose.pose.position.x;
  y=odom->pose.pose.position.y;


  //curr_x=x;
  //curr_y=y;
 bot_position_x=findx(x,y);
  bot_position_y=findy(x,y);

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

       std::cout<<"ENTER THE coordinates x and y:";
    std::cin>>dest_x;
    std::cin>>dest_y;



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
/*
 while(ros::ok()) {

            //double inc_x=goal.x-x;
            //double inc_y=goal.y-y;
            
            
            //double angle_to_goal=atan2(inc_y,inc_x);        //atan2 returns in radians and it is inverse tan.


            double theta_dest = findTheta()*PI/180;
            double diff = 0.0;
            std::cout<<"CHECK\n\n";
        if(theta_dest==-1){
          std::cout<<"Returned -1. Cannot turn!!\n";
        }
        else if(theta_dest > PI){
          theta_dest = 2*PI - theta_dest; 
          diff = -theta_dest - theta;
        }else{
          diff = theta_dest - theta;
        }
        std::cout<<"\ntheta="<<theta_dest<<" && "<<theta<<" => "<<(diff)*180/PI<<"\n";

            if (diff > 0.05){          //if the vehicle is not facing to goal then change angle
                deep.linear.x=0.0;
                deep.angular.z=0.3;
            }

            else{                                           //if vehicle is facing to goal then go straight and dont change angle
                deep.linear.x=5;
                deep.angular.z=0.0;

            }

             // if(cnt%10!=0){
             //     deep.linear.x=2.0;
             //     cnt++;
             // }

             if((int)(bot_position_x) == (int)(dest_x) && (int)(bot_position_y) == (int)(dest_y)){
              std::cout<<"REACHED!!!\n\n";
              break;
             }

            //Publish the message
            pub.publish(deep);
            //Delays until it is time to send another message
            rate.sleep();
    ros::spinOnce();


        }*/


}