#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include<iostream>
#include<fstream>
#include<cmath>
#include<stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include"generate_matrix.h"
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_cost");
  ros::NodeHandle n;
  ros::Publisher matrix_pub = n.advertise<std_msgs::Int32MultiArray>("matrix",1);

  ros::Rate loop_rate(30);
  srand (time(NULL));

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Int32MultiArray array;
    //Clear Array
    array.data.clear();

    int** a = new int*[MATRIX_LENGTH];
    float** matrix = new float*[MATRIX_LENGTH];
    for(int i=0;i<MATRIX_LENGTH;i++){
      a[i] = new int[MATRIX_WIDTH];
      matrix[i] = new float[MATRIX_WIDTH];
    }
    for(int i=0;i<MATRIX_LENGTH;i++){
      for(int j=0;j<MATRIX_WIDTH;j++){
        a[i][j]=0;
        matrix[i][j]=0.0;
      }
    }

    generate_random_binary_matrix(a);
    //weighted_matrix(a,matrix);
    random_lane_costs(a);
    for(int i=0;i<MATRIX_LENGTH;i++){
      for(int j=0;j<MATRIX_WIDTH;j++){
        a[i][j] = a[i][j]+matrix[i][j];
        if(a[i][j]>OBSTACLE_COST){
          a[i][j] = OBSTACLE_COST;
        }
        array.data.push_back(a[i][j]);
      }
    }

    matrix_pub.publish(array);
    ros::spinOnce();
    loop_rate.sleep();
    cout<<"matrix published"<<count<<"\n";
    count++;
  }

  return 0;
}  