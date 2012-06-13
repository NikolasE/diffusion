/*
 * main.cpp
 *
 *  Created on: Jun 12, 2012
 *      Author: lengelhan
 */


#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <ros/publisher.h>


#include "water_simulation.h"

using namespace std;

Water_Simulator* simulator;


int main(int argc, char ** argv){

 ros::init(argc, argv,"simulation");

 simulator = new Water_Simulator();
 cv::Mat land;

 if (argc > 1)
  land = cv::imread(argv[1],0);
 else
  land = cv::imread("imgs/land_circ.png",0);

 float scale = 0.5;

 cv::resize(land, land, cv::Size(),scale,scale, CV_INTER_CUBIC);

 simulator->setScene(land);

 ros::Rate r(10);

 int iteration = 0;

 int iter_per_step = 100;

 while (ros::ok()){

//  ROS_INFO("iteration %i", iteration);



  for (int i=0; i<iter_per_step; ++i){
   simulator->setWaterHeight(0.2,80*scale,200*scale);
   simulator->flow_stepStone();
   iteration++;
  }





  ros::Time start= ros::Time::now();
  simulator->sendCloudVisualization();
  ros::Duration dt = (ros::Time::now()-start);
  ROS_INFO("visualization: %.1f ms", dt.toNSec()/1000.0/1000.0);



  r.sleep();
 }

 // cv::namedWindow("land");
 // cv::imshow("land", land);
 // cv::waitKey(-1);



}
