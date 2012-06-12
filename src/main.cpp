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


 cv::Mat land = cv::imread("imgs/maze.png",0);

 cv::resize(land, land, cv::Size(), 0.5,0.5, CV_INTER_CUBIC);

 simulator->setScene(land);

 ros::Rate r(10);

 int iteration =0;

 while (ros::ok()){

  ROS_INFO("iteration %i", iteration);

  for (uint i=0; i<60; ++i){
   simulator->setWaterHeight(0.2,80*0.5,200*0.5);
   simulator->flow_stepStone();
   iteration++;
  }
  simulator->sendCloudVisualization();




  r.sleep();
 }

// cv::namedWindow("land");
// cv::imshow("land", land);
// cv::waitKey(-1);



}
