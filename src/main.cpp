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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "water_simulation.h"

#include "water_simulation/simulator_init.h"

using namespace std;

Water_Simulator* simulator;

namespace enc = sensor_msgs::image_encodings;

bool init_watersimulation(water_simulation::simulator_init::Request &req, water_simulation::simulator_init::Response &res){


 cv_bridge::CvImagePtr cv_ptr;
 cv_ptr = cv_bridge::toCvCopy(req.land_img, enc::MONO8);

 simulator->setScene(cv_ptr->image, req.viscosity);

 return true;

}







int main(int argc, char ** argv){

 ros::init(argc, argv,"simulation");

 simulator = new Water_Simulator();
 cv::Mat land;

 if (argc > 1)
  land = cv::imread(argv[1],0);
 else{
  ROS_INFO("Water simulation started as Service!");





 }


 float scale = 0.4;

 cv::resize(land, land, cv::Size(),scale,scale, CV_INTER_CUBIC);

 simulator->setScene(land);

 ros::Rate r(10);

 int iteration = 0;

 int iter_per_step = 50;


 float x = (land.cols -   10);
 float y = land.rows/2;

 ROS_INFO("water at %f %f", x, y);

 while (ros::ok()){

//  ROS_INFO("iteration %i", iteration);



  for (int i=0; i<iter_per_step; ++i){
   simulator->setWaterHeight(0.2,x,y);
   simulator->flow_stepStone();
   iteration++;
  }





  ros::Time start= ros::Time::now();
  simulator->sendCloudVisualization();
  ros::Duration dt = (ros::Time::now()-start);
//  ROS_INFO("visualization: %.1f ms", dt.toNSec()/1000.0/1000.0);



  r.sleep();
 }

 // cv::namedWindow("land");
 // cv::imshow("land", land);
 // cv::waitKey(-1);



}
