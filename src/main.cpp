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

#include "water_simulation/water_simulation.h"

#include "water_simulation/simulator_init.h"
#include "water_simulation/simulator_step.h"
#include "water_simulation/msg_source_sink.h"


using namespace std;

Water_Simulator* simulator;

namespace enc = sensor_msgs::image_encodings;

bool init_watersimulation(water_simulation::simulator_init::Request &req, water_simulation::simulator_init::Response &res){


 ROS_INFO("init was called with id %i", req.id);


 cv_bridge::CvImagePtr cv_ptr;
 cv_ptr = cv_bridge::toCvCopy(req.land_img, enc::TYPE_64FC1);

 simulator->dry_border = req.add_sink_border;

 simulator->setScene(cv_ptr->image, req.viscosity);
 simulator->land_id = req.id;

 cv::imwrite("land_init.png", cv_ptr->image);

 res.id_cpy = simulator->land_id;
 return true;

}

bool step_watersimulation(water_simulation::simulator_step::Request &req, water_simulation::simulator_step::Response & res){

 if (req.id != simulator->land_id){
  res.valid_id = false;
  ROS_WARN("Simulator was called with wrong land_id");
  return true;
 }

// ROS_INFO("Simulating %i steps", req.iteration_cnt);

 for (int iter = 0; iter < req.iteration_cnt; ++iter){
  // water: (new height can also be zero (sink area)
  for (uint i=0; i< req.sources_sinks.size(); ++i){
   water_simulation::msg_source_sink water = req.sources_sinks.at(i);
   simulator->setWaterHeight(water.height, water.radius, water.x, water.y);
  }
  simulator->iterate();
 }

 cv_bridge::CvImage out_msg;
 out_msg.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
 out_msg.image    = simulator->getWaterImage();

 res.valid_id = true;
 res.water_img = *out_msg.toImageMsg();

 simulator->sendCloudVisualization();

// simulator->showWaterImages();

 return true;
}







int main(int argc, char ** argv){

 ros::init(argc, argv,"simulation");

 simulator = new Water_Simulator();


 if (argc == 1){


  cv::namedWindow("water");



  ROS_INFO("Water simulation started as Service!");

  ros::NodeHandle n;
  ros::ServiceServer service_init = n.advertiseService("srv_simulator_init", init_watersimulation);
  ros::ServiceServer service_step = n.advertiseService("srv_simulator_step", step_watersimulation);


  ros::spin();
  //  ros::Rate r(10);
  //  while (ros::ok()){
  //   ros::spinOnce()
  //   r.sleep();
  //  }
 } else{



  cv::Mat land;
  float scale = 0.4;

  land = cv::imread(argv[1],0);
  cv::resize(land, land, cv::Size(),scale,scale, CV_INTER_CUBIC);
  land.convertTo(land, CV_64FC1,0.4/255); // conversion from uchar image to meters

  simulator->setScene(land,1);

  //  cv::namedWindow("land",1);
  //  cv::imshow("land", land);
  //  cv::waitKey(0);

  ros::Rate r(10);

  int iteration = 0;

  int iter_per_step = 50;


  float x = (land.cols -   10);
  float y = land.rows/2;

  ROS_INFO("water at %f %f", x, y);

  while (ros::ok()){

   for (int i=0; i<iter_per_step; ++i){
    simulator->setWaterHeight(0.2,5,x,y);
    simulator->iterate();
    iteration++;
   }

   ros::Time start= ros::Time::now();
   simulator->sendCloudVisualization();
   ros::Duration dt = (ros::Time::now()-start);
   //  ROS_INFO("visualization: %.1f ms", dt.toNSec()/1000.0/1000.0);

   r.sleep();
  }

 }


}
