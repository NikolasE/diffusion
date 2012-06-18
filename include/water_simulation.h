/*
 * water_simulation.h
 *
 *  Created on: Jun 9, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef WATER_SIMULATION_H_
#define WATER_SIMULATION_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

#include <omp.h>

typedef pcl::Normal Normals;
typedef pcl::PointXYZRGBNormal PointTypeNormal;
typedef pcl::PointXYZRGB pcl_Point;
typedef pcl::PointCloud<pcl_Point> Cloud;

struct Water_Simulator {

 ros::NodeHandle nh;
 ros::Publisher pub_water, pub_land, pub_sum, pub_flow;

 int land_id;


 cv::Mat land_height;
 cv::Mat water_depth;
 cv::Mat flow;
 cv::Mat mask_;

 Cloud::Ptr land_cloud_msg;

 Cloud cloud_;
 Cloud land_cloud;
 cv::Mat sum_;

 float img_to_height_factor;
 float px2m_scale;
 float viscosity_;

 Water_Simulator(){

  img_to_height_factor = 0.2/255; // full white translates to 0.2m

  px2m_scale = 2/640.0; // 4 m length for an VGA-image

  pub_water = nh.advertise<Cloud>("/simulator_water", 1);
  pub_land = nh.advertise<Cloud>("/simulator_land", 10);
  pub_sum = nh.advertise<Cloud>("/simulator_total", 10);


  //land_height = cv::Mat(480,640,CV_64FC1); // height above zero
  //water_depth = cv::Mat(480,640,CV_64FC1); // height above zero
  //dummy = cv::Mat(480,640,CV_64FC1); // used for storage of intermediate values

//  water_depth.setTo(0);
//  land_height.setTo(0);
//  dummy.setTo(0);
 }

// void visualizeWater(cv::Mat& img, cv::Mat P);

// void setScene(const cv::Mat& land, const cv::Mat& water);
 void setScene(const cv::Mat& land, float viscosity = 0.5);

 void sendCloudVisualization();

 void updateScene(const cv::Mat& land);


 void showWaterImages();
 void setWaterHeight(double new_height, float radius, int x, int y);

 void updateLandHeight(const Cloud& cloud, const cv::Mat& mask);

 Cloud projectIntoImage(cv::Mat& img, cv::Mat P);

 void flow_step();

 void flow_stepStone();




 void createSimData();

private:
 Cloud img2Cloud(const cv::Mat& img, cv::Scalar color, bool ignore_zero = false);

 Cloud getWaterCloud();


};





#endif /* WATER_SIMULATION_H_ */
