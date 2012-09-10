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

//#include <omp.h>

typedef pcl::Normal Normals;
typedef pcl::PointXYZRGBNormal PointTypeNormal;
typedef pcl::PointXYZRGB pcl_Point;
typedef pcl::PointCloud<pcl_Point> Cloud;

struct Water_Simulator {


public:



 Cloud getWaterCloud();

 /** ID to identity if iteration is requested for the current scene */
 int land_id;

 /** true iff water at the outmost cells should vanish in each timestep */
 bool dry_border;


 Water_Simulator(){

  px2m_scale = 2/640.0; // 4 m length for a VGA-image

  pub_water = nh.advertise<Cloud>("/simulator_water", 1);
  pub_land = nh.advertise<Cloud>("/simulator_land", 10);
 }


 void setScene(const cv::Mat& land, float viscosity = 0.5);

 void sendCloudVisualization();



 void showWaterImages();
 void setWaterHeight(double new_height, float radius, int x, int y);

 void updateLandHeight(const Cloud& cloud, const cv::Mat& mask);

 Cloud projectIntoImage(cv::Mat& img, cv::Mat P);

// void flow_step();

 void iterate();

 cv::Mat getWaterImage(){return water_depth;}


private:

 ros::NodeHandle nh;
 ros::Publisher pub_water, pub_land, pub_sum, pub_flow;


 Cloud::Ptr land_cloud_msg;

 Cloud cloud_;
 Cloud land_cloud;
 cv::Mat sum_;



 cv::Mat land_height;
 cv::Mat water_depth;
 cv::Mat flow;
 cv::Mat mask_;


 Cloud img2Cloud(const cv::Mat& img, cv::Scalar color, bool ignore_zero = false);
 void createSimData();

 void updateScene(const cv::Mat& land);
 float viscosity_;

 /** Scaling from image to cloud to show simulation on RVIZ */
 float px2m_scale;



};





#endif /* WATER_SIMULATION_H_ */
