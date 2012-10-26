/*
 * water_simulation.cpp
 *
 *  Created on: Jun 9, 2012
 *      Author: Nikolas Engelhard
 */

#include "water_simulation/water_simulation.h"
using namespace std;

// TODO: return organized cloud
Cloud Water_Simulator::img2Cloud(const cv::Mat& img, cv::Scalar color,bool ignore_zero){
 Cloud c;
 c.reserve(img.cols*img.rows);

 pcl_Point p;
 p.r = color.val[0];
 p.g = color.val[1];
 p.b = color.val[2];


 for (int i=0; i<img.cols; ++i){
  p.x = i*px2m_scale;
  for (int j=0; j<img.rows; ++j){
   p.y = j*px2m_scale;
   p.z = img.at<double>(j,i);
   if (ignore_zero && p.z == 0) continue;
   c.push_back(p);
  }

 }

 return c;

}


/**
*
* @param threshold  Cells with less water are returned as zeros
*/
/// Returns water depth after thresholding
cv::Mat Water_Simulator::getWaterImage(float threshold){
// cv::Mat thres;
// cv::Mat cpy;
// water_depth.convertTo(cpy, CV_32FC1,1);
// cv::threshold(cpy,thres, threshold,1,CV_THRESH_TOZERO);
// thres.convertTo(thres, CV_64FC1,1);
// return thres;

 return water_depth;
}



/**
* @param land new surface
*/
/// Setting a new surface (e.g. after user has changed it)
void Water_Simulator::updateScene(const cv::Mat& land){

 land_height = land;

 land_cloud = img2Cloud(land_height,cv::Scalar(255,0,0));
 land_cloud_msg = land_cloud.makeShared();
 land_cloud_msg->header.frame_id = "/fixed_frame";
}


/**
*
* Intialization of scene.
*
* @todo make it work with negative z-values for land!
*
* @param land definition of ground level at each position. Expected to be 64FC1 image with non-negative values
* @param viscosity viscosity of water used in simulation. (0 < v <= 1)
*/
void Water_Simulator::initializeSimulation(const cv::Mat& land, float viscosity){

 viscosity_ = viscosity;

 if (!(0<viscosity_ && viscosity_ <= 1)){
  ROS_FATAL("Water_Simulator::setScene: viscosity has to be in (0,1] !");
  assert(0 <= viscosity_ && viscosity_ <= 1);
 }


 // double min_,max_;
 // cv::minMaxLoc(land, &min_, &max_);
 // assert(min_>0);

 updateScene(land);

 water_depth = cv::Mat(land.rows,land.cols,CV_64FC1);
 flow = cv::Mat(land.rows,land.cols,CV_64FC1);
 water_depth.setTo(0);
 flow.setTo(0);

}



/**
* Returns current water level as unorganized pointcloud. If a cell contains more than
* 0.002 water, a point in the middle of the cell and with the z-value of the current waterlevel is added to
* the resulting point cloud
*
* @todo 0.002 as parameter
* @todo return water as organized cloud to simplify meshing
* @return water level as pointcloud
* @see px2m_scale
*
*/
Cloud Water_Simulator::getWaterCloud(){
 Cloud c;
 c.reserve(water_depth.cols*water_depth.rows);

 pcl_Point p;
 p.r = 0;
 p.g = 0;
 p.b = 255;


 // contains thresholding
 cv::Mat depth = getWaterImage();

 for (int i=0; i<depth.cols; ++i){
  p.x = i*px2m_scale;
  for (int j=0; j<depth.rows; ++j){

   p.y = j*px2m_scale;
   p.z = depth.at<double>(j,i);
   p.z += land_height.at<double>(j,i);

   c.push_back(p);
  }
 }

 return c;
}


/**
* Sends visualization to RVIZ in frame /fixed_frame (if someone is listening)
*
* Water on topic "simulator_water"
* Surface on topic "simulator_land"
*/
void Water_Simulator::sendCloudVisualization(){


 if (pub_land.getNumSubscribers()>0){
  land_cloud_msg->header.stamp = ros::Time::now ();
  pub_land.publish(land_cloud_msg);
 }


 if (pub_water.getNumSubscribers()>0){
  Cloud::Ptr msg = getWaterCloud().makeShared();
  msg->header.frame_id = "/fixed_frame";
  msg->header.stamp = ros::Time::now ();
  pub_water.publish(msg);
 }

 // cv::Mat total = land_height+water_depth;
 //
 // if (pub_sum.getNumSubscribers()>0){
 // Cloud total_c = img2Cloud(total, cv::Scalar(0,0,255));
 // msg = total_c.makeShared();
 // msg->header.frame_id = "/fixed_frame";
 // msg->header.stamp = ros::Time::now ();
 // pub_sum.publish(msg);
 // }

}

///**
// * Sets new land height (e.g. after user changed the surface) without reseting the water height
// *
// * @param new_height  new landscape
// */
//void Water_Simulator::updateLand(const cv::Mat& new_height){
// assert(new_height.cols == land_height.cols && new_height.rows == land_height.rows);
// land_height = new_height;
//}


void Water_Simulator::updateLandHeight(const Cloud& cloud, const cv::Mat& mask){

 assert(int(cloud.width) == land_height.cols && int(cloud.height) == land_height.rows);

 cloud_ = cloud;
 mask_ = mask;

 for (uint x=0; x<cloud.width; ++x)
  for (uint y=0; y<cloud.height; ++y){
   pcl_Point p = cloud.at(x,y);
   if (!(p.x == p.x)) continue; // don't change depth if none was measured (is intially zero)
   land_height.at<double>(y,x) = p.z;
  }

}

/**
* Visualization of Water and Land as Debug info using cv::namedwindows ("landHeight" and "waterdepth")
*/
void Water_Simulator::showWaterImages(){

 cv::namedWindow("landHeight", 1);
 cv::namedWindow("waterDepth", 1);
 // cv::namedWindow("sum", 1);

 //
 double max_landheight = 0.3;
 // double max_waterdepth = 0.2;

 double foo = 0;
 for (int x=1; x<water_depth.cols-1; ++x)
  for (int y=1; y<water_depth.rows-1; ++y){
   foo = std::max(foo, water_depth.at<double>(y,x));
  }

 //  ROS_INFO("max val: %f", foo);


 cv::Mat land_scaled = land_height/max_landheight;
 cv::Mat water_scaled = water_depth/foo;

 water_scaled.setTo(0);
 for (int x=1; x<water_depth.cols-1; ++x)
  for (int y=1; y<water_depth.rows-1; ++y){
   if( water_depth.at<double>(y,x) > 0){
    water_scaled.at<double>(y,x) = 1;
    //    water_depth.at<double>(y,x) = 1;
   }
  }


 // double val = water_scaled.at<double>(480/2,640/2);
 // ROS_INFO("val: %f", val);
 // cv::Mat sum = (land_height+water_depth)/(max_landheight+max_waterdepth);

 cv::imshow("landHeight", land_scaled);
 cv::imshow("waterDepth", water_scaled);

 cv::waitKey(10);
 //  cv::imshow("sum", sum);

}



/**
* Computation of one iteration of the simulation
*
* @todo if land is below zero, nothing happens!!
*
* @see dry_border, initializeSimulation
* @todo let water percolate
*/
void Water_Simulator::iterate(){

// ros::Time start= ros::Time::now();

 assert(flow.size == water_depth.size);
 assert(land_height.size == water_depth.size);


 flow.setTo(0);

 double total_water = 0;
 int cells_used = 0;

 float heights[4]; // top, bottom, left, right

 sum_ = water_depth+land_height;


 double out_flow = 0;
 double absorbed = 0;


 for (int x=0; x<water_depth.cols;x++){
  for (int y=0; y<water_depth.rows; y++){

   // border conditions
   if (x==0 || y == 0 || x == water_depth.cols-1 || y == water_depth.rows-1){
    //water_depth.at<double>(y,x) = min(0.5,water_depth.at<double>(y,x)); // wall

    if (dry_border){ // absorbing border
     absorbed += water_depth.at<double>(y,x);
     water_depth.at<double>(y,x) = 0;
    }

    continue;
   }

   // if (mask_.at<uchar>(y,x) == 0) continue;

   double water = water_depth.at<double>(y,x);

   // cell with less than half a mm of water is ignored
   // TODO: let water percolate
   // if (water <= 0.005) continue;

   double stone = land_height.at<double>(y,x);

   cells_used++;
   total_water+=water;


   double max_height = water+stone;
   double mean = 0;
   double lower_cell_cnt = 0;


   double dist_sum = 0;

   heights[0] = sum_.at<double>(y-1,x);
   heights[1] = sum_.at<double>(y+1,x);
   heights[2] = sum_.at<double>(y,x-1);
   heights[3] = sum_.at<double>(y,x+1);

   for (uint i=0; i<4; ++i)
    {

    double height = heights[i];

    // no flow to higher cells
    if (height >= max_height) {
     heights[i] = -1;
     continue;
    }


    // computation of average height-distance of surrouding cells
    dist_sum += (max_height-height);

    // average height of surrounding cells
    mean += height;

    lower_cell_cnt++;
    }

   if (lower_cell_cnt==0) continue;
   mean /= lower_cell_cnt;

   // wie viel Wasser kann abfliessen?
   double mass = std::min((max_height-mean)*viscosity_,water);

   flow.at<double>(y,x) -= mass;

   out_flow +=mass;


//   // Verteilung der Masse auf die Nachbarn
//   for (uint i=0; i<4; ++i)
//    {
//
//    float height = heights[i];
//
//    if (height<0) continue;
//
//    // wassermenge haengt von relativer Hoehe ab
//    double flow_ = (max_height-height)/dist_sum*mass;
//
//    switch(i)
//    {
//    case 0: flow.at<double>(y-1,x) += flow_; break;
//    case 1: flow.at<double>(y+1,x) += flow_; break;
//    case 2: flow.at<double>(y,x-1) += flow_; break;
//    case 3: flow.at<double>(y,x+1) += flow_; break;
//    }
//    }

   // unrolled loop
   if (heights[0]>0){  flow.at<double>(y-1,x) += (max_height-heights[0])/dist_sum*mass; }
   if (heights[1]>0){  flow.at<double>(y+1,x) += (max_height-heights[1])/dist_sum*mass; }
   if (heights[2]>0){  flow.at<double>(y,x-1) += (max_height-heights[2])/dist_sum*mass; }
   if (heights[3]>0){  flow.at<double>(y,x+1) += (max_height-heights[3])/dist_sum*mass; }


  }
 }



 // ROS_INFO("Flow: %f, absorbed: %f", out_flow, absorbed);


 // Show flow:

 // double min_val, max_val;
 // cv::minMaxLoc(flow, &min_val,&max_val, NULL, NULL);
 // cv::namedWindow("flow");
 // cv::imshow("flow", flow/max_val);
 // cv::waitKey(1);


 water_depth += flow;

 // ROS_INFO("Used %i cells (%.1f%%)",cells_used, cells_used*100.0/(water_depth.cols*water_depth.rows));

 // water_depth -= 0.00001;



 // double dt_per_k_cells = dt.toNSec()*1.0/(water_depth.cols*water_depth.rows);
 // ROS_INFO("total for %i cells: %f ms, %f mys per 1000 cells",dt_per_k_cells,water_depth.cols*water_depth.rows,dt.toNSec()/1000.0/1000.0);




}



void Water_Simulator::createSimData(){

 float max_height = 0.3;

 for (int x=1; x<land_height.cols-1; ++x)
  for (int y=1; y<land_height.rows-1; ++y){
   if (x<land_height.cols/2)
    land_height.at<double>(y,x) =max_height- x*2.0/land_height.cols*max_height;
   else
    land_height.at<double>(y,x) = (x-land_height.cols/2)*2.0/land_height.cols*max_height;

  }


 water_depth.setTo(0);
 setWaterHeight(0.1,5, 640/2,480/2);

 // cv::namedWindow("land");
 // cv::imshow("land", land_height/max_height);
 // cv::waitKey(-1);

}




/**
*
* Setting the waterheight at a circular patch. This can be used to simulate sources or sinks
*
* @param new_height new waterheight. (zero for sinks, large for sources)
* @param radius radius of patch
* @param x center of patch
* @param y center of patch
* @todo additive source (adds the height to the current water level, otherwise a fixed height can first be
* a source and then turn into a sink. (which could be interesting to define a groundwater level
*/
void Water_Simulator::setWaterHeight(double new_height, float radius,  int x, int y){
 assert(new_height>=0);

 cv::circle(water_depth, cv::Point(x,y),radius, cv::Scalar::all(new_height),-1);

 //   water_depth.at<double>(y,x) = new_height;
}



