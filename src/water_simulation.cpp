/*
 * water_simulation.cpp
 *
 *  Created on: Jun 9, 2012
 *      Author: Nikolas Engelhard
 */

#include "water_simulation/water_simulation.h"
using namespace std;

Cloud Water_Simulator::img2Cloud(const cv::Mat& img, cv::Scalar color,bool ignore_zero){
 //ROS_INFO("creating cloud %i %i", img.cols, img.rows);
 Cloud c;
 c.reserve(img.cols*img.rows);

 pcl_Point p;
 p.r = color.val[0];
 p.g = color.val[1];
 p.b = color.val[2];


 for (int i=0; i<img.cols; ++i){
  for (int j=0; j<img.rows; ++j){

   p.x = i*px2m_scale;
   p.y = j*px2m_scale;
   p.z = img.at<double>(j,i);
   if (ignore_zero && p.z == 0) continue;

   c.push_back(p);
  }

 }

 //ROS_INFO("Created cloud");

 return c;

}



void Water_Simulator::updateScene(const cv::Mat& land){

 land_height = land;

 ROS_INFO("Initializing simulation with %i cells", land.rows*land.cols);

 land_cloud = img2Cloud(land_height,cv::Scalar(255,0,0));
 land_cloud_msg = land_cloud.makeShared();
 land_cloud_msg->header.frame_id = "/fixed_frame";
}


void Water_Simulator::setScene(const cv::Mat& land, float viscosity){

 viscosity_ = viscosity;

 if (!(0<viscosity_ && viscosity_ <= 1)){
  ROS_FATAL("Water_Simulator::setScene: viscosity has to be in (0,1] !");
  assert(0<viscosity_ && viscosity_ <= 1);
 }

 updateScene(land);

 water_depth = cv::Mat(land.rows,land.cols,CV_64FC1);
 flow = cv::Mat(land.rows,land.cols,CV_64FC1);
 water_depth.setTo(0);
 flow.setTo(0);

}



Cloud Water_Simulator::getWaterCloud(){
 Cloud c;
 c.reserve(water_depth.cols*water_depth.rows);

 pcl_Point p;
 p.r = 0;
 p.g = 0;
 p.b = 255;


 for (int i=0; i<water_depth.cols; ++i){
  for (int j=0; j<water_depth.rows; ++j){

   p.x = i*px2m_scale;
   p.y = j*px2m_scale;
   p.z = water_depth.at<double>(j,i);
   if (p.z < 0.002) continue;

   p.z += land_height.at<double>(j,i);

   c.push_back(p);
  }
 }

 return c;
}


void Water_Simulator::sendCloudVisualization(){


 if (pub_land.getNumSubscribers()>0){
  // land cloud doesn't change often
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



void Water_Simulator::flow_stepStone(){

 ros::Time start= ros::Time::now();


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
   if (water <= 0.005) continue;

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


   // Verteilung der Masse auf die Nachbarn
   for (uint i=0; i<4; ++i)
    {

    float height = heights[i];

    if (height<0) continue;

    // wassermenge haengt von relativer Hoehe ab
    double flow_ = (max_height-height)/dist_sum*mass;

    switch(i)
    {
    case 0: flow.at<double>(y-1,x) += flow_; break;
    case 1: flow.at<double>(y+1,x) += flow_; break;
    case 2: flow.at<double>(y,x-1) += flow_; break;
    case 3: flow.at<double>(y,x+1) += flow_; break;
    }


    }


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
 ros::Duration dt = (ros::Time::now()-start);

 double dt_per_k_cells = dt.toNSec()*1.0/(water_depth.cols*water_depth.rows);


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

/*
void Water_Simulator::flow_step(){

 double c = 0.9;
 flow.setTo(0);

 // double sum = cv::sum(water_depth).val[0];
 //
 // ROS_WARN("Image contains: %f",sum);

 int water_pixels = 0;

 for (int x=1; x<water_depth.cols-1; ++x)
  for (int y=1; y<water_depth.rows-1; ++y){

   double height = water_depth.at<double>(y,x);

   double mean = 0;
   double weight_sum = 0;
   float weight;

   for (int dx=-1; dx<=1; ++dx)
    for (int dy=-1; dy<=1; ++dy){
     if (dx == 0 && dy == 0) continue;

     if (dx != 0 && dy !=0){
      weight = 1/sqrt(2);
     }else
      weight = 1;

     mean+= weight*water_depth.at<double>(y+dy,x+dx);
     weight_sum += weight;
    }

   mean /= weight_sum;

   //   double h_1 = water_depth.at<double>(y-1,x);
   //   double h_2 = water_depth.at<double>(y+1,x);
   //   double h_3 = water_depth.at<double>(y,x-1);
   //   double h_4 = water_depth.at<double>(y,x+1);
   //
   //   double mean = (height + h_1 + h_2 + h_3 + h_4)/5;

   double diff = height-mean;


   if (height>0){
    water_pixels++;
    //     ROS_INFO("Height: %f, mean: %f, pos: %i %i, new: %f", height, mean, x,y,height-c*diff);
   }


   flow.at<double>(y,x) = c*diff;
  }

 ROS_INFO("Waterpixelcnt: %i", water_pixels);

 water_depth -= flow;

}
*/



/*
Cloud Water_Simulator::projectIntoImage(cv::Mat& img, cv::Mat P){

 Cloud c;

 img.setTo(0);

 for (uint x=0; x<cloud_.width; ++x){
  for (uint y=0; y<cloud_.height; ++y){

   double z = water_depth.at<double>(y,x);

   if (!(z>0)) continue;

   pcl_Point p = cloud_.at(x,y);

   cv::Point2f px;
   applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),P,px);

   int x = px.x; int y = px.y;
   if (x<0 || y<0 || x>=img.cols || y >= img.rows) continue;

   int h = int((z/0.02)*30)%30;
   // ROS_INFO("waterdd z: %f", z);
 //  ROS_INFO("z: %f, col: %i", z,h);

   cv::circle(img, px, 3, cv::Scalar(h,255,255) ,-1);
   //    cv::circle(img, px, 3, cv::Scalar(90,255,255) ,-1);

   // add to cloud:

   p.z -= z;
   c.push_back(p);
  }
 }

 cv::cvtColor(img, img, CV_HSV2BGR);

 // cv::namedWindow("foo");
 // cv::imshow("foo", img);
 // cv::waitKey(10);

 return c;

}

 */

void Water_Simulator::setWaterHeight(double new_height, float radius,  int x, int y){
 assert(new_height>=0);

 cv::circle(water_depth, cv::Point(x,y),radius, cv::Scalar::all(new_height),-1);

 //   water_depth.at<double>(y,x) = new_height;
}



