#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/pcl_config.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

typedef struct{
  double min, max, diff, origin_x, origin_y;
} cell_t;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PC_to_dhmap");
  ROS_WARN("activate");
  ros::NodeHandle nh;
  sensor_msgs::PointCloud2 pc_in_;
  sensor_msgs::Image ros_img_;
  double x_max_, x_min_, y_max_, y_min_;
  double resolution_ = 0.05;
  double diff_max_ = 1.0;
  int pc_size_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outrem_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_pc(new pcl::PointCloud<pcl::PointXYZ>);
  //ros::Publisher pub_img_ = nh.advertise<sensor_msgs::Image>("/dhmap", 1);
  //ros::Rate r(0.5);
  
  //if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tera/0.pcd", *pcl_pc_in) == -1){
  if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/akito/pcdfile/autoware-190609.pcd", *pcl_pc_in) == -1){
    ROS_ERROR("Couldn't read pcd file");
    return(-1);
  }
  
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(pcl_pc_in);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius(2);
  //outrem.filter(*outrem_pc);
  outrem.filter(*voxel_pc);
  
  ROS_WARN("outrem");
  
  //pcl::VoxelGrid<pcl::PointXYZ> sor;
  //sor.setInputCloud(outrem_pc);
  //sor.setLeafSize(0.1f, 0.1f, 0.1f);
  //sor.filter(*voxel_pc);
  
  ROS_WARN("voxel");
  //pcl::toROSMsg(*voxel_pc, pc_in_);
  pc_size_ = voxel_pc->points.size();
  x_max_ = voxel_pc->points[0].x;
  x_min_ = voxel_pc->points[0].x;
  y_max_ = voxel_pc->points[0].y;
  y_min_ = voxel_pc->points[0].y;
  for(int i=0; i<pc_size_; i++){
    if(x_max_ < voxel_pc->points[i].x) x_max_ = voxel_pc->points[i].x;
    if(x_min_ > voxel_pc->points[i].x) x_min_ = voxel_pc->points[i].x;
    if(y_max_ < voxel_pc->points[i].y) y_max_ = voxel_pc->points[i].y;
    if(y_min_ > voxel_pc->points[i].y) y_min_ = voxel_pc->points[i].y;
  }
  
  ROS_WARN("for1");
  const int x_cell_size_ = floor( fabs(x_max_ - x_min_) / resolution_);
  const int y_cell_size_ = floor( fabs(y_max_ - y_min_) / resolution_);
  const int map_cell_size_ = x_cell_size_ * y_cell_size_;
  ROS_WARN("x_cell_size_ = %d", x_cell_size_);
  ROS_WARN("y_cell_size_ = %d", y_cell_size_);
  ROS_WARN("x_min_ = %f, x_max_ = %f", x_min_, x_max_);
  ROS_WARN("y_min_ = %f, y_max_ = %f", y_min_, y_max_);
  
  double x_cell_lead = x_min_, x_cell_behind = x_min_ + resolution_;
  double y_cell_lead, y_cell_behind;
  //cell_t (*cell)[y_cell_size_];
  
  cell_t **cell = new cell_t*[x_cell_size_];
  for(int p=0; p<x_cell_size_; p++){
    cell[p] = new cell_t[y_cell_size_];
  }
  
  //cell_t cell[x_cell_size_][y_cell_size_];
  
  ROS_WARN("cell define");
  int j=0, k=0, l=0, cnt=0;

  for(j=0; j<x_cell_size_; j++){
    for(k=0; k<y_cell_size_; k++){
      if(j==0){
        cell[j][k].origin_x = x_min_;
      }else{
        cell[j][k].origin_x = cell[j-1][k].origin_x + resolution_;
      }
      if(k==0){
        cell[j][k].origin_y = y_min_;
      }else{
        cell[j][k].origin_y = cell[j][k-1].origin_y + resolution_;
      }
    }
  }
  
  #pragma omp parallel for private(k, l)
  for(j=0; j<x_cell_size_; j++){
    ROS_INFO("j=%d",j);
    //ROS_INFO("pc_size_ = %d", voxel_pc->points.size());
    for(k=0; k<y_cell_size_; k++){
      //ROS_INFO("k=%d",k);
      cnt = 0;
      //pc_size_ = voxel_pc->points.size();
      for(l=0; l<pc_size_; l++){
        if(cell[j][k].origin_x <= voxel_pc->points[l].x && voxel_pc->points[l].x <= cell[j][k].origin_x+resolution_){ 
          if(cell[j][k].origin_y <= voxel_pc->points[l].y && voxel_pc->points[l].y <= cell[j][k].origin_y+resolution_){
            if(cell[j][k].min > voxel_pc->points[l].z || cnt == 0) cell[j][k].min = voxel_pc->points[l].z;
            if(cell[j][k].max < voxel_pc->points[l].z || cnt == 0) cell[j][k].max = voxel_pc->points[l].z;
            cell[j][k].diff = fabs(cell[j][k].max - cell[j][k].min);
            if(cell[j][k].diff > diff_max_) cell[j][k].diff = diff_max_;
            //ROS_WARN("in if");
            //voxel_pc->points.erase(voxel_pc->points.begin() + l);
            cnt++;
          }
        }
      }
      //ROS_WARN("cell[%d][%d] = %f", j, k, cell[j][k].diff);
      //ROS_WARN("x_cell_lead = %f, x_cell_behind = %f", x_cell_lead, x_cell_behind);
      //ROS_WARN("y_cell_lead = %f, y_cell_behind = %f", y_cell_lead, y_cell_behind);
    }
  }
  ROS_WARN("for");
  ROS_WARN("x_cell_lead = %f, x_cell_behind = %f", x_cell_lead, x_cell_behind);
  ROS_WARN("y_cell_lead = %f, y_cell_behind = %f", y_cell_lead, y_cell_behind);
  
  cv::Mat pc_img;
  pc_img = cv::Mat(x_cell_size_, y_cell_size_, CV_8UC3);
  for(int m=0; m<x_cell_size_; m++){
    for(int n=0; n<y_cell_size_; n++){
      pc_img.at<cv::Vec3b>(m, n)[0] = (255 - floor(255 * cell[m][n].diff)/diff_max_);
      pc_img.at<cv::Vec3b>(m, n)[1] = (255 - floor(255 * cell[m][n].diff)/diff_max_);
      pc_img.at<cv::Vec3b>(m, n)[2] = (255 - floor(255 * cell[m][n].diff)/diff_max_);
    }
  }
  for(int q=0; q<x_cell_size_;q++){
    delete [] cell[q];
  }
  delete [] cell;
  
  ROS_WARN("for3");
  
  //cv_bridge::CvImage out_img;
  //out_img.encoding = "bgr8";
  //out_img.image = pc_img;
  //out_img.toImageMsg(ros_img_);
  //ros_img_.header.frame_id = "/map";
  
  cv::imwrite("/home/akito/dhmap.jpg", pc_img);
    
  //while(ros::ok()){
  //  ROS_WARN("loop");
  //  pub_img_.publish(ros_img_);
  //  
  //  ros::spinOnce();
  //  r.sleep();
  //}
  
  return 0;
}
