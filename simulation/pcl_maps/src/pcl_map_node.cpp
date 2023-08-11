// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

 
using namespace std::chrono_literals;
 
class PointCloudMapper : public rclcpp::Node
{
  public:
    PointCloudMapper(): Node("pcl_map_node"), count_(0)
    {
      time_publisher_ = this->create_publisher<std_msgs::msg::String>("addison",10);
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);

      timer_ = this->create_wall_timer(500ms, std::bind(&PointCloudMapper::timer_callback, this));
      subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar", 
        10, 
        std::bind(&PointCloudMapper::scan_callback, this, std::placeholders::_1)\
      );

    }
 
  private:
    void timer_callback()
    {
//       auto message = std_msgs::msg::String();
//       message.data = "Hi Automatic Addison! " + std::to_string(count_++);
//      RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", message.data.c_str());
//      time_publisher_->publish(message);
    }

    void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {

      auto message = std_msgs::msg::String();
      // message.data = "lidar message received " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", message.data.c_str());
      time_publisher_->publish(message);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg, *cloud);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
      voxelGrid.setInputCloud(cloud);
      voxelGrid.setLeafSize(0.02, 0.02, 0.02);
      voxelGrid.filter(*cloud_filtered);

      // SAC Segmentation
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
      double threshould = 0.05; // 0.01
      seg.setOptimizeCoefficients (true);  
      seg.setModelType (pcl::SACMODEL_PLANE);  
      seg.setMethodType (pcl::SAC_RANSAC);  
      seg.setDistanceThreshold (threshould);  
      seg.setInputCloud (cloud_filtered);  
      seg.segment (*inliers, *coefficients);  

      for (size_t i = 0; i < inliers->indices.size (); ++i) {
        cloud_filtered->points[inliers->indices[i]].r = 255;  
        cloud_filtered->points[inliers->indices[i]].g = 0;  
        cloud_filtered->points[inliers->indices[i]].b = 0;  
      }  

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter (*cloud_filtered);


      // Create the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud (cloud_filtered);

      // create the extraction object for the clusters
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
      // specify euclidean cluster parameters
      ece.setClusterTolerance (0.3); // 30cm earlier 0.02
      ece.setMinClusterSize (20);
      ece.setMaxClusterSize (10000);
      ece.setSearchMethod (tree);
      ece.setInputCloud (cloud_filtered);
      // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
      ece.extract (cluster_indices);
//      if (cluster_indices.size() > 0)
//        RCLCPP_INFO(this->get_logger(), "cluster size (%d)",cluster_indices.size());


      pcl::PCDWriter writer;
      int j = 0;  
      float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
      pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);  
      
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
        {  
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
            cloud_cluster->points[*pit].r = colors[j%6][0];  
            cloud_cluster->points[*pit].g = colors[j%6][1];  
            cloud_cluster->points[*pit].b = colors[j%6][2];  
          }  
          //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//          std::stringstream ss;
//          ss << "cloud_cluster_" << j << ".pcd";
//          writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
          j++;  
        }  

      sensor_msgs::msg::PointCloud2 sensor_msg;
      pcl::toROSMsg(*cloud_cluster, sensor_msg);
      sensor_msg.header.frame_id = "lidar";
      sensor_msg.header.stamp = rclcpp::Node::now();
      publisher_->publish(sensor_msg);
    }
     
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr time_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    size_t count_;
};
 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMapper>());
  rclcpp::shutdown();
  return 0;
}