#include <string> // String functions
#include <iostream>
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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
using std::chrono::system_clock;


class PointCloudMapper : public rclcpp::Node
{
  public:
    //Конструктор, инициализация
    PointCloudMapper(): Node("pcl_map_node"), count_(0)
    {
      this->declare_parameter("config_dir", rclcpp::PARAMETER_STRING);
      rclcpp::Parameter config_dir = this->get_parameter("config_dir"); 
      //Загрузка констант из YAML
      YAML::Node config = YAML::LoadFile(config_dir.as_string() + "/lidardata.yaml");
      front_lidar_x = config["front_lidar_x"].as<double>();
      front_lidar_y = config["front_lidar_y"].as<double>();
      front_lidar_z = config["front_lidar_z"].as<double>();
      rear_lidar_x = config["rear_lidar_x"].as<double>();
      rear_lidar_y = config["rear_lidar_y"].as<double>();
      rear_lidar_z = config["rear_lidar_z"].as<double>();
      filter_leaf_size = config["filter_leaf_size"].as<double>();
      segmentation_threshold = config["segmentation_threshold"].as<double>();
      cluster_tolerance = config["cluster_tolerance"].as<double>();
      min_cluster_size = config["min_cluster_size"].as<int>();
      max_cluster_size = config["max_cluster_size"].as<int>();
      file_record = config["file_record"].as<bool>();
      vehicle_corner1_x = config["vehicle_corner1_x"].as<double>();
      vehicle_corner1_y = config["vehicle_corner1_y"].as<double>();
      vehicle_corner2_x = config["vehicle_corner2_x"].as<double>();
      vehicle_corner2_y = config["vehicle_corner2_y"].as<double>();
      vehicle_corner3_x = config["vehicle_corner3_x"].as<double>();
      vehicle_corner3_y = config["vehicle_corner3_y"].as<double>();
      vehicle_corner4_x = config["vehicle_corner4_x"].as<double>();
      vehicle_corner4_y = config["vehicle_corner4_y"].as<double>();
      front_lidar_x_angle = config["front_lidar_x_angle"].as<double>();
      front_lidar_y_angle = config["front_lidar_y_angle"].as<double>();
      front_lidar_z_angle = config["front_lidar_z_angle"].as<double>();
      rear_lidar_x_angle = config["rear_lidar_x_angle"].as<double>();
      rear_lidar_y_angle = config["rear_lidar_y_angle"].as<double>();
      rear_lidar_z_angle = config["rear_lidar_z_angle"].as<double>();
      max_distance = config["max_distance"].as<double>();
      file_record_frameinterval = config["file_record_frameinterval"].as<int>();
      min_hmax = config["min_hmax"].as<double>();
      max_hmin = config["max_hmin"].as<double>();
      max_hmax = config["max_hmax"].as<double>();
      min_axis_distance = config["min_axis_distance"].as<double>();
      orthogonal_boxes = config["orthogonal_boxes"].as<bool>();
      rear_filter_leaf_size = config["rear_filter_leaf_size"].as<double>();
      rear_segmentation_threshold = config["rear_segmentation_threshold"].as<double>();
      rear_cluster_tolerance = config["rear_cluster_tolerance"].as<double>();
      rear_min_cluster_size = config["rear_min_cluster_size"].as<int>();
      rear_max_cluster_size = config["rear_max_cluster_size"].as<int>();


      //Публикаторы данных о препятствиях
      json_publisher = this->create_publisher<std_msgs::msg::String>("obstacles",10);
      lidar_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        // "lidar",
        "/ch64w/lslidar_point_cloud", 
        10, 
        std::bind(&PointCloudMapper::lidar_callback, this, std::placeholders::_1)\
      );
      auto qos = rclcpp::QoS(1);
      lidar_subscriber_rear = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", 
        qos.best_effort(), 
        std::bind(&PointCloudMapper::lidar_callback_rear_2, this, std::placeholders::_1)\
      );

      if (file_record)
      {
        mkdir("data/lidar_data", 0777);
      }
    }
 
  private:
    
    double front_lidar_x = 0;
    double front_lidar_y = 0;
    double front_lidar_z = 0;
    double rear_lidar_x = 0;
    double rear_lidar_y = 0;
    double rear_lidar_z = 0;
    double filter_leaf_size = 0.2;
    double segmentation_threshold = 0.5;
    double cluster_tolerance = 0.3;
    double front_lidar_x_angle = -5.0;
    double front_lidar_y_angle = 0.0;
    double front_lidar_z_angle = 0.0;
    double rear_lidar_x_angle = -5.0;
    double rear_lidar_y_angle = 0.0;
    double rear_lidar_z_angle = 0.0;
    double vehicle_corner1_x = 0;
    double vehicle_corner1_y = 0;
    double vehicle_corner2_x = 0;
    double vehicle_corner2_y = 0;
    double vehicle_corner3_x = 0;
    double vehicle_corner3_y = 0;
    double vehicle_corner4_x = 0;
    double vehicle_corner4_y = 0;
    double min_axis_distance = 1;
    double max_distance = 20;
    double min_hmax = 1000;
    double max_hmin = 0;
    double max_hmax = 3.5;
    int min_cluster_size = 5;
    int max_cluster_size = 10000;
    bool file_record = false;
    int file_record_frameinterval = 1;
    int recieved_frames = 0;
    bool orthogonal_boxes = false;
    double rear_filter_leaf_size = 0.2;
    double rear_segmentation_threshold = 0.5;
    double rear_cluster_tolerance = 0.3;
    int rear_min_cluster_size = 5;
    int rear_max_cluster_size = 10000;


    //Дистанция между двумя точками
    double calc_distance (double x1, double y1, double x2, double y2) {
      return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    }

    //Минимальное значение из четырех (для вычисления расстояния до ближайшего угла)
    double minval(double v1, double v2, double v3, double v4) {
      double min = v1 < v2 ? v1 : v2;
      min = v3 < min ? v3 : min;
      min = v4 < min ? v4 : min;
      return min;
    }
 
    //Получение имени файла для записи облака точек
    std::string get_new_filename(bool is_rear) {
      system_clock::time_point tp = system_clock::now();
      time_t raw_time = system_clock::to_time_t(tp);
      struct tm  *timeinfo = std::localtime(&raw_time);
      std::stringstream ss;
      if (!is_rear) {
        ss << std::put_time(timeinfo, "data/lidar_data/Lidar_front_%Y-%m-%d_%H_%M_%S_");
      } else {
        ss << std::put_time(timeinfo, "data/lidar_data/Lidar_rear_%Y-%m-%d_%H_%M_%S_");
      }
      std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
      std::string milliseconds_str = std::to_string(ms.count() % 1000);
      if (milliseconds_str.length() < 3) {
        milliseconds_str = std::string(3 - milliseconds_str.length(), '0') + milliseconds_str;
      }
      return ss.str() + milliseconds_str + ".pcd";
    }

    //Создание JSON-сообщения с данными о препятствии по облаку точек
    std_msgs::msg::String make_json_obstacles(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, std::string key, bool is_rear,
                                              double lidar_x, double lidar_y, double lidar_z, double angle_x, double angle_y, double angle_z) {
      auto message = std_msgs::msg::String();
      message.data = "{ \"" + key + "\" : [";
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg, *rawcloud);
      // Поворачиваем всё облако точек в соответствии с параметрами установки лидара
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();//Initialize the transformation matrix as the identity matrix
      transform.rotate(Eigen::AngleAxisf(angle_x * (M_PI / 180), Eigen::Vector3f::UnitX()));
      transform.rotate(Eigen::AngleAxisf(angle_y * (M_PI / 180), Eigen::Vector3f::UnitY()));
      transform.rotate(Eigen::AngleAxisf(angle_z * (M_PI / 180), Eigen::Vector3f::UnitZ()));
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*rawcloud, *cloud, transform);


      //Если нужно, записываем облако точек в файл
      recieved_frames++;
      if ((file_record) && recieved_frames >= file_record_frameinterval) {
        pcl::io::savePCDFileBinary(get_new_filename(is_rear), *cloud);
        recieved_frames = 0;
      }
      
      //RCLCPP_INFO(this->get_logger(), get_new_filename().c_str());



      // Фильтрация облака точек - "разуплотнение"
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
      voxelGrid.setInputCloud(cloud);
      //oxelGrid.setLeafSize(0.02, 0.02, 0.02);
      if (!is_rear) {
        voxelGrid.setLeafSize(filter_leaf_size, filter_leaf_size, filter_leaf_size);
      } else {
        voxelGrid.setLeafSize(rear_filter_leaf_size, rear_filter_leaf_size, rear_filter_leaf_size);
      }
      voxelGrid.filter(*cloud_filtered);

      //  Сегментация
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
      double threshould = is_rear ? rear_segmentation_threshold : segmentation_threshold; // 0.01
      seg.setOptimizeCoefficients (true);  
      seg.setModelType (pcl::SACMODEL_PLANE);  
      seg.setMethodType (pcl::SAC_RANSAC);  
      seg.setDistanceThreshold (threshould);  
      seg.setInputCloud (cloud_filtered);  
      seg.segment (*inliers, *coefficients);  

/*    Красить точки пока не надо
      for (size_t i = 0; i < inliers->indices.size (); ++i) {
        cloud_filtered->points[inliers->indices[i]].r = 255;  
        cloud_filtered->points[inliers->indices[i]].g = 0;  
        cloud_filtered->points[inliers->indices[i]].b = 0;  
      }  
*/
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
      ece.setClusterTolerance (is_rear ? rear_cluster_tolerance : cluster_tolerance); // 30cm earlier 0.02
      ece.setMinClusterSize (is_rear ? rear_min_cluster_size : min_cluster_size);
      ece.setMaxClusterSize (is_rear? rear_max_cluster_size : max_cluster_size);
      ece.setSearchMethod (tree);
      ece.setInputCloud (cloud_filtered);
      // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
      ece.extract (cluster_indices);

      int j = 0;  
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
      pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);  

      bool isFirst = true;  //Для JSON-строки
      pcl::PointXYZRGB zero_point;
      zero_point.x = lidar_x;
      zero_point.y = lidar_y;
      zero_point.z = lidar_z;

      //Построение отдельных облаков точек и их обработка
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
        {  
          double dist = 1000; //maximal obstacle distacne
          //Кластер для горизонтальной проекции, с которым будем работать
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr one_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
          one_cluster->width = cloud_cluster->width;
          one_cluster->height = cloud_cluster->height;
          one_cluster->is_dense = cloud_cluster->is_dense;




          //one_cluster->points.resize(cloud_cluster->width * cloud_cluster->height);
          int size = 0;
          //Формирование облака точек
          
          
          double hmin = 1000;    //Минимальная высота препятствия
          double hmax = -1000;   //Максимальная высота препятствия
          double xmin = 1000;
          double xmax = -1000;
          double ymin = 1000;
          double ymax = -1000;
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
            pcl::PointXYZRGB cluster_point =  cloud_cluster->points[*pit];
            
            if (is_rear) {
              double tmp = cluster_point.y;
              cluster_point.y = cluster_point.x;
              cluster_point.y = tmp;

            } 
            //Сохраняем высоту низа и верха препятствия
            if (cluster_point.z < hmin)
              hmin = cluster_point.z;
            if (cluster_point.z > hmax)
              hmax = cluster_point.z;
            if (cluster_point.x < xmin)
              xmin = cluster_point.x;
            if (cluster_point.x > xmax)
              xmax = cluster_point.x;
            if (cluster_point.y < ymin)
              ymin = cluster_point.y;
            if (cluster_point.y > ymax)
              ymax = cluster_point.y;
            //Роняем точку на нулевую плоскость по Z
            cluster_point.z = zero_point.z;

            one_cluster->push_back(cluster_point);
            if (!is_rear) {
              double pdist1 = calc_distance(cluster_point.x, cluster_point.y, vehicle_corner1_x, vehicle_corner1_y);
              double pdist2 = calc_distance(cluster_point.x, cluster_point.y, vehicle_corner2_x, vehicle_corner2_y);
              double pdist = pdist1 < pdist2 ? pdist1 : pdist2;
              if (pdist < dist) {
                dist = pdist;
              }
            } else {
              double pdist1 = calc_distance(cluster_point.x, cluster_point.y, vehicle_corner3_x, vehicle_corner3_y);
              double pdist2 = calc_distance(cluster_point.x, cluster_point.y, vehicle_corner4_x, vehicle_corner4_y);
              double pdist = pdist1 < pdist2 ? pdist1 : pdist2;
              if (pdist < dist) {
                dist = pdist;
              }
            }
            size++;
          }

          //Если препятствие не проходит под наш фильтр по высоте, идём к следующему, не добавляя этого
          if ((hmax + zero_point.z < min_hmax) || (hmin + zero_point.z > max_hmin) || (hmax + zero_point.z > max_hmax) || dist > max_distance || dist < min_axis_distance)
          {
            continue;
          }
          if (is_rear && xmin < 0) {
            continue;
          }


          //Минимальные и максимальные значения координат
          pcl::PointXYZRGB min_cluster, max_cluster;
	        pcl::getMinMax3D(*one_cluster, min_cluster, max_cluster);

          //Построение BoundingBox
          Eigen::Vector4f pcaCentroid;
	        pcl::compute3DCentroid(*one_cluster, pcaCentroid);
	        Eigen::Matrix3f covariance;
	        pcl::computeCovarianceMatrixNormalized(*one_cluster, pcaCentroid, covariance);
	        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	        //Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //correct vertical between main directions
	        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));


	        Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	        Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	        tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	        tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	        tm_inv = tm.inverse();
 
	        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	        pcl::transformPointCloud(*one_cluster, *transformedCloud, tm);
 
	        pcl::PointXYZRGB min_p1, max_p1;
	        Eigen::Vector3f c1, c;
	        pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
          c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
	        Eigen::Affine3f tm_inv_aff(tm_inv);
	        pcl::transformPoint(c1, c, tm_inv_aff);
 
	        Eigen::Vector3f whd, whd1;
	        whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	        whd = whd1;

	        const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	        const Eigen::Vector3f    bboxT(c);

          char val_str[400];
          if (!isFirst) {
           message.data += ", ";
          }

          //Добавление информации о препятствии в JSON
          isFirst = false;

          sprintf(val_str, "[%d, %lf, %lf, %lf, ", j, dist, hmin + zero_point.z, hmax + zero_point.z);
          message.data += val_str;

        
        //Определение координат углов препятствия
          if (orthogonal_boxes) { // Если нужны ортогональные BoundingBox'ы
            if (!is_rear) {
              sprintf(val_str, "[%lf, %lf], [%lf, %lf], [%lf, %lf], [%lf, %lf]", xmin, ymin, xmax, ymin, xmin, ymax, xmax, ymax);
            } else {
              sprintf(val_str, "[%lf, %lf], [%lf, %lf], [%lf, %lf], [%lf, %lf]", xmin * -1, ymin * -1, xmax * -1, 
                                ymin * -1, xmin * -1, ymax * -1, xmax * -1, ymax * -1);
            }
            message.data += val_str;

          } else {
            vtkSmartPointer<vtkDataSet> boundingbox;
            boundingbox = pcl::visualization::createCube(bboxT, bboxQ, whd(0), whd(1), whd(2));
        
            for (int i = 0; i < 4; i++) {
              
              double boxpoints[3];
              boundingbox->GetPoint(i, boxpoints);
              if (is_rear) { //Если лидар смотрит назад, то координаты у точек идут наоборот
                boxpoints[0] *= -1;
                boxpoints[1] *= -1;
              }
              sprintf(val_str, "[%lf, %lf]", boxpoints[0] + lidar_x, boxpoints[1] + lidar_y);
              message.data+=val_str;
              if (i < 3) {
                message.data += ", ";
              }
            }
          }

          sprintf(val_str, ", %lf, %lf, %lf, %lf]", xmin, xmax, ymin, ymax);
          message.data += val_str;
          j++;  
        }  
        message.data += "]}";
        return message;

    }

    // Обработка сообщения с переднего лидара
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
      json_publisher->publish(make_json_obstacles(cloud_msg, "obstacles", false, front_lidar_x, front_lidar_y, front_lidar_z, 
      front_lidar_x_angle, front_lidar_y_angle, front_lidar_z_angle));
    }

    // Обработка сообщения с заднего лидара
    void lidar_callback_rear_2(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
      json_publisher->publish(make_json_obstacles(cloud_msg, "obstacles_rear", true, rear_lidar_x, rear_lidar_y, rear_lidar_z, 
                              rear_lidar_x_angle, rear_lidar_y_angle, rear_lidar_z_angle));
    }


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_rear;
    size_t count_;
};
 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMapper>());
  rclcpp::shutdown();
  return 0;
}