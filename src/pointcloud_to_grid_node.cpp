// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// PCL (kept only if you use crop_box elsewhere)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// From this ROS package
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
#include <pointcloud_to_grid/value_reader.hpp>
#include <pointcloud_to_grid/grid_processing.hpp>
// c++
#include <cstdint>
#include <limits>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>


using pointcloud_to_grid::RetainedGrid;
using pointcloud_to_grid::cloudValidationError;
using pointcloud_to_grid::cloudTimeNs;
using pointcloud_to_grid::accumulatePoints;
using pointcloud_to_grid::fillOccupancyGrid;
using pointcloud_to_grid::fillGridMap;
using pointcloud_to_grid::ValueSource; 
using pointcloud_to_grid::ValueReader;

auto height_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
auto intensity_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
auto height_gridmap = std::make_shared<grid_map_msgs::msg::GridMap>();
auto intensity_gridmap = std::make_shared<grid_map_msgs::msg::GridMap>();

class PointCloudToGrid : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "mapi_topic_name")
      {
        grid_map.mapi_topic_name = param.as_string();
        pub_igrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.mapi_topic_name, 10);
      }
      if (param.get_name() == "maph_topic_name")
      {
        grid_map.maph_topic_name = param.as_string();
        pub_hgrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.maph_topic_name, 10);
      }
      if (param.get_name() == "mapi_gridmap_topic_name")
      {
        grid_map.mapi_gridmap_topic_name = param.as_string();
        pub_igridmap = this->create_publisher<grid_map_msgs::msg::GridMap>(grid_map.mapi_gridmap_topic_name, 10);
      }
      if (param.get_name() == "maph_gridmap_topic_name")
      {
        grid_map.maph_gridmap_topic_name = param.as_string();
        pub_hgridmap = this->create_publisher<grid_map_msgs::msg::GridMap>(grid_map.maph_gridmap_topic_name, 10);
      }
      if (param.get_name() == "cloud_in_topic")
      {
        cloud_in_topic = param.as_string();
        auto sensor_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in_topic, sensor_qos,
            std::bind(&PointCloudToGrid::pointCloudCallback, this, std::placeholders::_1));
      }
      if (param.get_name() == "cell_size")
      {
        grid_map.cell_size = param.as_double();
      }
      if (param.get_name() == "position_x")
      {
        grid_map.position_x = param.as_double();
      }
      if (param.get_name() == "position_y")
      {
        grid_map.position_y = param.as_double();
      }
      if (param.get_name() == "length_x")
      {
        grid_map.length_x = param.as_double();
      }
      if (param.get_name() == "length_y")
      {
        grid_map.length_y = param.as_double();
      }
      if (param.get_name() == "intensity_factor")
      {
        grid_map.intensity_factor = param.as_double();
      }
      if (param.get_name() == "height_factor")
      {
        grid_map.height_factor = param.as_double();
      }
      if (param.get_name() == "value_field")
      {
        value_field = param.as_string();
      }
      if (param.get_name() == "verbose1")
      {
        verbose1 = param.as_bool();
      }
      if (param.get_name() == "verbose2")
      {
        verbose2 = param.as_bool();
      }
      if (param.get_name() == "retention_time")
      {
        if (param.as_double() < 0.0)
        {
          result.successful = false;
          result.reason = "retention_time must be >= 0";
        }
        else
        {
          retention_time_ = param.as_double();
        }
      }
      grid_map.paramRefresh();
      // grid may have moved or resized: forget retained cells
      retained_.clear();
    }
    return result;
  }

public:
  PointCloudToGrid() : Node("pointcloud_to_grid_node"), count_(0)
  {
    this->declare_parameter<std::string>("mapi_topic_name", "intensity_grid");
    this->declare_parameter<std::string>("maph_topic_name", "height_grid");
    this->declare_parameter<std::string>("mapi_gridmap_topic_name", "intensity_gridmap");
    this->declare_parameter<std::string>("maph_gridmap_topic_name", "height_gridmap");
    this->declare_parameter<std::string>("cloud_in_topic", cloud_in_topic);
    this->declare_parameter<std::string>("value_field", "intensity");
    this->declare_parameter<float>("cell_size", 0.5);
    this->declare_parameter<float>("position_x", 0.0);
    this->declare_parameter<float>("position_y", 0.0);
    this->declare_parameter<float>("length_x", 20.0);
    this->declare_parameter<float>("length_y", 30.0);
    this->declare_parameter<float>("intensity_factor", 1.0);
    this->declare_parameter<float>("height_factor", 1.0);
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
    this->declare_parameter<double>("retention_time", 0.0);

    this->get_parameter("mapi_topic_name", grid_map.mapi_topic_name);
    this->get_parameter("maph_topic_name", grid_map.maph_topic_name);
    this->get_parameter("mapi_gridmap_topic_name", grid_map.mapi_gridmap_topic_name);
    this->get_parameter("maph_gridmap_topic_name", grid_map.maph_gridmap_topic_name);
    this->get_parameter("cloud_in_topic", cloud_in_topic);
    this->get_parameter("value_field", value_field);
    this->get_parameter("cell_size", grid_map.cell_size);
    this->get_parameter("position_x", grid_map.position_x);
    this->get_parameter("position_y", grid_map.position_y);
    this->get_parameter("length_x", grid_map.length_x);
    this->get_parameter("length_y", grid_map.length_y);
    this->get_parameter("intensity_factor", grid_map.intensity_factor);
    this->get_parameter("height_factor", grid_map.height_factor);
    this->get_parameter("verbose1", verbose1);
    this->get_parameter("verbose2", verbose2);
    this->get_parameter("retention_time", retention_time_);
    
    grid_map.paramRefresh();

    pub_igrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.mapi_topic_name, 10);
    pub_hgrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map.maph_topic_name, 10);
    pub_igridmap = this->create_publisher<grid_map_msgs::msg::GridMap>(grid_map.mapi_gridmap_topic_name, 10);
    pub_hgridmap = this->create_publisher<grid_map_msgs::msg::GridMap>(grid_map.maph_gridmap_topic_name, 10);
    retention_time_ = std::max(0.0, retention_time_);

    auto sensor_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_in_topic, sensor_qos,
        std::bind(&PointCloudToGrid::pointCloudCallback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PointCloudToGrid::parametersCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(this->get_logger(), "pointcloud_to_grid_node has been started.");
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribing to: " << cloud_in_topic.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "Value field for intensity grid: " << value_field.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to: " << grid_map.mapi_topic_name.c_str() << " and " << grid_map.maph_topic_name.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing GridMaps to: " << grid_map.mapi_gridmap_topic_name.c_str() << " and " << grid_map.maph_gridmap_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Retention time is set to:  %.2f s", retention_time_);
  }

private:

  // ---------- grid messages and publishing ----------
  void initGridMessages()
  {
    grid_map.initGrid(intensity_grid);
    grid_map.initGrid(height_grid);
    grid_map.initGridMap(intensity_gridmap, "intensity");
    grid_map.initGridMap(height_gridmap, "height");
  }

  void publishGrids(const std::string &frame_id)
  {
    const builtin_interfaces::msg::Time stamp = this->now();
    fillOccupancyGrid(*intensity_grid, retained_.intensity(), frame_id, stamp);
    fillOccupancyGrid(*height_grid, retained_.height(), frame_id, stamp);
    fillGridMap(*intensity_gridmap, retained_.intensity(), frame_id, stamp);
    fillGridMap(*height_gridmap, retained_.height(), frame_id, stamp);

    pub_hgrid->publish(*height_grid);
    pub_igrid->publish(*intensity_grid);
    pub_hgridmap->publish(*height_gridmap);
    pub_igridmap->publish(*intensity_gridmap);

    if (verbose1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Published " << grid_map.mapi_topic_name.c_str() << " and " << grid_map.maph_topic_name.c_str());
    }
  }

  void
  pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    const ValueSource source(value_field);

    const std::string error = cloudValidationError(*input_msg, source);
    if (!error.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "%s Skipping frame.", error.c_str());
      return;
    }
    if (!retained_.prepare(grid_map.cell_num_x, grid_map.cell_num_y)) { return; }

    initGridMessages();

    const int64_t now_ns = cloudTimeNs(*input_msg, this->now().nanoseconds());
    retained_.expire(now_ns, static_cast<int64_t>(retention_time_ * 1e9));
    accumulatePoints(*input_msg, source, grid_map, retained_, now_ns);
    publishGrids(input_msg->header.frame_id);
  }

  // ---------- variables ----------
  //ros
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_igrid, pub_hgrid;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_igridmap, pub_hgridmap;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  
  // grid
  std::string cloud_in_topic = "nonground";
  std::string value_field = "intensity";
  bool verbose1 = true, verbose2 = false;
  GridMap grid_map;
  size_t count_;
  
  //point retention state (persists between clouds) ---
  double retention_time_ = 0.0;                 // seconds; 0 = no retention (old behaviour)
  RetainedGrid retained_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToGrid>());
  rclcpp::shutdown();
  return 0;
}