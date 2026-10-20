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
            std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
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
      ipoints_.clear();   // grid may have moved or resized: forget retained cells
      hpoints_.clear();
      cell_stamp_ns_.clear();
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
        std::bind(&PointCloudToGrid::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PointCloudToGrid::parametersCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(this->get_logger(), "pointcloud_to_grid_node has been started.");
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribing to: " << cloud_in_topic.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "Value field for intensity grid: " << value_field.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to: " << grid_map.mapi_topic_name.c_str() << " and " << grid_map.maph_topic_name.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing GridMaps to: " << grid_map.mapi_gridmap_topic_name.c_str() << " and " << grid_map.maph_gridmap_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Retention time is set to:  %.2f s", retention_time_);
  }

private:
   // ---------- small stateless helpers ----------
  static bool hasField(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name)
  {
    return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                       [&name](const sensor_msgs::msg::PointField &f) { return f.name == name; });
  }

  static std::string listFields(const sensor_msgs::msg::PointCloud2 &cloud)
  {
    std::string out;
    for (const auto &f : cloud.fields)
    {
      out += (out.empty() ? "" : ", ") + f.name;
    }
    return out;
  }

  // ---------- validation and time ----------
  bool validateCloud(const sensor_msgs::msg::PointCloud2 &cloud, ValueSource source) const
  {
    if (!hasField(cloud, "x") || !hasField(cloud, "y"))
    {
      RCLCPP_ERROR(this->get_logger(), "Incoming PointCloud2 is missing 'x' or 'y' field. Skipping frame.");
      return false;
    }
    if (!hasField(cloud, source.physicalFieldName()))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Requested value_field '%s' not found in incoming PointCloud2. "
                   "Available fields: [%s]. Skipping frame.",
                   value_field.c_str(), listFields(cloud).c_str());
      return false;
    }
    return true;
  }

  // Message stamp in ns, falling back to the node clock if the stamp is unset
  int64_t cloudTimeNs(const sensor_msgs::msg::PointCloud2 &cloud) const
  {
    rclcpp::Time stamp(cloud.header.stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() == 0)
    {
      stamp = this->now();
    }
    return stamp.nanoseconds();
  }

  // ---------- retention state ----------
  // Returns false if the grid has no cells (invalid size/cell_size parameters)
  bool prepareRetentionState()
  {
    if (grid_map.cell_num_x <= 0 || grid_map.cell_num_y <= 0)
    {
      return false;
    }
    const size_t n_cells = static_cast<size_t>(grid_map.cell_num_x) * grid_map.cell_num_y;
    if (ipoints_.size() != n_cells)
    {
      ipoints_.assign(n_cells, -128);
      hpoints_.assign(n_cells, -128);
      cell_stamp_ns_.assign(n_cells, kNoStamp);
    }
    return true;
  }

  // With retention_time_ == 0 every cell expires, which reproduces the
  // previous behaviour. age < 0 means time jumped backwards (e.g. bag loop).
  void expireCells(int64_t now_ns)
  {
    const int64_t retention_ns = static_cast<int64_t>(retention_time_ * 1e9);
    for (size_t i = 0; i < cell_stamp_ns_.size(); ++i)
    {
      if (cell_stamp_ns_[i] == kNoStamp)
      {
        continue;
      }
      const int64_t age_ns = now_ns - cell_stamp_ns_[i];
      if (age_ns < 0 || age_ns >= retention_ns)
      {
        ipoints_[i] = -128;
        hpoints_[i] = -128;
        cell_stamp_ns_[i] = kNoStamp;
      }
    }
  }

  // ---------- point accumulation ----------
  // True if (x, y) lies inside the grid area and not on the x ~ 0 axis
  bool isInsideGrid(float x, float y) const
  {
    if (std::abs(x) <= 0.01f)
    {
      return false;
    }
    return x > grid_map.bottomright_x && x < grid_map.topleft_x &&
           y > grid_map.bottomright_y && y < grid_map.topleft_y;
  }

  void writeCell(const PointXY &cell, float value, int64_t now_ns)
  {
    const size_t idx = static_cast<size_t>(cell.y) * grid_map.cell_num_x + cell.x;
    ipoints_[idx] = value * grid_map.intensity_factor;
    hpoints_[idx] = value * grid_map.height_factor;
    cell_stamp_ns_[idx] = now_ns;
  }

  void accumulatePoints(const sensor_msgs::msg::PointCloud2 &cloud, ValueSource source, int64_t now_ns)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    ValueReader reader(cloud, source);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
    {
      // Always read the value, even if the point is skipped below,
      // so the value iterator stays in step with x and y.
      const float value = reader.next();
      const float px = *iter_x;
      const float py = *iter_y;

      if (!isInsideGrid(px, py))
      {
        continue;
      }

      const PointXY cell = grid_map.getIndex(px, py);
      if (!grid_map.isValidCell(cell))
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Cell out of range: " << cell.x << " - " << grid_map.cell_num_x << " ||| " << cell.y << " - " << grid_map.cell_num_y);
        continue;
      }
      writeCell(cell, value, now_ns);
    }
  }

  // ---------- grid messages and publishing ----------
  void initGridMessages()
  {
    grid_map.initGrid(intensity_grid);
    grid_map.initGrid(height_grid);
    grid_map.initGridMap(intensity_gridmap, "intensity");
    grid_map.initGridMap(height_gridmap, "height");
  }

  void fillOccupancyGrid(nav_msgs::msg::OccupancyGrid &grid,
                         const std::vector<signed char> &cells,
                         const std::string &frame_id)
  {
    grid.header.stamp = this->now();
    grid.header.frame_id = frame_id;
    grid.info.map_load_time = this->now();
    grid.data = cells;
  }

  void fillGridMap(grid_map_msgs::msg::GridMap &msg,
                   const std::vector<signed char> &cells,
                   const std::string &frame_id)
  {
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    for (size_t i = 0; i < cells.size(); ++i)
    {
      msg.data[0].data[i] = static_cast<float>(cells[i]) / 127.0f;
    }
  }

  void publishGrids(const std::string &frame_id)
  {
    fillOccupancyGrid(*intensity_grid, ipoints_, frame_id);
    fillOccupancyGrid(*height_grid, hpoints_, frame_id);
    fillGridMap(*intensity_gridmap, ipoints_, frame_id);
    fillGridMap(*height_gridmap, hpoints_, frame_id);

    pub_hgrid->publish(*height_grid);
    pub_igrid->publish(*intensity_grid);
    pub_hgridmap->publish(*height_gridmap);
    pub_igridmap->publish(*intensity_gridmap);

    if (verbose1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Published " << grid_map.mapi_topic_name.c_str() << " and " << grid_map.maph_topic_name.c_str());
    }
  }

  // ---------- the callback itself ----------
  void
  lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    const ValueSource source(value_field);    
    if (!validateCloud(*input_msg, source)) { return; }
    if (!prepareRetentionState()) { return; }

    initGridMessages();

    const int64_t now_ns = cloudTimeNs(*input_msg);
    expireCells(now_ns);
    accumulatePoints(*input_msg, source, now_ns);
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
  static constexpr int64_t kNoStamp = std::numeric_limits<int64_t>::min();
  double retention_time_ = 0.0;                 // seconds; 0 = no retention (old behaviour)
  std::vector<signed char> ipoints_, hpoints_;  // retained cells (-128 = unknown)
  std::vector<int64_t> cell_stamp_ns_;          // time each cell was last written
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToGrid>());
  rclcpp::shutdown();
  return 0;
}