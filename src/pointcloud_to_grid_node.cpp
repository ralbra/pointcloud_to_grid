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
// ROS package
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
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

    void
  lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    // --- Validate required fields ---
    if (!hasField(*input_msg, "x") || !hasField(*input_msg, "y"))
    {
      RCLCPP_ERROR(this->get_logger(), "Incoming PointCloud2 is missing 'x' or 'y' field. Skipping frame.");
      return;
    }

    const bool use_height = (value_field == "z" || value_field == "height");
    const bool use_rgb_channel = (value_field == "r" || value_field == "g" || value_field == "b");

    // Physical field that must exist in the cloud: "z" for height, the packed
    // "rgb" for r/g/b channels, otherwise the requested field itself.
    const std::string physical_field =
        use_height ? "z" : (use_rgb_channel ? "rgb" : value_field);

    if (!hasField(*input_msg, physical_field))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Requested value_field '%s' not found in incoming PointCloud2. "
                   "Available fields: [%s]. Skipping frame.",
                   value_field.c_str(), listFields(*input_msg).c_str());
      return;
    }

    if (grid_map.cell_num_x <= 0 || grid_map.cell_num_y <= 0)
    {
      return;
    }
    const size_t n_cells = static_cast<size_t>(grid_map.cell_num_x) * grid_map.cell_num_y;

    // Initialize grid messages
    grid_map.initGrid(intensity_grid);
    grid_map.initGrid(height_grid);
    grid_map.initGridMap(intensity_gridmap, "intensity");
    grid_map.initGridMap(height_gridmap, "height");

    // --- Point retention ---
    // (Re)initialise retained state on first use or after a size change
    if (ipoints_.size() != n_cells)
    {
      ipoints_.assign(n_cells, -128);
      hpoints_.assign(n_cells, -128);
      cell_stamp_ns_.assign(n_cells, kNoStamp);
    }

    // Time base: message stamp, falling back to the node clock if unset
    rclcpp::Time stamp(input_msg->header.stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() == 0)
    {
      stamp = this->now();
    }
    const int64_t now_ns = stamp.nanoseconds();
    const int64_t retention_ns = static_cast<int64_t>(retention_time_ * 1e9);

    // Expire old cells. With retention_ns == 0 every cell expires, which
    // reproduces the previous behaviour. age < 0 means time jumped backwards.
    for (size_t i = 0; i < n_cells; ++i)
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

    // --- Generic iterators: x, y always; value field chosen at runtime ---
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_msg, "y");

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_z;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_value_float;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_value_byte;

    if (use_height)
    {
      iter_z = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input_msg, "z");
    }
    else if (use_rgb_channel)
    {
      // "r"/"g"/"b" are pseudo-fields: the iterator unpacks the right byte
      // from the packed "rgb" float field automatically.
      iter_value_byte = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*input_msg, value_field);
    }
    else
    {
      // Covers "intensity", "reflectivity", or any other custom FLOAT32 field name.
      iter_value_float = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input_msg, value_field);
    }

    // --- Write points into the retained cells ---
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
    {
      const float px = *iter_x;
      const float py = *iter_y;

      // Read and advance the value iterator for every point, even if the
      // point is skipped below, so it stays in step with x and y.
      float value = 0.0f;
      if (use_height)
      {
        value = **iter_z;
        ++(*iter_z);
      }
      else if (use_rgb_channel)
      {
        value = static_cast<float>(**iter_value_byte);
        ++(*iter_value_byte);
      }
      else
      {
        value = **iter_value_float;
        ++(*iter_value_float);
      }

      if (px > 0.01 || px < -0.01)
      {
        if (px > grid_map.bottomright_x && px < grid_map.topleft_x)
        {
          if (py > grid_map.bottomright_y && py < grid_map.topleft_y)
          {
            PointXY cell = grid_map.getIndex(px, py);
            if (cell.x < grid_map.cell_num_x && cell.y < grid_map.cell_num_y)
            {
              // --- this is the block that replaces the old two lines ---
              const size_t idx = static_cast<size_t>(cell.y) * grid_map.cell_num_x + cell.x;
              ipoints_[idx] = value * grid_map.intensity_factor;
              hpoints_[idx] = value * grid_map.height_factor;
              cell_stamp_ns_[idx] = now_ns;
            }
            else
            {
              RCLCPP_WARN_STREAM(this->get_logger(), "Cell out of range: " << cell.x << " - " << grid_map.cell_num_x << " ||| " << cell.y << " - " << grid_map.cell_num_y);
            }
          }
        }
      }
    }

    // --- Publish OccupancyGrids ---
    intensity_grid->header.stamp = this->now();
    intensity_grid->header.frame_id = input_msg->header.frame_id;
    intensity_grid->info.map_load_time = this->now();
    intensity_grid->data = ipoints_;
    height_grid->header.stamp = this->now();
    height_grid->header.frame_id = input_msg->header.frame_id;
    height_grid->info.map_load_time = this->now();
    height_grid->data = hpoints_;

    // --- Convert to GridMap format ---
    intensity_gridmap->header.stamp = this->now();
    intensity_gridmap->header.frame_id = input_msg->header.frame_id;
    for (size_t i = 0; i < ipoints_.size(); ++i)
    {
      intensity_gridmap->data[0].data[i] = static_cast<float>(ipoints_[i]) / 127.0f;
    }

    height_gridmap->header.stamp = this->now();
    height_gridmap->header.frame_id = input_msg->header.frame_id;
    for (size_t i = 0; i < hpoints_.size(); ++i)
    {
      height_gridmap->data[0].data[i] = static_cast<float>(hpoints_[i]) / 127.0f;
    }

    pub_hgrid->publish(*height_grid);
    pub_igrid->publish(*intensity_grid);
    pub_hgridmap->publish(*height_gridmap);
    pub_igridmap->publish(*intensity_gridmap);
    if (verbose1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Published " << grid_map.mapi_topic_name.c_str() << " and " << grid_map.maph_topic_name.c_str());
    }
  }

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