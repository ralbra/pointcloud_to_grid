#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "pointcloud_to_grid/pointcloud_to_grid_core.hpp"
#include "pointcloud_to_grid/value_reader.hpp"

namespace pointcloud_to_grid
{

// ---------- cloud helpers ----------
inline bool hasField(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name)
{
  return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                     [&name](const sensor_msgs::msg::PointField &f) { return f.name == name; });
}

inline std::string listFields(const sensor_msgs::msg::PointCloud2 &cloud)
{
  std::string out;
  for (const auto &f : cloud.fields)
  {
    out += (out.empty() ? "" : ", ") + f.name;
  }
  return out;
}

// Returns an empty string if the cloud can be processed, otherwise a
// description of what is missing. The caller decides how to log it.
inline std::string cloudValidationError(const sensor_msgs::msg::PointCloud2 &cloud,
                                        const ValueSource &source)
{
  if (!hasField(cloud, "x") || !hasField(cloud, "y"))
  {
    return "Incoming PointCloud2 is missing 'x' or 'y' field.";
  }
  if (!hasField(cloud, source.physicalFieldName()))
  {
    return "Requested value_field '" + source.requestedField() +
           "' not found in incoming PointCloud2. Available fields: [" + listFields(cloud) + "].";
  }
  return "";
}

// Message stamp in ns, or fallback_ns if the stamp is unset (zero).
inline int64_t cloudTimeNs(const sensor_msgs::msg::PointCloud2 &cloud, int64_t fallback_ns)
{
  const int64_t stamp_ns =
      static_cast<int64_t>(cloud.header.stamp.sec) * 1000000000LL + cloud.header.stamp.nanosec;
  return stamp_ns == 0 ? fallback_ns : stamp_ns;
}

// ---------- retained cell state ----------
class RetainedGrid
{
public:
  static constexpr signed char kUnknown = -128;
  static constexpr int64_t kNoStamp = std::numeric_limits<int64_t>::min();

  // Makes the storage match the grid size. Cells are kept if the size is
  // unchanged; a size change resets everything. Returns false for an empty grid.
  bool prepare(int cells_x, int cells_y)
  {
    if (cells_x <= 0 || cells_y <= 0)
    {
      return false;
    }
    if (cells_x != cells_x_ || cells_y != cells_y_)
    {
      const size_t n = static_cast<size_t>(cells_x) * static_cast<size_t>(cells_y);
      intensity_.assign(n, kUnknown);
      height_.assign(n, kUnknown);
      stamp_ns_.assign(n, kNoStamp);
      cells_x_ = cells_x;
      cells_y_ = cells_y;
    }
    return true;
  }

  // Forget all retained cells (e.g. after the grid geometry changed).
  void clear()
  {
    intensity_.clear();
    height_.clear();
    stamp_ns_.clear();
    cells_x_ = 0;
    cells_y_ = 0;
  }

  // Drops cells older than retention_ns. retention_ns == 0 drops every cell,
  // i.e. no retention. A negative age means time jumped backwards (e.g. a
  // looping bag); such cells are dropped too.
  void expire(int64_t now_ns, int64_t retention_ns)
  {
    for (size_t i = 0; i < stamp_ns_.size(); ++i)
    {
      if (stamp_ns_[i] == kNoStamp)
      {
        continue;
      }
      const int64_t age_ns = now_ns - stamp_ns_[i];
      if (age_ns < 0 || age_ns >= retention_ns)
      {
        intensity_[i] = kUnknown;
        height_[i] = kUnknown;
        stamp_ns_[i] = kNoStamp;
      }
    }
  }

  // Writes one cell and refreshes its timestamp. Returns false if the cell
  // lies outside the grid.
  bool set(int cell_x, int cell_y, signed char intensity, signed char height, int64_t stamp_ns)
  {
    if (cell_x < 0 || cell_x >= cells_x_ || cell_y < 0 || cell_y >= cells_y_)
    {
      return false;
    }
    const size_t idx = static_cast<size_t>(cell_y) * static_cast<size_t>(cells_x_) +
                       static_cast<size_t>(cell_x);
    intensity_[idx] = intensity;
    height_[idx] = height;
    stamp_ns_[idx] = stamp_ns;
    return true;
  }

  // kNoStamp if the cell is unknown or outside the grid.
  int64_t stampAt(int cell_x, int cell_y) const
  {
    if (cell_x < 0 || cell_x >= cells_x_ || cell_y < 0 || cell_y >= cells_y_)
    {
      return kNoStamp;
    }
    return stamp_ns_[static_cast<size_t>(cell_y) * static_cast<size_t>(cells_x_) +
                     static_cast<size_t>(cell_x)];
  }

  const std::vector<signed char> &intensity() const { return intensity_; }
  const std::vector<signed char> &height() const { return height_; }

private:
  int cells_x_ = 0;
  int cells_y_ = 0;
  std::vector<signed char> intensity_;
  std::vector<signed char> height_;
  std::vector<int64_t> stamp_ns_;
};

// ---------- point accumulation ----------
// True if (x, y) lies inside the grid area and not on the x ~ 0 axis.
inline bool isInsideGrid(const GridMap &grid, float x, float y)
{
  if (std::abs(x) <= 0.01f)
  {
    return false;
  }
  return x > grid.bottomright_x && x < grid.topleft_x &&
         y > grid.bottomright_y && y < grid.topleft_y;
}

// Writes every usable point of the cloud into the retained grid. Returns the
// number of points that were written.
inline size_t accumulatePoints(const sensor_msgs::msg::PointCloud2 &cloud,
                               const ValueSource &source,
                               const GridMap &grid,
                               RetainedGrid &retained,
                               int64_t now_ns)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  ValueReader reader(cloud, source);

  size_t written = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  {
    // Always read the value, even if the point is skipped below,
    // so the value iterator stays in step with x and y.
    const float value = reader.next();
    const float px = *iter_x;
    const float py = *iter_y;

    if (!isInsideGrid(grid, px, py))
    {
      continue;
    }
    const PointXY cell = grid.getIndex(px, py);
    if (!grid.isValidCell(cell))
    {
      continue;
    }

    const auto intensity_cell = static_cast<signed char>(value * grid.intensity_factor);
    const auto height_cell = static_cast<signed char>(value * grid.height_factor);
    if (retained.set(cell.x, cell.y, intensity_cell, height_cell, now_ns))
    {
      ++written;
    }
  }
  return written;
}

// ---------- message filling ----------
inline void fillOccupancyGrid(nav_msgs::msg::OccupancyGrid &grid,
                              const std::vector<signed char> &cells,
                              const std::string &frame_id,
                              const builtin_interfaces::msg::Time &stamp)
{
  grid.header.stamp = stamp;
  grid.header.frame_id = frame_id;
  grid.info.map_load_time = stamp;
  grid.data = cells;
}

// The layer must already be sized (see GridMap::initGridMap). Returns false,
// and leaves the data untouched, if the sizes do not match.
inline bool fillGridMap(grid_map_msgs::msg::GridMap &msg,
                        const std::vector<signed char> &cells,
                        const std::string &frame_id,
                        const builtin_interfaces::msg::Time &stamp)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  if (msg.data.empty() || msg.data[0].data.size() != cells.size())
  {
    return false;
  }
  for (size_t i = 0; i < cells.size(); ++i)
  {
    msg.data[0].data[i] = static_cast<float>(cells[i]) / 127.0f;
  }
  return true;
}

}  // namespace pointcloud_to_grid