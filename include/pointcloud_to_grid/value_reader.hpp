#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace pointcloud_to_grid
{

class ValueSource
{
public:
  enum class Kind { Height, RgbChannel, CustomField };

  // Parses the value_field parameter string.
  explicit ValueSource(const std::string &value_field)
  : field_(value_field)
  {
    if (value_field == "z" || value_field == "height") { kind_ = Kind::Height; }
    else if (value_field == "r" || value_field == "g" || value_field == "b") { kind_ = Kind::RgbChannel; }
    else { kind_ = Kind::CustomField; }
  }

  Kind kind() const { return kind_; }

  // The value_field exactly as requested (e.g. "reflectivity" or "r").
  const std::string &requestedField() const { return field_; }

  // Name of the field that must physically exist in the incoming cloud.
  std::string physicalFieldName() const
  {
    switch (kind_)
    {
      case Kind::Height: return "z";
      case Kind::RgbChannel: return "rgb";   // r/g/b are unpacked from the packed rgb field
      case Kind::CustomField: break;
    }
    return field_;
  }

private:
  Kind kind_ = Kind::CustomField;
  std::string field_;
};

// Reads the chosen value field one point at a time, hiding which of the
// iterator types is behind it.
class ValueReader
{
public:
  ValueReader(const sensor_msgs::msg::PointCloud2 &cloud, const ValueSource &source)
  {
    if (source.kind() == ValueSource::Kind::RgbChannel)
    {
      // "r"/"g"/"b" are pseudo-fields: the iterator unpacks the right byte
      byte_iter_ = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(cloud, source.requestedField());
    }
    else
    {
      const std::string iter_field =
          (source.kind() == ValueSource::Kind::Height) ? std::string("z") : source.requestedField();
      float_iter_ = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(cloud, iter_field);
    }
  }

  // Returns the current point's value and advances to the next point.
  float next()
  {
    if (byte_iter_)
    {
      const float v = static_cast<float>(**byte_iter_);
      ++(*byte_iter_);
      return v;
    }
    const float v = **float_iter_;
    ++(*float_iter_);
    return v;
  }

private:
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> float_iter_;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> byte_iter_;
};

}  // namespace pointcloud_to_grid