#include <gtest/gtest.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pointcloud_to_grid/value_reader.hpp"

using pointcloud_to_grid::ValueReader;
using pointcloud_to_grid::ValueSource;

TEST(ValueSource, ParsesKinds)
{
  EXPECT_EQ(ValueSource("z").kind(), ValueSource::Kind::Height);
  EXPECT_EQ(ValueSource("height").kind(), ValueSource::Kind::Height);
  EXPECT_EQ(ValueSource("r").kind(), ValueSource::Kind::RgbChannel);
  EXPECT_EQ(ValueSource("g").kind(), ValueSource::Kind::RgbChannel);
  EXPECT_EQ(ValueSource("b").kind(), ValueSource::Kind::RgbChannel);
  EXPECT_EQ(ValueSource("intensity").kind(), ValueSource::Kind::CustomField);
  EXPECT_EQ(ValueSource("reflectivity").kind(), ValueSource::Kind::CustomField);
}

TEST(ValueSource, PhysicalFieldNames)
{
  EXPECT_EQ(ValueSource("height").physicalFieldName(), "z");
  EXPECT_EQ(ValueSource("g").physicalFieldName(), "rgb");
  EXPECT_EQ(ValueSource("reflectivity").physicalFieldName(), "reflectivity");
}

TEST(ValueSource, KeepsRequestedField)
{
  EXPECT_EQ(ValueSource("reflectivity").requestedField(), "reflectivity");
}

// Two points with fields x, y, z, rgb, reflectivity
static sensor_msgs::msg::PointCloud2 makeCloud()
{
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    5,
    "x", 1, PointField::FLOAT32,
    "y", 1, PointField::FLOAT32,
    "z", 1, PointField::FLOAT32,
    "rgb", 1, PointField::FLOAT32,
    "reflectivity", 1, PointField::FLOAT32);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");
  sensor_msgs::PointCloud2Iterator<float> iter_refl(cloud, "reflectivity");
  for (int i = 0; i < 2; ++i, ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++iter_refl)
  {
    *iter_z = 0.5f * static_cast<float>(i);
    *iter_r = static_cast<uint8_t>(10 + i);
    *iter_g = static_cast<uint8_t>(20 + i);
    *iter_b = static_cast<uint8_t>(30 + i);
    *iter_refl = 0.25f + static_cast<float>(i);
  }
  return cloud;
}

TEST(ValueReader, ReadsHeight)
{
  const auto cloud = makeCloud();
  ValueReader reader(cloud, ValueSource("z"));
  EXPECT_FLOAT_EQ(reader.next(), 0.0f);
  EXPECT_FLOAT_EQ(reader.next(), 0.5f);
}

TEST(ValueReader, ReadsRgbChannel)
{
  const auto cloud = makeCloud();
  ValueReader reader(cloud, ValueSource("g"));
  EXPECT_FLOAT_EQ(reader.next(), 20.0f);
  EXPECT_FLOAT_EQ(reader.next(), 21.0f);
}

TEST(ValueReader, ReadsCustomField)
{
  const auto cloud = makeCloud();
  ValueReader reader(cloud, ValueSource("reflectivity"));
  EXPECT_FLOAT_EQ(reader.next(), 0.25f);
  EXPECT_FLOAT_EQ(reader.next(), 1.25f);
}