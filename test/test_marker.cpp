#include <gtest/gtest.h>

#include "pointcloud_to_grid/marker.hpp"

using visualization_msgs::msg::Marker;

TEST(Marker, DebugMarkerHasExpectedShapeAndPose)
{
  Marker m;
  init_debug_marker(m, 1.5, -2.0, 7);

  EXPECT_EQ(m.ns, "debug7");
  EXPECT_EQ(m.type, Marker::CYLINDER);
  EXPECT_EQ(m.action, Marker::MODIFY);
  EXPECT_EQ(m.id, 3);
  EXPECT_DOUBLE_EQ(m.scale.x, 0.4);
  EXPECT_DOUBLE_EQ(m.scale.y, 0.4);
  EXPECT_DOUBLE_EQ(m.scale.z, 0.4);
  EXPECT_FLOAT_EQ(m.color.r, 0.1f);
  EXPECT_FLOAT_EQ(m.color.g, 0.2f);
  EXPECT_FLOAT_EQ(m.color.b, 0.8f);
  EXPECT_FLOAT_EQ(m.color.a, 1.0f);
  EXPECT_DOUBLE_EQ(m.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(m.pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(m.pose.position.z, 0.0);
}

TEST(Marker, DebugMarkersWithDifferentIdsUseDifferentNamespaces)
{
  // RViz identifies a marker by (ns, id); the id stays 3, so ns must differ
  Marker a;
  Marker b;
  init_debug_marker(a, 0.0, 0.0, 1);
  init_debug_marker(b, 0.0, 0.0, 2);
  EXPECT_NE(a.ns, b.ns);
}

TEST(Marker, TextMarkerHasExpectedProperties)
{
  Marker m;
  init_text_debug_marker(m);

  EXPECT_EQ(m.ns, "debug_text");
  EXPECT_EQ(m.type, Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(m.action, Marker::MODIFY);
  EXPECT_EQ(m.id, 4);
  EXPECT_DOUBLE_EQ(m.scale.z, 0.4);
  EXPECT_FLOAT_EQ(m.color.r, 0.2f);
  EXPECT_FLOAT_EQ(m.color.g, 0.8f);
  EXPECT_FLOAT_EQ(m.color.b, 0.2f);
  EXPECT_FLOAT_EQ(m.color.a, 1.0f);
  EXPECT_DOUBLE_EQ(m.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.position.z, 0.0);
}