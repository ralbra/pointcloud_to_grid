#include <gtest/gtest.h>

#include <memory>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pointcloud_to_grid/pointcloud_to_grid_core.hpp"

namespace
{
GridMap makeGrid(float pos_x, float pos_y, float cell, float len_x, float len_y)
{
  GridMap g{};
  g.position_x = pos_x;
  g.position_y = pos_y;
  g.cell_size = cell;
  g.length_x = len_x;
  g.length_y = len_y;
  g.frame_out = "map";
  g.paramRefresh();
  return g;
}
}  // namespace

TEST(GridMap, ParamRefreshComputesBoundsAndCellCounts)
{
  const GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);
  EXPECT_FLOAT_EQ(g.topleft_x, 5.0f);
  EXPECT_FLOAT_EQ(g.bottomright_x, -5.0f);
  EXPECT_FLOAT_EQ(g.topleft_y, 10.0f);
  EXPECT_FLOAT_EQ(g.bottomright_y, -10.0f);
  EXPECT_EQ(g.cell_num_x, 20);
  EXPECT_EQ(g.cell_num_y, 40);
}

TEST(GridMap, ParamRefreshRespectsPosition)
{
  const GridMap g = makeGrid(-5.0f, 2.0f, 1.0f, 4.0f, 6.0f);
  EXPECT_FLOAT_EQ(g.topleft_x, -3.0f);
  EXPECT_FLOAT_EQ(g.bottomright_x, -7.0f);
  EXPECT_FLOAT_EQ(g.topleft_y, 5.0f);
  EXPECT_FLOAT_EQ(g.bottomright_y, -1.0f);
  EXPECT_EQ(g.cell_num_x, 4);
  EXPECT_EQ(g.cell_num_y, 6);
}

TEST(GridMap, CellCountTruncatesPartialCells)
{
  const GridMap g = makeGrid(0.0f, 0.0f, 0.3f, 10.0f, 10.0f);
  EXPECT_EQ(g.cell_num_x, 33);   // 10 / 0.3 = 33.3
}

TEST(GridMap, Getters)
{
  GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);
  EXPECT_EQ(g.getSize(), 800);
  EXPECT_EQ(g.getSizeX(), 20);
  EXPECT_EQ(g.getSizeY(), 40);
  EXPECT_DOUBLE_EQ(g.getLengthX(), 10.0);
  EXPECT_DOUBLE_EQ(g.getLengthY(), 20.0);
  EXPECT_DOUBLE_EQ(g.getResolution(), 0.5);
}

TEST(GridMap, GetIndexCountsFromTopLeftCorner)
{
  GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);

  PointXY c = g.getIndex(4.75, 9.75);     // just inside the top-left corner
  EXPECT_EQ(c.x, 0);
  EXPECT_EQ(c.y, 0);

  c = g.getIndex(4.25, 9.25);             // one cell towards -x and -y
  EXPECT_EQ(c.x, 1);
  EXPECT_EQ(c.y, 1);

  c = g.getIndex(0.25, 0.25);
  EXPECT_EQ(c.x, 9);
  EXPECT_EQ(c.y, 19);

  c = g.getIndex(-4.75, -9.75);           // bottom-right corner = last cell
  EXPECT_EQ(c.x, g.cell_num_x - 1);
  EXPECT_EQ(c.y, g.cell_num_y - 1);
}

TEST(GridMap, GetIndexRespectsPosition)
{
  GridMap g = makeGrid(-5.0f, 2.0f, 1.0f, 4.0f, 6.0f);
  PointXY c = g.getIndex(-3.5, 4.5);
  EXPECT_EQ(c.x, 0);
  EXPECT_EQ(c.y, 0);
  c = g.getIndex(-6.5, -0.5);
  EXPECT_EQ(c.x, 3);
  EXPECT_EQ(c.y, 5);
}

TEST(GridMap, InitGridFillsOccupancyGridInfo)
{
  GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);
  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  g.initGrid(grid);

  EXPECT_EQ(grid->header.frame_id, "map");
  EXPECT_EQ(grid->info.width, 20u);
  EXPECT_EQ(grid->info.height, 40u);
  EXPECT_FLOAT_EQ(grid->info.resolution, 0.5f);
  EXPECT_DOUBLE_EQ(grid->info.origin.position.x, 5.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.position.y, 10.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(grid->info.origin.orientation.w, 0.0);
}

TEST(GridMap, InitGridMapBuildsSingleLayer)
{
  GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  g.initGridMap(msg, "intensity");

  ASSERT_EQ(msg->layers.size(), 1u);
  EXPECT_EQ(msg->layers[0], "intensity");
  ASSERT_EQ(msg->data.size(), 1u);
  EXPECT_EQ(msg->data[0].data.size(), 800u);

  ASSERT_EQ(msg->data[0].layout.dim.size(), 2u);
  EXPECT_EQ(msg->data[0].layout.dim[0].label, "column_index");
  EXPECT_EQ(msg->data[0].layout.dim[0].size, 40u);
  EXPECT_EQ(msg->data[0].layout.dim[0].stride, 800u);
  EXPECT_EQ(msg->data[0].layout.dim[1].label, "row_index");
  EXPECT_EQ(msg->data[0].layout.dim[1].size, 20u);
  EXPECT_EQ(msg->data[0].layout.dim[1].stride, 20u);

  EXPECT_DOUBLE_EQ(msg->info.resolution, 0.5);
  EXPECT_DOUBLE_EQ(msg->info.length_x, 10.0);
  EXPECT_DOUBLE_EQ(msg->info.length_y, 20.0);
  EXPECT_DOUBLE_EQ(msg->info.pose.orientation.w, 1.0);
}

TEST(GridMap, InitGridMapReplacesPreviousLayer)
{
  GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  g.initGridMap(msg, "first");
  g.initGridMap(msg, "second");

  ASSERT_EQ(msg->layers.size(), 1u);
  EXPECT_EQ(msg->layers[0], "second");
  EXPECT_EQ(msg->data.size(), 1u);
}

TEST(GridMap, GetIndexOutsideGridIsNotMirrored)
{
  GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);

  PointXY c = g.getIndex(5.25, 10.25);   // just beyond the top-left corner
  EXPECT_EQ(c.x, -1);
  EXPECT_EQ(c.y, -1);

  c = g.getIndex(7.25, 9.75);            // the old fabs() version returned x = 4 here
  EXPECT_EQ(c.x, -5);
  EXPECT_EQ(c.y, 0);

  c = g.getIndex(-5.25, -10.25);         // beyond the bottom-right corner
  EXPECT_EQ(c.x, 20);
  EXPECT_EQ(c.y, 40);
}

TEST(GridMap, IsValidCell)
{
  GridMap g = makeGrid(0.0f, 0.0f, 0.5f, 10.0f, 20.0f);   // 20 x 40 cells
  EXPECT_TRUE(g.isValidCell({0, 0}));
  EXPECT_TRUE(g.isValidCell({19, 39}));
  EXPECT_FALSE(g.isValidCell({-1, 0}));
  EXPECT_FALSE(g.isValidCell({0, -1}));
  EXPECT_FALSE(g.isValidCell({20, 0}));
  EXPECT_FALSE(g.isValidCell({0, 40}));
}