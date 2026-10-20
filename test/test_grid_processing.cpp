#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pointcloud_to_grid/grid_processing.hpp"

using pointcloud_to_grid::RetainedGrid;
using pointcloud_to_grid::ValueSource;
using pointcloud_to_grid::accumulatePoints;
using pointcloud_to_grid::cloudTimeNs;
using pointcloud_to_grid::cloudValidationError;
using pointcloud_to_grid::fillGridMap;
using pointcloud_to_grid::fillOccupancyGrid;
using pointcloud_to_grid::hasField;
using pointcloud_to_grid::isInsideGrid;
using pointcloud_to_grid::listFields;

namespace
{
constexpr int64_t kSecond = 1000000000LL;

// A cloud that only carries field names; enough for the validation helpers.
sensor_msgs::msg::PointCloud2 cloudWithFields(const std::vector<std::string> &names)
{
  sensor_msgs::msg::PointCloud2 cloud;
  for (const auto &name : names)
  {
    sensor_msgs::msg::PointField f;
    f.name = name;
    cloud.fields.push_back(f);
  }
  return cloud;
}

struct TestPoint
{
  float x;
  float y;
  float z;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  float reflectivity;
};

// Cloud with fields x, y, z, rgb, reflectivity (the layout the generator
// publishes for xyzrgbi). Pass at least one point.
sensor_msgs::msg::PointCloud2 makeCloud(const std::vector<TestPoint> &points)
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
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> ir(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> ig(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> ib(cloud, "b");
  sensor_msgs::PointCloud2Iterator<float> irefl(cloud, "reflectivity");
  for (const auto &p : points)
  {
    *ix = p.x;
    *iy = p.y;
    *iz = p.z;
    *ir = p.r;
    *ig = p.g;
    *ib = p.b;
    *irefl = p.reflectivity;
    ++ix; ++iy; ++iz; ++ir; ++ig; ++ib; ++irefl;
  }
  return cloud;
}

// 10 x 10 cells of 1 m: x and y range from -5 to 5
GridMap makeGrid10()
{
  GridMap g{};
  g.position_x = 0.0f;
  g.position_y = 0.0f;
  g.cell_size = 1.0f;
  g.length_x = 10.0f;
  g.length_y = 10.0f;
  g.frame_out = "map";
  g.paramRefresh();
  return g;
}

// 2 x 2 cells of 1 m
GridMap makeGrid2x2()
{
  GridMap g{};
  g.cell_size = 1.0f;
  g.length_x = 2.0f;
  g.length_y = 2.0f;
  g.frame_out = "map";
  g.paramRefresh();
  return g;
}
}  // namespace

// ---------------------------------------------------------------- fields

TEST(CloudFields, HasFieldIsExactAndCaseSensitive)
{
  const auto cloud = cloudWithFields({"x", "y", "z"});
  EXPECT_TRUE(hasField(cloud, "x"));
  EXPECT_TRUE(hasField(cloud, "z"));
  EXPECT_FALSE(hasField(cloud, "X"));
  EXPECT_FALSE(hasField(cloud, "intensity"));
  EXPECT_FALSE(hasField(cloudWithFields({}), "x"));
}

TEST(CloudFields, ListFieldsJoinsNamesInOrder)
{
  EXPECT_EQ(listFields(cloudWithFields({"x", "y", "z", "rgb"})), "x, y, z, rgb");
  EXPECT_EQ(listFields(cloudWithFields({"x"})), "x");
  EXPECT_EQ(listFields(cloudWithFields({})), "");
}

// ------------------------------------------------------------ validation

TEST(CloudValidation, AcceptsEveryKindOfValueSource)
{
  const auto cloud = cloudWithFields({"x", "y", "z", "rgb", "reflectivity"});
  EXPECT_EQ(cloudValidationError(cloud, ValueSource("z")), "");
  EXPECT_EQ(cloudValidationError(cloud, ValueSource("height")), "");
  EXPECT_EQ(cloudValidationError(cloud, ValueSource("g")), "");           // needs "rgb"
  EXPECT_EQ(cloudValidationError(cloud, ValueSource("reflectivity")), "");
}

TEST(CloudValidation, ReportsMissingXOrY)
{
  const std::string err_no_x = cloudValidationError(cloudWithFields({"y", "z"}), ValueSource("z"));
  EXPECT_NE(err_no_x.find("missing 'x' or 'y'"), std::string::npos);

  const std::string err_no_y = cloudValidationError(cloudWithFields({"x", "z"}), ValueSource("z"));
  EXPECT_NE(err_no_y.find("missing 'x' or 'y'"), std::string::npos);
}

TEST(CloudValidation, ReportsMissingValueFieldAndListsAvailableOnes)
{
  const auto cloud = cloudWithFields({"x", "y", "z", "rgb", "reflectivity"});
  const std::string err = cloudValidationError(cloud, ValueSource("intensity"));
  EXPECT_NE(err.find("'intensity'"), std::string::npos);
  EXPECT_NE(err.find("[x, y, z, rgb, reflectivity]"), std::string::npos);
}

TEST(CloudValidation, RgbChannelNeedsThePackedRgbField)
{
  const auto cloud = cloudWithFields({"x", "y", "z"});
  EXPECT_NE(cloudValidationError(cloud, ValueSource("r")), "");
}

TEST(CloudValidation, HeightNeedsZ)
{
  const auto cloud = cloudWithFields({"x", "y"});
  EXPECT_NE(cloudValidationError(cloud, ValueSource("z")), "");
}

// ------------------------------------------------------------------ time

TEST(CloudTime, UsesMessageStampWhenSet)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 12;
  cloud.header.stamp.nanosec = 500000000u;
  EXPECT_EQ(cloudTimeNs(cloud, 999), 12 * kSecond + 500000000LL);
}

TEST(CloudTime, FallsBackWhenStampIsZero)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 0;
  cloud.header.stamp.nanosec = 0u;
  EXPECT_EQ(cloudTimeNs(cloud, 999), 999);
}

TEST(CloudTime, NanosecondsOnlyStampIsStillASetStamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 0;
  cloud.header.stamp.nanosec = 5u;
  EXPECT_EQ(cloudTimeNs(cloud, 999), 5);
}

TEST(CloudTime, LargeSecondsDoNotOverflow)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 2000000000;
  cloud.header.stamp.nanosec = 1u;
  EXPECT_EQ(cloudTimeNs(cloud, 0), 2000000000LL * kSecond + 1);
}

// ---------------------------------------------------------- RetainedGrid

TEST(RetainedGrid, PrepareRejectsEmptyGrids)
{
  RetainedGrid r;
  EXPECT_FALSE(r.prepare(0, 5));
  EXPECT_FALSE(r.prepare(5, 0));
  EXPECT_FALSE(r.prepare(-1, 5));
  EXPECT_FALSE(r.prepare(5, -1));
}

TEST(RetainedGrid, PrepareAllocatesUnknownCells)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(4, 3));
  ASSERT_EQ(r.intensity().size(), 12u);
  ASSERT_EQ(r.height().size(), 12u);
  for (size_t i = 0; i < 12; ++i)
  {
    EXPECT_EQ(r.intensity()[i], RetainedGrid::kUnknown);
    EXPECT_EQ(r.height()[i], RetainedGrid::kUnknown);
  }
  EXPECT_EQ(r.stampAt(0, 0), RetainedGrid::kNoStamp);
}

TEST(RetainedGrid, SetStoresValuesAtRowMajorIndex)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(4, 3));
  EXPECT_TRUE(r.set(1, 2, 7, 9, 100));
  EXPECT_EQ(r.intensity()[2 * 4 + 1], 7);
  EXPECT_EQ(r.height()[2 * 4 + 1], 9);
  EXPECT_EQ(r.stampAt(1, 2), 100);
}

TEST(RetainedGrid, SetOutsideTheGridFails)
{
  RetainedGrid r;
  EXPECT_FALSE(r.set(0, 0, 1, 1, 1));   // not prepared yet
  ASSERT_TRUE(r.prepare(4, 3));
  EXPECT_FALSE(r.set(4, 0, 1, 1, 1));
  EXPECT_FALSE(r.set(0, 3, 1, 1, 1));
  EXPECT_FALSE(r.set(-1, 0, 1, 1, 1));
  EXPECT_FALSE(r.set(0, -1, 1, 1, 1));
  EXPECT_EQ(r.stampAt(4, 0), RetainedGrid::kNoStamp);
}

TEST(RetainedGrid, PrepareKeepsCellsWhenSizeIsUnchanged)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(4, 3));
  r.set(1, 1, 5, 6, 42);
  ASSERT_TRUE(r.prepare(4, 3));
  EXPECT_EQ(r.intensity()[1 * 4 + 1], 5);
  EXPECT_EQ(r.stampAt(1, 1), 42);
}

TEST(RetainedGrid, PrepareResetsWhenDimensionsChangeEvenIfCellCountMatches)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(4, 3));
  r.set(1, 1, 5, 6, 42);
  ASSERT_TRUE(r.prepare(3, 4));   // also 12 cells, but a different shape
  EXPECT_EQ(r.intensity().size(), 12u);
  for (const auto cell : r.intensity()) { EXPECT_EQ(cell, RetainedGrid::kUnknown); }
  EXPECT_EQ(r.stampAt(1, 1), RetainedGrid::kNoStamp);
}

TEST(RetainedGrid, ClearForgetsEverything)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(4, 3));
  r.set(1, 1, 5, 6, 42);
  r.clear();
  EXPECT_TRUE(r.intensity().empty());
  EXPECT_FALSE(r.set(1, 1, 5, 6, 42));   // needs prepare() again
  ASSERT_TRUE(r.prepare(4, 3));
  EXPECT_EQ(r.stampAt(1, 1), RetainedGrid::kNoStamp);
}

TEST(RetainedGrid, ExpireDropsOnlyCellsOlderThanRetentionTime)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(4, 3));
  r.set(0, 0, 1, 2, 0);
  r.set(1, 0, 3, 4, 500000000);

  r.expire(900000000, kSecond);        // ages 0.9 s and 0.4 s: both kept
  EXPECT_EQ(r.stampAt(0, 0), 0);
  EXPECT_EQ(r.stampAt(1, 0), 500000000);

  r.expire(1200000000, kSecond);       // ages 1.2 s and 0.7 s
  EXPECT_EQ(r.stampAt(0, 0), RetainedGrid::kNoStamp);
  EXPECT_EQ(r.intensity()[0], RetainedGrid::kUnknown);
  EXPECT_EQ(r.height()[0], RetainedGrid::kUnknown);
  EXPECT_EQ(r.stampAt(1, 0), 500000000);
  EXPECT_EQ(r.intensity()[1], 3);
}

TEST(RetainedGrid, ExpireDropsCellsExactlyAtTheRetentionAge)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(2, 2));
  r.set(0, 0, 1, 1, 0);
  r.expire(kSecond, kSecond);          // age == retention
  EXPECT_EQ(r.stampAt(0, 0), RetainedGrid::kNoStamp);
}

TEST(RetainedGrid, ZeroRetentionDropsEverythingAtOnce)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(2, 2));
  r.set(0, 0, 1, 1, 100);
  r.expire(100, 0);                    // age 0, retention 0
  EXPECT_EQ(r.stampAt(0, 0), RetainedGrid::kNoStamp);
}

TEST(RetainedGrid, ExpireDropsCellsFromTheFuture)
{
  RetainedGrid r;
  ASSERT_TRUE(r.prepare(2, 2));
  r.set(0, 0, 1, 1, 500);
  r.expire(100, kSecond);              // time jumped backwards
  EXPECT_EQ(r.stampAt(0, 0), RetainedGrid::kNoStamp);
}

// --------------------------------------------------------- isInsideGrid

TEST(IsInsideGrid, EdgesAreExclusive)
{
  const GridMap g = makeGrid10();
  EXPECT_TRUE(isInsideGrid(g, 1.0f, 1.0f));
  EXPECT_TRUE(isInsideGrid(g, -4.9f, -4.9f));
  EXPECT_FALSE(isInsideGrid(g, 5.0f, 1.0f));
  EXPECT_FALSE(isInsideGrid(g, -5.0f, 1.0f));
  EXPECT_FALSE(isInsideGrid(g, 1.0f, 5.0f));
  EXPECT_FALSE(isInsideGrid(g, 1.0f, -5.0f));
  EXPECT_FALSE(isInsideGrid(g, 7.0f, 1.0f));
  EXPECT_FALSE(isInsideGrid(g, 1.0f, -7.0f));
}

TEST(IsInsideGrid, SkipsPointsOnTheXAxis)
{
  const GridMap g = makeGrid10();
  EXPECT_FALSE(isInsideGrid(g, 0.0f, 1.0f));
  EXPECT_FALSE(isInsideGrid(g, 0.01f, 1.0f));
  EXPECT_FALSE(isInsideGrid(g, -0.01f, 1.0f));
  EXPECT_TRUE(isInsideGrid(g, 0.02f, 1.0f));
  EXPECT_TRUE(isInsideGrid(g, -0.02f, 1.0f));
}

// ------------------------------------------------------ accumulatePoints

class AccumulateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    grid_ = makeGrid10();
    grid_.intensity_factor = 10.0f;
    grid_.height_factor = 100.0f;
    ASSERT_TRUE(retained_.prepare(grid_.cell_num_x, grid_.cell_num_y));
  }

  // Cell value at (x, y) of a 10-cell-wide grid
  static int at(const std::vector<signed char> &cells, int x, int y)
  {
    return cells[static_cast<size_t>(y) * 10 + static_cast<size_t>(x)];
  }

  size_t knownCells() const
  {
    const auto &c = retained_.intensity();
    return std::count_if(c.begin(), c.end(),
                         [](signed char v) { return v != RetainedGrid::kUnknown; });
  }

  GridMap grid_{};
  RetainedGrid retained_;
  const int64_t now_ns_ = 5 * kSecond;
};

// point layout below: x, y, z, r, g, b, reflectivity

TEST_F(AccumulateTest, WritesInsidePointsWithScaledValues)
{
  const auto cloud = makeCloud({
    {4.5f, 4.5f, 0.0f, 0, 0, 0, 0.5f},     // cell (0, 0)
    {-4.5f, -4.5f, 0.0f, 0, 0, 0, 0.25f},  // cell (9, 9)
    {0.5f, 1.5f, 0.0f, 0, 0, 0, 1.0f},     // cell (4, 3)
    {6.0f, 0.5f, 0.0f, 0, 0, 0, 0.9f},     // outside in x
    {0.5f, 6.0f, 0.0f, 0, 0, 0, 0.9f},     // outside in y
    {0.005f, 1.0f, 0.0f, 0, 0, 0, 0.9f}});  // on the x axis

  const size_t written =
    accumulatePoints(cloud, ValueSource("reflectivity"), grid_, retained_, now_ns_);

  EXPECT_EQ(written, 3u);
  EXPECT_EQ(knownCells(), 3u);

  EXPECT_EQ(at(retained_.intensity(), 0, 0), 5);     // 0.5 * 10
  EXPECT_EQ(at(retained_.height(), 0, 0), 50);       // 0.5 * 100
  EXPECT_EQ(at(retained_.intensity(), 9, 9), 2);     // 0.25 * 10 = 2.5, truncated
  EXPECT_EQ(at(retained_.height(), 9, 9), 25);
  EXPECT_EQ(at(retained_.intensity(), 4, 3), 10);
  EXPECT_EQ(at(retained_.height(), 4, 3), 100);
  EXPECT_EQ(retained_.stampAt(0, 0), now_ns_);
}

TEST_F(AccumulateTest, SkippedPointsDoNotShiftTheValues)
{
  // the first point is skipped; the second must still get its own value (0.5)
  const auto cloud = makeCloud({
    {6.0f, 0.0f, 0.0f, 0, 0, 0, 0.9f},
    {4.5f, 4.5f, 0.0f, 0, 0, 0, 0.5f}});

  EXPECT_EQ(accumulatePoints(cloud, ValueSource("reflectivity"), grid_, retained_, now_ns_), 1u);
  EXPECT_EQ(at(retained_.intensity(), 0, 0), 5);
}

TEST_F(AccumulateTest, LastPointInACellWins)
{
  const auto cloud = makeCloud({
    {4.5f, 4.5f, 0.0f, 0, 0, 0, 0.25f},
    {4.4f, 4.4f, 0.0f, 0, 0, 0, 1.0f}});   // same cell (0, 0)

  EXPECT_EQ(accumulatePoints(cloud, ValueSource("reflectivity"), grid_, retained_, now_ns_), 2u);
  EXPECT_EQ(knownCells(), 1u);
  EXPECT_EQ(at(retained_.intensity(), 0, 0), 10);
}

TEST_F(AccumulateTest, HeightValueSourceUsesZ)
{
  const auto cloud = makeCloud({{4.5f, 4.5f, 0.5f, 0, 0, 0, 0.0f}});

  EXPECT_EQ(accumulatePoints(cloud, ValueSource("z"), grid_, retained_, now_ns_), 1u);
  EXPECT_EQ(at(retained_.intensity(), 0, 0), 5);
  EXPECT_EQ(at(retained_.height(), 0, 0), 50);
}

TEST_F(AccumulateTest, RgbChannelsAreReadSeparately)
{
  grid_.intensity_factor = 0.5f;
  grid_.height_factor = 1.0f;
  const auto cloud = makeCloud({{4.5f, 4.5f, 0.0f, 10, 20, 30, 0.0f}});

  const struct { const char *field; int expected; } cases[] = {{"r", 10}, {"g", 20}, {"b", 30}};
  for (const auto &c : cases)
  {
    retained_.clear();
    ASSERT_TRUE(retained_.prepare(grid_.cell_num_x, grid_.cell_num_y));
    EXPECT_EQ(accumulatePoints(cloud, ValueSource(c.field), grid_, retained_, now_ns_), 1u);
    EXPECT_EQ(at(retained_.height(), 0, 0), c.expected) << "channel " << c.field;
    EXPECT_EQ(at(retained_.intensity(), 0, 0), c.expected / 2) << "channel " << c.field;
  }
}

TEST_F(AccumulateTest, NothingIsWrittenWhenAllPointsAreOutside)
{
  const auto cloud = makeCloud({{6.0f, 6.0f, 0.0f, 0, 0, 0, 0.5f}});

  EXPECT_EQ(accumulatePoints(cloud, ValueSource("reflectivity"), grid_, retained_, now_ns_), 0u);
  EXPECT_EQ(knownCells(), 0u);
}

TEST_F(AccumulateTest, RetentionKeepsRecentPointsAndDropsOldOnes)
{
  const ValueSource source("reflectivity");
  const auto cloud_a = makeCloud({{4.5f, 4.5f, 0.0f, 0, 0, 0, 0.5f}});    // cell (0, 0)
  const auto cloud_b = makeCloud({{-4.5f, -4.5f, 0.0f, 0, 0, 0, 0.5f}});  // cell (9, 9)

  accumulatePoints(cloud_a, source, grid_, retained_, 0);
  retained_.expire(500000000, kSecond);
  accumulatePoints(cloud_b, source, grid_, retained_, 500000000);
  EXPECT_EQ(knownCells(), 2u);                       // both within 1 s

  retained_.expire(1200000000, kSecond);             // cloud A is now 1.2 s old
  EXPECT_EQ(knownCells(), 1u);
  EXPECT_EQ(at(retained_.intensity(), 0, 0), RetainedGrid::kUnknown);
  EXPECT_EQ(at(retained_.intensity(), 9, 9), 5);
}

// ------------------------------------------------------- message filling

TEST(FillOccupancyGrid, CopiesCellsFrameAndStamp)
{
  nav_msgs::msg::OccupancyGrid grid;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 5;
  stamp.nanosec = 7u;

  fillOccupancyGrid(grid, {1, 2, 3}, "map", stamp);

  EXPECT_EQ(grid.header.frame_id, "map");
  EXPECT_EQ(grid.header.stamp.sec, 5);
  EXPECT_EQ(grid.header.stamp.nanosec, 7u);
  EXPECT_EQ(grid.info.map_load_time.sec, 5);
  EXPECT_EQ(grid.info.map_load_time.nanosec, 7u);
  ASSERT_EQ(grid.data.size(), 3u);
  EXPECT_EQ(grid.data[0], 1);
  EXPECT_EQ(grid.data[2], 3);
}

TEST(FillGridMap, NormalisesCellsToFloats)
{
  GridMap g = makeGrid2x2();
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  g.initGridMap(msg, "intensity");   // sizes the layer to 2 x 2 = 4 cells

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 9;
  stamp.nanosec = 1u;

  ASSERT_TRUE(fillGridMap(*msg, {127, 0, -128, 64}, "map", stamp));

  EXPECT_EQ(msg->header.frame_id, "map");
  EXPECT_EQ(msg->header.stamp.sec, 9);
  ASSERT_EQ(msg->data[0].data.size(), 4u);
  EXPECT_FLOAT_EQ(msg->data[0].data[0], 1.0f);
  EXPECT_FLOAT_EQ(msg->data[0].data[1], 0.0f);
  EXPECT_FLOAT_EQ(msg->data[0].data[2], -128.0f / 127.0f);
  EXPECT_FLOAT_EQ(msg->data[0].data[3], 64.0f / 127.0f);
}

TEST(FillGridMap, RefusesWhenTheLayerSizeDoesNotMatch)
{
  GridMap g = makeGrid2x2();
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  g.initGridMap(msg, "intensity");
  std::fill(msg->data[0].data.begin(), msg->data[0].data.end(), 42.0f);

  EXPECT_FALSE(fillGridMap(*msg, {1, 2, 3}, "map", builtin_interfaces::msg::Time()));
  for (const float v : msg->data[0].data) { EXPECT_FLOAT_EQ(v, 42.0f); }
}

TEST(FillGridMap, RefusesWhenTheLayerWasNeverInitialised)
{
  grid_map_msgs::msg::GridMap msg;   // no initGridMap() call
  EXPECT_FALSE(fillGridMap(msg, {1, 2, 3, 4}, "map", builtin_interfaces::msg::Time()));
}