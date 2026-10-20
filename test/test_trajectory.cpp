#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "pointcloud_to_grid/trajectory.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class TrajectoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    grid_.position_x = 0.0f;
    grid_.position_y = 0.0f;
    grid_.cell_size = 1.0f;
    grid_.length_x = 10.0f;
    grid_.length_y = 10.0f;
    grid_.paramRefresh();
    cells_.assign(100, 0);
  }

  size_t drawnCount() const
  {
    return std::count_if(cells_.begin(), cells_.end(), [](signed char c) { return c != 0; });
  }

  GridMapTrajectory grid_{};
  std::vector<signed char> cells_;
};

TEST_F(TrajectoryTest, DrawsStraightLineAlongNegativeX)
{
  // from cell (0,0) to cell (3,0)
  EXPECT_TRUE(grid_.drawline(cells_, 4.5, 4.5, kPi, 3.0));
  for (int x = 0; x <= 3; ++x) { EXPECT_EQ(cells_[x], 1) << "cell x=" << x; }
  EXPECT_EQ(drawnCount(), 4u);
}

TEST_F(TrajectoryTest, DrawsExactDiagonal)
{
  // from cell (0,0) to cell (3,3)
  EXPECT_TRUE(grid_.drawline(cells_, 4.5, 4.5, 5.0 * kPi / 4.0, 3.0 * std::sqrt(2.0)));
  for (int i = 0; i <= 3; ++i) { EXPECT_EQ(cells_[i * 10 + i], 1) << "diagonal cell " << i; }
  EXPECT_EQ(drawnCount(), 4u);
}

TEST_F(TrajectoryTest, ZeroLengthDrawsSingleCell)
{
  EXPECT_TRUE(grid_.drawline(cells_, 4.5, 4.5, 0.0, 0.0));
  EXPECT_EQ(cells_[0], 1);
  EXPECT_EQ(drawnCount(), 1u);
}

TEST_F(TrajectoryTest, OccupiedCellOnPathBlocksTheLine)
{
  cells_[2] = 100;
  EXPECT_FALSE(grid_.drawline(cells_, 4.5, 4.5, kPi, 3.0));
  EXPECT_EQ(cells_[2], 100);   // the obstacle is left untouched
  EXPECT_EQ(cells_[3], 0);     // nothing is drawn beyond it
}

TEST_F(TrajectoryTest, OccupiedStartCellBlocksImmediately)
{
  cells_[0] = 100;
  EXPECT_FALSE(grid_.drawline(cells_, 4.5, 4.5, kPi, 3.0));
  EXPECT_EQ(cells_[1], 0);
}

TEST_F(TrajectoryTest, ThresholdIsStrictlyAboveFifty)
{
  cells_[2] = 50;   // still free
  EXPECT_TRUE(grid_.drawline(cells_, 4.5, 4.5, kPi, 3.0));

  cells_.assign(100, 0);
  cells_[2] = 51;   // occupied
  EXPECT_FALSE(grid_.drawline(cells_, 4.5, 4.5, kPi, 3.0));
}

TEST_F(TrajectoryTest, LineEndingOutsideGridIsClipped)
{
  // end point is far beyond the -x edge; only the 10 cells of row 0 exist
  EXPECT_TRUE(grid_.drawline(cells_, 4.5, 4.5, kPi, 20.0));
  for (int x = 0; x < 10; ++x) { EXPECT_EQ(cells_[x], 1) << "cell x=" << x; }
  EXPECT_EQ(drawnCount(), 10u);
  EXPECT_EQ(cells_.size(), 100u);
}

TEST_F(TrajectoryTest, LineMovingAwayFromGridDrawsOnlyStartCell)
{
  // theta = 0 points towards +x, out of the grid. The old getIndex mirrored
  // the end point back into the grid and drew 8 cells; now only the start
  // cell is inside the grid.
  EXPECT_TRUE(grid_.drawline(cells_, 4.5, 4.5, 0.0, 8.0));
  EXPECT_EQ(cells_[0], 1);
  EXPECT_EQ(drawnCount(), 1u);
}