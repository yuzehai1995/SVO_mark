// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_FEATURE_DETECTION_H_
#define SVO_FEATURE_DETECTION_H_

#include <svo/global.h>
#include <svo/frame.h>

namespace svo {

/// Implementation of various feature detectors.
namespace feature_detection {

/// Temporary container used for corner detection. Features are initialized from these.
struct Corner  //角点
{
  int x;        //!< x-coordinate of corner in the image.
  int y;        //!< y-coordinate of corner in the image.
  int level;    //!< pyramid level of the corner.
  float score;  //!< shi-tomasi score of the corner.
  float angle;  //!< for gradient-features: dominant gradient angle.
  Corner(int x, int y, float score, int level, float angle) :
    x(x), y(y), level(level), score(score), angle(angle)
  {}
};
typedef vector<Corner> Corners;

/// All detectors should derive from this abstract class.
class AbstractDetector
{
public:
  AbstractDetector(
      const int img_width,
      const int img_height,
      const int cell_size, //每个网格的尺寸
      const int n_pyr_levels  //金字塔层数？
      );

  virtual ~AbstractDetector() {};

  virtual void detect(
      Frame* frame,
      const ImgPyr& img_pyr,
      const double detection_threshold,
      Features& fts) = 0;

  /// Flag the grid cell as occupied
  void setGridOccpuancy(const Vector2d& px);  ////如果点px在某一个cell中，将该cell对应的grid_occupanvy_置为true

  /// Set grid cells of existing features as occupied
  void setExistingFeatures(const Features& fts);// 遍历fts,如果fts的像素坐标在某一个cell中，就将对应的grid_occupancy置为true。

protected:

  static const int border_ = 8; //!< no feature should be within 8px of border.
  const int cell_size_;			//// 将一张图像划分成很多网格，每个网格的大小（边长）
  const int n_pyr_levels_;
  const int grid_n_cols_;		//网格的列数
  const int grid_n_rows_;		//网格的行数

  vector<bool> grid_occupancy_; //用于记录每个网格是否已经有特征点落入其中。大小为网格的数量，


  void resetGrid();

  inline int getCellIndex(int x, int y, int level)		//对不同层的点xy，确定其在第0层的cell的索引   ？？？？？
  {
    const int scale = (1<<level);						//level是层数？  1<<level是右移操作
    return (scale*y)/cell_size_*grid_n_cols_ + (scale*x)/cell_size_;
  }
};
typedef boost::shared_ptr<AbstractDetector> DetectorPtr;

/// FAST detector by Edward Rosten.
class FastDetector : public AbstractDetector
{
public:
  FastDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  virtual ~FastDetector() {}

  virtual void detect(
      Frame* frame,
      const ImgPyr& img_pyr,
      const double detection_threshold,
      Features& fts);
};

} // namespace feature_detection
} // namespace svo

#endif // SVO_FEATURE_DETECTION_H_
