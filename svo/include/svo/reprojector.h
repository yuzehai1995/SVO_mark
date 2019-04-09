//
//将点从地图关键点(overlap_kfs)投影到图像，并寻找相应的特征点
//
//

#ifndef SVO_REPROJECTION_H_
#define SVO_REPROJECTION_H_

#include <svo/global.h>
#include <svo/matcher.h>

namespace vk {
class AbstractCamera;
}

namespace svo {

class Map;
class Point;

/// Project points from the map into the image and find the corresponding
/// feature (corner). We don't search a match for every point but only for one
/// point per cell. Thereby, we achieve a homogeneously distributed set of
/// matched features and at the same time we can save processing time by not
/// projecting all points.
class Reprojector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Reprojector config parameters
  struct Options {
    size_t max_n_kfs;   //!< max number of keyframes to reproject from
    bool find_match_direct;
    Options()
    : max_n_kfs(10),
      find_match_direct(true)
    {}
  } options_;

  size_t n_matches_;
  size_t n_trials_;

  Reprojector(vk::AbstractCamera* cam, Map& map);////初始化map，调用initializeGrid

  ~Reprojector();


  /// Project points from the map into the image. First finds keyframes with
  /// overlapping field of view and projects only those map-points.
  //先用map_.getcloseKeyframes来获得与frame相近的关键帧，将它们按距离由小到大排序，得到最近的N个关键帧。
  //将每个关键帧对应的fts_用reprojectPoint重投影到frame中。然后将map_中的candidates用reprojectPoint重投影到frame中。
  //最后，对于frame，每个cell中有许多投影点。用reprojectCell对每个cell选出一个点。  
  void reprojectMap(
      FramePtr frame,
      std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs);

private:

  /// A candidate is a point that projects into the image plane and for which we！！！！！！！！！！
  /// will search a maching feature in the image.！！！！！！！！！！！！！！！！！！！！！！！！！！
  struct Candidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Point* pt;       //!< 3D point.
    Vector2d px;     //!< projected 2D pixel location.
    Candidate(Point* pt, Vector2d& px) : pt(pt), px(px) {}
  };
  typedef std::list<Candidate, aligned_allocator<Candidate> > Cell;//typedef  cell
  typedef std::vector<Cell*> CandidateGrid;						   //typedef  CandidateGrid（就是cell的容器）

  /// The grid stores a set of candidate matches. For every grid cell we try to find one match.！！！
  struct Grid    //就是candidate（或者说cell）集合
  {
    CandidateGrid cells;
    vector<int> cell_order;
    int cell_size;
    int grid_n_cols;
    int grid_n_rows;
  };

  Grid grid_;
  Matcher matcher_;
  Map& map_;

  static bool pointQualityComparator(Candidate& lhs, Candidate& rhs);
  void initializeGrid(vk::AbstractCamera* cam);//初始化grid大小，cells大小和数量，给cell_order赋值后打乱顺序          
  void resetGrid();

  //对于某一个cell，可能会有很多points在其中。对这些points按照类型好坏进行排序，然后遍历这些points，
  //对每个point进行matcher_.findMatchDirect来校准该point。 当有一个point通过了筛选，则
  //将该point从cell中删除(因为不再是候选点了)，同时return true。
  bool reprojectCell(Cell& cell, FramePtr frame);

  //将point进行投影，得到px，如果px在frame中，则计算落在哪个cell中，
  //然后grid_.cells.at(k)->push_back(Candidate(point, px));
  bool reprojectPoint(FramePtr frame, Point* point); 


};

} // namespace svo

#endif // SVO_REPROJECTION_H_
