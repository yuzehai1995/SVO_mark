//
//����ӵ�ͼ�ؼ���(overlap_kfs)ͶӰ��ͼ�񣬲�Ѱ����Ӧ��������
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

  Reprojector(vk::AbstractCamera* cam, Map& map);////��ʼ��map������initializeGrid

  ~Reprojector();


  /// Project points from the map into the image. First finds keyframes with
  /// overlapping field of view and projects only those map-points.
  //����map_.getcloseKeyframes�������frame����Ĺؼ�֡�������ǰ�������С�������򣬵õ������N���ؼ�֡��
  //��ÿ���ؼ�֡��Ӧ��fts_��reprojectPoint��ͶӰ��frame�С�Ȼ��map_�е�candidates��reprojectPoint��ͶӰ��frame�С�
  //��󣬶���frame��ÿ��cell�������ͶӰ�㡣��reprojectCell��ÿ��cellѡ��һ���㡣  
  void reprojectMap(
      FramePtr frame,
      std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs);

private:

  /// A candidate is a point that projects into the image plane and for which we��������������������
  /// will search a maching feature in the image.����������������������������������������������������
  struct Candidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Point* pt;       //!< 3D point.
    Vector2d px;     //!< projected 2D pixel location.
    Candidate(Point* pt, Vector2d& px) : pt(pt), px(px) {}
  };
  typedef std::list<Candidate, aligned_allocator<Candidate> > Cell;//typedef  cell
  typedef std::vector<Cell*> CandidateGrid;						   //typedef  CandidateGrid������cell��������

  /// The grid stores a set of candidate matches. For every grid cell we try to find one match.������
  struct Grid    //����candidate������˵cell������
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
  void initializeGrid(vk::AbstractCamera* cam);//��ʼ��grid��С��cells��С����������cell_order��ֵ�����˳��          
  void resetGrid();

  //����ĳһ��cell�����ܻ��кܶ�points�����С�����Щpoints�������ͺû���������Ȼ�������Щpoints��
  //��ÿ��point����matcher_.findMatchDirect��У׼��point�� ����һ��pointͨ����ɸѡ����
  //����point��cell��ɾ��(��Ϊ�����Ǻ�ѡ����)��ͬʱreturn true��
  bool reprojectCell(Cell& cell, FramePtr frame);

  //��point����ͶӰ���õ�px�����px��frame�У�����������ĸ�cell�У�
  //Ȼ��grid_.cells.at(k)->push_back(Candidate(point, px));
  bool reprojectPoint(FramePtr frame, Point* point); 


};

} // namespace svo

#endif // SVO_REPROJECTION_H_
