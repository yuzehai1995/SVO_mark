//
//
//

#ifndef SVO_MATCHER_H_
#define SVO_MATCHER_H_

#include <svo/global.h>

namespace vk {
  class AbstractCamera;
  namespace patch_score {
    template<int HALF_PATCH_SIZE> class ZMSSD;
  }
}

namespace svo {

class Point;
class Frame;
class Feature;

/// Warp a patch from the reference view to the current view.
namespace warp {

void getWarpMatrixAffine(   ////计算仿射矩阵A_cur_ref
    const vk::AbstractCamera& cam_ref,
    const vk::AbstractCamera& cam_cur,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,
    Matrix2d& A_cur_ref);//仿射矩阵A_cur_ref

int getBestSearchLevel(
    const Matrix2d& A_cur_ref,
    const int max_level);

void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int level_cur,
    const int halfpatch_size,
    uint8_t* patch);		////对参考帧像素对应的patch进行仿射变化， uint8_t *patch 依次指向仿射变化后像素对应的值.

} // namespace warp

/// Patch-matcher for reprojection-matching and epipolar search in triangulation.
class Matcher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int halfpatch_size_ = 4;
  static const int patch_size_ = 8;

  typedef vk::patch_score::ZMSSD<halfpatch_size_> PatchScore;

  struct Options
  {
    bool align_1d;              //!< in epipolar search: align patch 1D along epipolar line
    int align_max_iter;         //!< number of iterations for aligning the feature patches in gauss newton
    double max_epi_length_optim;//!< max length of epipolar line to skip epipolar search and directly go to img align
								//当极线长度小于这个值时，不用进行极线搜索，直接进行img_align

    size_t max_epi_search_steps;//!< max number of evaluations along epipolar line
    bool subpix_refinement;     //!< do gauss newton feature patch alignment after epipolar search
    bool epi_search_edgelet_filtering;
    double epi_search_edgelet_max_angle;
    Options() :
      align_1d(false),
      align_max_iter(10),
      max_epi_length_optim(2.0),
      max_epi_search_steps(1000),
      subpix_refinement(true),
      epi_search_edgelet_filtering(true),
      epi_search_edgelet_max_angle(0.7)
    {}
  } options_;

  uint8_t patch_[patch_size_*patch_size_] __attribute__ ((aligned (16)));
  uint8_t patch_with_border_[(patch_size_+2)*(patch_size_+2)] __attribute__ ((aligned (16)));
  Matrix2d A_cur_ref_;          //!< affine warp matrix
  Vector2d epi_dir_;
  double epi_length_;           //!< length of epipolar line segment in pixels (only used for epipolar search)
  double h_inv_;                //!< hessian of 1d image alignment along epipolar line
  int search_level_;
  bool reject_;
  Feature* ref_ftr_;
  Vector2d px_cur_;

  Matcher() = default;
  ~Matcher() = default;

  /// Find a match by directly applying subpix refinement.
  /// IMPORTANT! This function assumes that px_cur is already set to an estimate that is within ~2-3 pixel of the final result!
  bool findMatchDirect(
      const Point& pt,
      const Frame& frame,
      Vector2d& px_cur);

  /// Find a match by searching along the epipolar line without using any features.
  bool findEpipolarMatchDirect(
      const Frame& ref_frame,
      const Frame& cur_frame,
      const Feature& ref_ftr,
      const double d_estimate,
      const double d_min,
      const double d_max,
      double& depth);////如果极线长度小于2，则px_cur为 （px_A+px_B)/2; otherwise,进行极线搜索，在极线上每间隔一段距离采样，然后计算该采样点对应的patch和ref的相似度，对相似度进行打分，选择分值最高的。 如果option.subpixle_refine=true, 则进行feature_align。   


  void createPatchFromPatchWithBorder();
};

} // namespace svo

#endif // SVO_MATCHER_H_
