//
//优化帧的位姿，方法是最小化特征块的光度误差
//
#ifndef SVO_SPARSE_IMG_ALIGN_H_
#define SVO_SPARSE_IMG_ALIGN_H_

#include <vikit/nlls_solver.h>
#include <vikit/performance_monitor.h>
#include <svo/global.h>

namespace vk {
class AbstractCamera;
}

namespace svo {

class Feature;

/// Optimize the pose of the frame by minimizing the photometric error of feature patches.
class SparseImgAlign : public vk::NLLSSolver<6, SE3>
{
  static const int patch_halfsize_ = 2;        //？？？？？？？？？
  static const int patch_size_ = 2*patch_halfsize_;
  static const int patch_area_ = patch_size_*patch_size_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cv::Mat resimg_;

  SparseImgAlign(   //初始化。其中n_iter,method,verbose用于初始化vk::NLLSSolver的相关成员变量。n_levels对应max_level。   
      int n_levels,
      int min_level,
      int n_iter,
      Method method,
      bool display,
      bool verbose);

  size_t run(
      FramePtr ref_frame,
      FramePtr cur_frame);

  /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
  /// at the converged state.
  Matrix<double, 6, 6> getFisherInformation();

protected:
  FramePtr ref_frame_;            //!< reference frame, has depth for gradient pixels.
  FramePtr cur_frame_;            //!< only the image is known!
  int level_;                     //!< current pyramid level on which the optimization runs.
  bool display_;                  //!< display residual image.
  int max_level_;                 //!< coarsest pyramid level for the alignment.
  int min_level_;                 //!< finest pyramid level for the alignment.

  // cache:
  Matrix<double, 6, Dynamic, ColMajor> jacobian_cache_;////列数为ref_patch_cache的大小。每一列为一个#feature所在的像素#对应的jacobian。
  bool have_ref_patch_cache_;
  cv::Mat ref_patch_cache_;
  std::vector<bool> visible_fts_;

  //计算ref中每个fts对应的patch的雅克比。将所有的雅克比存在jacobian_cahe_中。
  //并将have_ref_patch_cache_置为true（在执行computeResiduals中，需要保证该步已经执行）    
  void precomputeReferencePatches();

  //以下的5个函数是对其父类NLSSLOVE对应成员函数，但是在NLSSOLVE中没有写具体的实现，是纯虚函数，
  //在该类中对这些纯虚函数进行实现！！   ！！！！ 

  ////直接法的目标函数的构造。包括将ref中3D点转化到cur中，再投影到cur中，然后计算两帧中对应点的光度误差；计算H和b
  virtual double computeResiduals(const SE3& model, bool linearize_system, bool compute_weight_scale = false);

  virtual int solve();//// 用ldlt来求解Hx=b;

  virtual void update (const ModelType& old_model, ModelType& new_model);////更新相对位姿 
  virtual void startIteration();
  virtual void finishIteration();
};

} // namespace svo

#endif // SVO_SPARSE_IMG_ALIGN_H_
