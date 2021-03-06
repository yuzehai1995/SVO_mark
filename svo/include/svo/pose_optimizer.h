//
//用BA来优化位姿。迭代优化结束后，用优化后的位姿来计算重投影误差，误差大的点删除,
//得到最终观测到的点num_obs。记录esitmated_scale,error_init,error_final,num_obs.  
//
#ifndef SVO_POSE_OPTIMIZER_H_
#define SVO_POSE_OPTIMIZER_H_

#include <svo/global.h>

namespace svo {

using namespace Eigen;
using namespace Sophus;
using namespace std;

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,2,6> Matrix26d;
typedef Matrix<double,6,1> Vector6d;

class Point;

/// Motion-only bundle adjustment. Minimize the reprojection error of a single frame.
namespace pose_optimizer {

void optimizeGaussNewton(
    const double reproj_thresh,
    const size_t n_iter,
    const bool verbose,
    FramePtr& frame,
    double& estimated_scale,
    double& error_init,
    double& error_final,
    size_t& num_obs);

} // namespace pose_optimizer
} // namespace svo

#endif // SVO_POSE_OPTIMIZER_H_
