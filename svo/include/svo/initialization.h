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

#ifndef SVO_INITIALIZATION_H
#define SVO_INITIALIZATION_H

#include <svo/global.h>

namespace svo {

class FrameHandlerMono;

/// Bootstrapping the map from the first two views.
namespace initialization {

enum InitResult { FAILURE, NO_KEYFRAME, SUCCESS };

/// Tracks features using Lucas-Kanade tracker and then estimates a homography.
class KltHomographyInit {
  friend class svo::FrameHandlerMono;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FramePtr frame_ref_;
  KltHomographyInit() {};
  ~KltHomographyInit() {};

  //调用detectFeatures,将检测到的特征存入px_ref_中。如果检测到的特征数小于100，则返回FAILURE，如果大于100，
  //则将frame_ref存为frame_ref_,将px_ref_中的特征存入px_cur_;
  InitResult addFirstFrame(FramePtr frame_ref);

  //调用trackKlt对px_ref进行跟踪。跟踪数量小于一定值，返回FAILURE; disparity的均值小于一定值，说明两帧太近，返回
  //NO_KEYFRAME。 调用computeHomography。如果inliers数量小于一定值，返回FAULURE。 以上筛选条件都通过后，
  //用xyz_in_cur来计算在当前帧中的点的平均深度，用于求尺度因子，然后算出当前帧的位姿。对每一个inliers，
  //计算它在世界坐标系下的3D点坐标，并将对应的feature加入到ref_frame和cur_frame中，同时将feature加入point->obs_中， 返回SUCCESS。
  InitResult addSecondFrame(FramePtr frame_ref);

  void reset();

protected:
  vector<cv::Point2f> px_ref_;      //!< keypoints to be tracked in reference frame.
  vector<cv::Point2f> px_cur_;      //!< tracked keypoints in current frame.
  vector<Vector3d> f_ref_;          //!< bearing vectors corresponding to the keypoints in the reference image.
  vector<Vector3d> f_cur_;          //!< bearing vectors corresponding to the keypoints in the current image.
  vector<double> disparities_;      //!< disparity between first and second frame.
  vector<int> inliers_;             //!< inliers after the geometric check (e.g., Homography).
  vector<Vector3d> xyz_in_cur_;     //!< 3D points computed during the geometric check.
  SE3 T_cur_from_ref_;              //!< computed transformation between the first two frames.
};

/// Detect Fast corners in the image.
void detectFeatures(////对frame进行fastdetect，得到new_features,然后将其对应的px和f存入px_vec,f_vec                
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec);

/// Compute optical flow (Lucas Kanade) for selected keypoints.
void trackKlt(		//用opencv自带函数进行KLT，对于追踪失败的点，会从px_cur,px_ref,f_ref中抹去，对于追踪成功的点，
					//会存在px_cur,f_cur中，并计算disparities。
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities);

void computeHomography(//从f_ref,f_cur得到像素坐标，然后通过vk::Homography计算两帧的相对位姿T_c_f,并用
						//vk::computeInliers计算inliers和xyz_in_cur. 
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref);

} // namespace initialization
} // namespace nslam

#endif // NSLAM_INITIALIZATION_H
