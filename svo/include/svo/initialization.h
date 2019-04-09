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

  //����detectFeatures,����⵽����������px_ref_�С������⵽��������С��100���򷵻�FAILURE���������100��
  //��frame_ref��Ϊframe_ref_,��px_ref_�е���������px_cur_;
  InitResult addFirstFrame(FramePtr frame_ref);

  //����trackKlt��px_ref���и��١���������С��һ��ֵ������FAILURE; disparity�ľ�ֵС��һ��ֵ��˵����̫֡��������
  //NO_KEYFRAME�� ����computeHomography�����inliers����С��һ��ֵ������FAULURE�� ����ɸѡ������ͨ����
  //��xyz_in_cur�������ڵ�ǰ֡�еĵ��ƽ����ȣ�������߶����ӣ�Ȼ�������ǰ֡��λ�ˡ���ÿһ��inliers��
  //����������������ϵ�µ�3D�����꣬������Ӧ��feature���뵽ref_frame��cur_frame�У�ͬʱ��feature����point->obs_�У� ����SUCCESS��
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
void detectFeatures(////��frame����fastdetect���õ�new_features,Ȼ�����Ӧ��px��f����px_vec,f_vec                
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec);

/// Compute optical flow (Lucas Kanade) for selected keypoints.
void trackKlt(		//��opencv�Դ���������KLT������׷��ʧ�ܵĵ㣬���px_cur,px_ref,f_ref��Ĩȥ������׷�ٳɹ��ĵ㣬
					//�����px_cur,f_cur�У�������disparities��
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities);

void computeHomography(//��f_ref,f_cur�õ��������꣬Ȼ��ͨ��vk::Homography������֡�����λ��T_c_f,����
						//vk::computeInliers����inliers��xyz_in_cur. 
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
