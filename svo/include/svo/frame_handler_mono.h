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

#ifndef SVO_FRAME_HANDLER_H_
#define SVO_FRAME_HANDLER_H_

#include <set>
#include <vikit/abstract_camera.h>
#include <svo/frame_handler_base.h>
#include <svo/reprojector.h>
#include <svo/initialization.h>

namespace svo {

/// Monocular Visual Odometry Pipeline as described in the SVO paper.
class FrameHandlerMono : public FrameHandlerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  FrameHandlerMono(vk::AbstractCamera* cam);
  virtual ~FrameHandlerMono();

  /// Provide an image.
  //先清理core_kfs,overlap_kfs. 然后用传入的图片初始化new_frame. 
  //然后通过系统所处的不同stage_进行不同处理，然后last_frame_=new_frame_;new_frame_.reset().
  void addImage(const cv::Mat& img, double timestamp);


  /// Set the first frame (used for synthetic datasets in benchmark node)
  void setFirstFrame(const FramePtr& first_frame);
  // resetAll();last_frame_ = first_frame; last_frame_->setKeyframe
  //map_.addKeyframe(last_frame_);stage_ = STAGE_DEFAULT_FRAME;    


  /// Get the last frame that has been processed.
  FramePtr lastFrame() { return last_frame_; }

  /// Get the set of spatially closest keyframes of the last frame.
  const set<FramePtr>& coreKeyframes() { return core_kfs_; }

  /// Return the feature track to visualize the KLT tracking during initialization.
  const vector<cv::Point2f>& initFeatureTrackRefPx() const { return klt_homography_init_.px_ref_; }
  const vector<cv::Point2f>& initFeatureTrackCurPx() const { return klt_homography_init_.px_cur_; }

  /// Access the depth filter.
  DepthFilter* depthFilter() const { return depth_filter_; }

  /// An external place recognition module may know where to relocalize.
  //通过keyframe_id,用map_中的getKeyframeById得到ref_keyframe.通过img重新初始化一个new_frame_. 
  //然后调用relocalizeFrame。只要返回结果不是REUSLT_FAILURE,说明重定位成功，将last_frame_=new_frame_, return true.
  bool relocalizeFrameAtPose(
      const int keyframe_id,
      const SE3& T_kf_f,
      const cv::Mat& img,
      const double timestamp);

protected:
  vk::AbstractCamera* cam_;                     //!< Camera model, can be ATAN, Pinhole or Ocam (see vikit).
  Reprojector reprojector_;                     //!< Projects points from other keyframes into the current frame
  FramePtr new_frame_;                          //!< Current frame.
  FramePtr last_frame_;                         //!< Last frame, not necessarily a keyframe.
  set<FramePtr> core_kfs_;                      //!< Keyframes in the closer neighbourhood.
  vector< pair<FramePtr,size_t> > overlap_kfs_; //!< All keyframes with overlapping field of view. the paired number specifies how many common mappoints are observed TODO: why vector!?
  initialization::KltHomographyInit klt_homography_init_; //!< Used to estimate pose of the first two keyframes by estimating a homography.
  DepthFilter* depth_filter_;                   //!< Depth estimation algorithm runs in a parallel thread and is used to initialize new 3D points.

  /// Initialize the visual odometry algorithm.
  virtual void initialize();//初始化vo

  /// Processes the first frame and sets it as a keyframe.
  //将第一帧的位姿设为单位阵作为参考。调用initialize中的addFirstFrame来提取特征点，如果成功，设置关键帧，stage_=STAGE_SECOND_FRAME  
  virtual UpdateResult processFirstFrame();


  /// Processes all frames after the first frame until a keyframe is selected.
  //调用initialize中的addSecondFrame来跟踪第一帧的特征点，并计算出特征点对应的3D坐标。将该帧new_frame_设为关键帧。
  //通过frame_utils中的getSceneDepth来计算new_frame_中的场景深度值，并调用depth_filter和map_中的addKeyFrame，  
  virtual UpdateResult processSecondFrame();

  /// Processes all frames after the first two keyframes.
  // 用上一帧的位姿初始化new_frame_的位姿。然后依次进行sparse_image_align, map rprojector&feature alignment, 
  //pose optimization, structure optimization.然后判断new_frame_是否是关键帧，如果不是，
  //该帧用于depth_filter中updateSeeds, 如果是，则设为关键帧，并判断删除旧的关键帧。
  virtual UpdateResult processFrame();


  /// Try relocalizing the frame at relative position to provided keyframe.
  //通过ref_keyframe来和new_frame_进行sparse_image_alignment    如果返回的追踪上的点的数量大于30，则将
  //ref_keyframe设为last_frame, 进行processFrame(), 只要返回值不是RESULT_FAILURE,就说明重定位成功。
  //如果小于30，则重定位失败。
  virtual UpdateResult relocalizeFrame(
      const SE3& T_cur_ref,
      FramePtr ref_keyframe);

  /// Reset the frame handler. Implement in derived class.
  virtual void resetAll();

  /// Keyframe selection criterion.
  //通过遍历overlap_kfs_来计算每个overlap_kf与new_frame的相对距离，如果这个距离大于一定的值，则返回true
  virtual bool needNewKf(double scene_depth_mean);

  void setCoreKfs(size_t n_closest);//从overlap_kfs_中选出与new_frame_共视点最多的n_closest个关键帧作为core_kfs_ 
};

} // namespace svo

#endif // SVO_FRAME_HANDLER_H_
