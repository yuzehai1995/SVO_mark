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

#ifndef SVO_MAP_H_
#define SVO_MAP_H_

#include <queue>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>
#include <svo/global.h>

namespace svo {

class Point;           //声明与#include有什么区别？？
class Feature;
class Seed;

/// Container for converged 3D points that are not already assigned to two keyframes.
class MapPointCandidates
{
public:
  typedef pair<Point*, Feature*> PointCandidate;   //pair是类型
  typedef list<PointCandidate> PointCandidateList;

  /// The depth-filter is running in a parallel thread and fills the canidate list.！！！！
  /// This mutex controls concurrent access to point_candidates.
  boost::mutex mut_;

  /// Candidate points are created from converged seeds.
  /// Until the next keyframe, these points can be used for reprojection and pose optimization.！！！！
  PointCandidateList candidates_;//！！！
  list< Point* > trash_points_;

  MapPointCandidates();
  ~MapPointCandidates();

  /// Add a candidate point.
  void newCandidatePoint(Point* point, double depth_sigma2);	//将point和对应的(在latest关键帧中的)feature加入candidates_.   

  /// Adds the feature to the frame and deletes candidate from list.
  void addCandidatePointToFrame(FramePtr frame);				//遍历candidates_,将frame对应的特征进行frame::addFeature(),同时将该特征点从candidates_中删除。 

  /// Remove a candidate point from the list of candidates.
  bool deleteCandidatePoint(Point* point);

  /// Remove all candidates that belong to a frame.
  void removeFrameCandidates(FramePtr frame);

  /// Reset the candidate list, remove and delete all points.
  void reset();													//用于～MapPointCandidates();

  void deleteCandidate(PointCandidate& c);

  void emptyTrash();//清空trashpoint
};

/// Map object which saves all keyframes which are in a map.
class Map : boost::noncopyable
{
public:
  list< FramePtr > keyframes_;          //!< List of keyframes in the map.
  list< Point* > trash_points_;         //!< A deleted point is moved to the trash bin. Now and then this is cleaned. One reason is that the visualizer must remove the points also.
  MapPointCandidates point_candidates_;

  Map();
  ~Map();

  /// Reset the map. Delete all keyframes and reset the frame and point counters.
  void reset();

  /// Delete a point in the map and remove all references in keyframes to it.
  void safeDeletePoint(Point* pt);//用于removePtFrameRef中。 如果是关键点，将关键点删除，将pt->obs_清空，调用deletePoint删除该点 

  /// Moves the point to the trash queue which is cleaned now and then.
  void deletePoint(Point* pt);

  /// Moves the frame to the trash queue which is cleaned now and then.
  bool safeDeleteFrame(FramePtr frame);//删除关键帧frame。具体来说，调用removePTFramRef,并将该frame从keyframes中erase。 调用point_candidates_.removeFrameCandidates(frame);  

  /// Remove the references between a point and a frame.
  void removePtFrameRef(Frame* frame, Feature* ftr);

  /// Add a new keyframe to the map.
  void addKeyframe(FramePtr new_keyframe);   ////push_back new_keyframe   

  /// Given a frame, return all keyframes which have an overlapping field of view.!!!!!!!!!!!!!!!!!!!
  //遍历keyframes，对每个keyframe中的keypoint测试是否能在当前帧frame中观测到，!!!!!!!!!!!!!!!!!!!!!!!!!
  //如果能，则在close_kfs中pushback该keyframe以及frame和keyframe间的距离。!!!!!!!!!!!!!!!!!!!!!!!!!!!
  void getCloseKeyframes(const FramePtr& frame, list< pair<FramePtr,double> >& close_kfs) const;

  /// Return the keyframe which is spatially closest and has overlapping field of view.
  FramePtr getClosestKeyframe(const FramePtr& frame) const;//对getCloseKeyframes中得到的close_kfs进行排序，选出最近的keyframe。

  /// Return the keyframe which is furthest apart from pos.
  FramePtr getFurthestKeyframe(const Vector3d& pos) const;//遍历keyframes，通过keyframes->pos(),计算与pos最远的关键帧。  

  bool getKeyframeById(const int id, FramePtr& frame) const;

  /// Transform the whole map with rotation R, translation t and scale s.
  //计算经R，t，s变化后每个keyframe的T_f_w, 同时将每个keyframe中的fts所对应的3D点进行变换。         
  void transform(const Matrix3d& R, const Vector3d& t, const double& s);
	
  /// Empty trash bin of deleted keyframes and map points. We don't delete the
  /// points immediately to ensure proper cleanup and to provide the visualizer
  /// a list of objects which must be removed.
  void emptyTrash();

  /// Return the keyframe which was last inserted in the map.
  inline FramePtr lastKeyframe() { return keyframes_.back(); }

  /// Return the number of keyframes in the map
  inline size_t size() const { return keyframes_.size(); }
};

/// A collection of debug functions to check the data consistency.
namespace map_debug {//用于调试

void mapStatistics(Map* map);//计算每个关键帧平均观测到多少个点，每个点平均被多少个关键帧观测到
void mapValidation(Map* map, int id);//计算每个关键帧平均观测到多少个点，每个点平均被多少个关键帧观测到
void frameValidation(Frame* frame, int id);
void pointValidation(Point* point, int id);

} // namespace map_debug
} // namespace svo

#endif // SVO_MAP_H_
