/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include "message_filters/subscriber.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

#include "line_segment_mapping/g2o_solver.h"
#include "line_segment_mapping/line_segment_extractor.h"
#include "line_segment_mapping/line_segment_mapper.h"
#include "line_segment_mapping/util.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto {
 public:
  SlamKarto();
  ~SlamKarto();

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool mapCallback(nav_msgs::GetMap::Request& req,    // NOLINT
                   nav_msgs::GetMap::Response& res);  // NOLINT

 private:
  bool getOdomPose(karto::Pose2* karto_pose, const ros::Time& t);
  karto::LaserRangeFinder* getLaser(
      const sensor_msgs::LaserScan::ConstPtr& scan);
  bool addScan(karto::LaserRangeFinder* laser,
               const sensor_msgs::LaserScan::ConstPtr& scan,
               karto::Pose2* karto_pose);
  bool updateMap();
  void publishTransform();
  void publishLoop(double transform_publish_period);
  // void publishGraphVisualization();
  void publishLineSegmentMapVisualization();

  // ROS handles
  ros::NodeHandle node_;
  tf::TransformListener tf_;
  tf::TransformBroadcaster* tfB_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
  ros::Publisher sst_;
  ros::Publisher marker_publisher_;
  ros::Publisher sstm_;
  ros::Publisher line_segment_map_marker_publisher_;
  ros::ServiceServer ss_;

  // The map that will be published / send to service callers
  nav_msgs::GetMap::Response map_;

  // Storage for ROS parameters
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  int throttle_scans_;
  ros::Duration map_update_interval_;
  double resolution_;
  boost::mutex map_mutex_;
  boost::mutex map_to_odom_mutex_;

  // Karto bookkeeping
  std::unique_ptr<karto::LineSegmentMapper> line_segment_mapper_;
  std::unique_ptr<karto::LineSegmentExtractor> line_segment_extractor_;
  std::unique_ptr<karto::Dataset> dataset_;
  std::unique_ptr<G2oSolver> solver_;

  std::map<std::string, karto::LaserRangeFinder*> lasers_;
  std::map<std::string, bool> lasers_inverted_;

  // Internal state
  bool got_map_ = false;
  int laser_count_ = 0;
  boost::thread* transform_thread_;
  tf::Transform map_to_odom_;
  bool inverted_laser_;
  double range_threshold_;
};

SlamKarto::SlamKarto() {
  map_to_odom_.setIdentity();
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if (!private_nh_.getParam("odom_frame", odom_frame_)) {
    odom_frame_ = "odom";
  }
  if (!private_nh_.getParam("map_frame", map_frame_)) {
    map_frame_ = "map";
  }
  if (!private_nh_.getParam("base_frame", base_frame_)) {
    base_frame_ = "base_link";
  }
  if (!private_nh_.getParam("scan_topic", scan_topic_)) {
    scan_topic_ = "base_scan";
  }
  if (!private_nh_.getParam("throttle_scans", throttle_scans_)) {
    throttle_scans_ = 1;
  }
  double tmp;
  if (!private_nh_.getParam("map_update_interval", tmp)) {
    tmp = 5.0;
  }
  map_update_interval_.fromSec(tmp);
  if (!private_nh_.getParam("resolution", resolution_)) {
    // Compatibility with slam_gmapping, which uses "delta" to mean
    // resolution
    if (!private_nh_.getParam("delta", resolution_)) {
      resolution_ = 0.05;
    }
  }
  if (!private_nh_.getParam("range_threshold", range_threshold_)) {
    range_threshold_ = 12.0;
  }
  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(
      node_, scan_topic_, 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(
      *scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(
      boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 1);
  line_segment_map_marker_publisher_ =
      node_.advertise<visualization_msgs::Marker>("line_markers", 1);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = new boost::thread(
      boost::bind(&SlamKarto::publishLoop, this, transform_publish_period));

  // Initialize Karto structures
  line_segment_mapper_ = std::make_unique<karto::LineSegmentMapper>();
  line_segment_extractor_ = std::make_unique<karto::LineSegmentExtractor>();
  dataset_ = std::make_unique<karto::Dataset>();

  // Setting General Parameters from the Parameter Server
  bool use_scan_matching;
  if (private_nh_.getParam("use_scan_matching", use_scan_matching)) {
    line_segment_mapper_->setParamUseScanMatching(use_scan_matching);
  }

  bool use_scan_barycenter;
  if (private_nh_.getParam("use_scan_barycenter", use_scan_barycenter)) {
    line_segment_mapper_->setParamUseScanBarycenter(use_scan_barycenter);
  }

  double minimum_travel_distance;
  if (private_nh_.getParam("minimum_travel_distance",
                           minimum_travel_distance)) {
    line_segment_mapper_->setParamMinimumTravelDistance(
        minimum_travel_distance);
  }

  double minimum_travel_heading;
  if (private_nh_.getParam("minimum_travel_heading", minimum_travel_heading)) {
    line_segment_mapper_->setParamMinimumTravelHeading(minimum_travel_heading);
  }

  int scan_buffer_size;
  if (private_nh_.getParam("scan_buffer_size", scan_buffer_size)) {
    line_segment_mapper_->setParamScanBufferSize(scan_buffer_size);
  }

  double scan_buffer_maximum_scan_distance;
  if (private_nh_.getParam("scan_buffer_maximum_scan_distance",
                           scan_buffer_maximum_scan_distance)) {
    line_segment_mapper_->setParamScanBufferMaximumScanDistance(
        scan_buffer_maximum_scan_distance);
  }

  double link_match_minimum_response_fine;
  if (private_nh_.getParam("link_match_minimum_response_fine",
                           link_match_minimum_response_fine)) {
    line_segment_mapper_->setParamLinkMatchMinimumResponseFine(
        link_match_minimum_response_fine);
  }

  double link_scan_maximum_distance;
  if (private_nh_.getParam("link_scan_maximum_distance",
                           link_scan_maximum_distance)) {
    line_segment_mapper_->setParamLinkScanMaximumDistance(
        link_scan_maximum_distance);
  }

  double loop_search_maximum_distance;
  if (private_nh_.getParam("loop_search_maximum_distance",
                           loop_search_maximum_distance)) {
    line_segment_mapper_->setParamLoopSearchMaximumDistance(
        loop_search_maximum_distance);
  }

  bool do_loop_closing;
  if (private_nh_.getParam("do_loop_closing", do_loop_closing)) {
    line_segment_mapper_->setParamDoLoopClosing(do_loop_closing);
  }

  int loop_match_minimum_chain_size;
  if (private_nh_.getParam("loop_match_minimum_chain_size",
                           loop_match_minimum_chain_size)) {
    line_segment_mapper_->setParamLoopMatchMinimumChainSize(
        loop_match_minimum_chain_size);
  }

  double loop_match_maximum_variance_coarse;
  if (private_nh_.getParam("loop_match_maximum_variance_coarse",
                           loop_match_maximum_variance_coarse)) {
    line_segment_mapper_->setParamLoopMatchMaximumVarianceCoarse(
        loop_match_maximum_variance_coarse);
  }

  double loop_match_minimum_response_coarse;
  if (private_nh_.getParam("loop_match_minimum_response_coarse",
                           loop_match_minimum_response_coarse)) {
    line_segment_mapper_->setParamLoopMatchMinimumResponseCoarse(
        loop_match_minimum_response_coarse);
  }

  double loop_match_minimum_response_fine;
  if (private_nh_.getParam("loop_match_minimum_response_fine",
                           loop_match_minimum_response_fine)) {
    line_segment_mapper_->setParamLoopMatchMinimumResponseFine(
        loop_match_minimum_response_fine);
  }

  // Setting Correlation Parameters from the Parameter Server

  double correlation_search_space_dimension;
  if (private_nh_.getParam("correlation_search_space_dimension",
                           correlation_search_space_dimension)) {
    line_segment_mapper_->setParamCorrelationSearchSpaceDimension(
        correlation_search_space_dimension);
  }

  double correlation_search_space_resolution;
  if (private_nh_.getParam("correlation_search_space_resolution",
                           correlation_search_space_resolution)) {
    line_segment_mapper_->setParamCorrelationSearchSpaceResolution(
        correlation_search_space_resolution);
  }

  double correlation_search_space_smear_deviation;
  if (private_nh_.getParam("correlation_search_space_smear_deviation",
                           correlation_search_space_smear_deviation)) {
    line_segment_mapper_->setParamCorrelationSearchSpaceSmearDeviation(
        correlation_search_space_smear_deviation);
  }

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter
  // Server

  double loop_search_space_dimension;
  if (private_nh_.getParam("loop_search_space_dimension",
                           loop_search_space_dimension)) {
    line_segment_mapper_->setParamLoopSearchSpaceDimension(
        loop_search_space_dimension);
  }

  double loop_search_space_resolution;
  if (private_nh_.getParam("loop_search_space_resolution",
                           loop_search_space_resolution)) {
    line_segment_mapper_->setParamLoopSearchSpaceResolution(
        loop_search_space_resolution);
  }

  double loop_search_space_smear_deviation;
  if (private_nh_.getParam("loop_search_space_smear_deviation",
                           loop_search_space_smear_deviation)) {
    line_segment_mapper_->setParamLoopSearchSpaceSmearDeviation(
        loop_search_space_smear_deviation);
  }

  // Setting Scan Matcher Parameters from the Parameter Server

  double distance_variance_penalty;
  if (private_nh_.getParam("distance_variance_penalty",
                           distance_variance_penalty)) {
    line_segment_mapper_->setParamDistanceVariancePenalty(
        distance_variance_penalty);
  }

  double angle_variance_penalty;
  if (private_nh_.getParam("angle_variance_penalty", angle_variance_penalty)) {
    line_segment_mapper_->setParamAngleVariancePenalty(angle_variance_penalty);
  }

  double fine_search_angle_offset;
  if (private_nh_.getParam("fine_search_angle_offset",
                           fine_search_angle_offset)) {
    line_segment_mapper_->setParamFineSearchAngleOffset(
        fine_search_angle_offset);
  }

  double coarse_search_angle_offset;
  if (private_nh_.getParam("coarse_search_angle_offset",
                           coarse_search_angle_offset)) {
    line_segment_mapper_->setParamCoarseSearchAngleOffset(
        coarse_search_angle_offset);
  }

  double coarse_angle_resolution;
  if (private_nh_.getParam("coarse_angle_resolution",
                           coarse_angle_resolution)) {
    line_segment_mapper_->setParamCoarseAngleResolution(
        coarse_angle_resolution);
  }

  double minimum_angle_penalty;
  if (private_nh_.getParam("minimum_angle_penalty", minimum_angle_penalty)) {
    line_segment_mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);
  }

  double minimum_distance_penalty;
  if (private_nh_.getParam("minimum_distance_penalty",
                           minimum_distance_penalty)) {
    line_segment_mapper_->setParamMinimumDistancePenalty(
        minimum_distance_penalty);
  }

  bool use_response_expansion;
  if (private_nh_.getParam("use_response_expansion", use_response_expansion)) {
    line_segment_mapper_->setParamUseResponseExpansion(use_response_expansion);
  }

  // Set solver to be used in loop closure
  solver_ = std::make_unique<G2oSolver>();
  line_segment_mapper_->SetScanSolver(solver_.get());
}

SlamKarto::~SlamKarto() {
  if (transform_thread_) {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (scan_filter_) {
    delete scan_filter_;
  }
  if (scan_filter_sub_) {
    delete scan_filter_sub_;
  }
}

void SlamKarto::publishLoop(double transform_publish_period) {
  if (transform_publish_period == 0) {
    return;
  }

  ros::Rate r(1.0 / transform_publish_period);
  while (ros::ok()) {
    publishTransform();
    r.sleep();
  }
}

void SlamKarto::publishTransform() {
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  // ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_->sendTransform(tf::StampedTransform(map_to_odom_, ros::Time::now(),
                                           map_frame_, odom_frame_));
}

karto::LaserRangeFinder* SlamKarto::getLaser(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base_link.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try {
      tf_.transformPose(base_frame_, ident, laser_pose);
    } catch (tf::TransformException& ex) {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
               ex.what());
      return nullptr;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
             scan->header.frame_id.c_str(), laser_pose.getOrigin().x(),
             laser_pose.getOrigin().y(), yaw);
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser
    // frame if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try {
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.6f", up.z());
    } catch (tf::TransformException& ex) {
      ROS_WARN("Unable to determine orientation of laser: %s", ex.what());
      return nullptr;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse) {
      ROS_DEBUG("laser is mounted upside-down");
    }

    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(
            karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
                                      laser_pose.getOrigin().y(), yaw));
    laser->SetRangeThreshold(range_threshold_);
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool SlamKarto::getOdomPose(karto::Pose2* karto_pose, const ros::Time& t) {
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(
      tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0)),
      t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  } catch (tf::TransformException& ex) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", ex.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  *karto_pose =
      karto::Pose2(odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw);
  return true;
}

void SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0) {
    return;
  }

  static ros::Time last_map_update(0, 0);

  // Check whether we know about this laser yet
  karto::LaserRangeFinder* laser = getLaser(scan);

  if (!laser) {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
             scan->header.frame_id.c_str());
    return;
  }

  karto::Pose2 odom_pose;
  if (addScan(laser, scan, &odom_pose)) {
    ROS_DEBUG("added scan at pose: %.6f %.6f %.6f", odom_pose.GetX(),
              odom_pose.GetY(), odom_pose.GetHeading());

    // visualization_msgs::MarkerArray marray;
    // solver_->publishGraphVisualization(marray);
    // marker_publisher_.publish(marray);

    publishLineSegmentMapVisualization();

    if (!got_map_ ||
        (scan->header.stamp - last_map_update) > map_update_interval_) {
      if (updateMap()) {
        last_map_update = scan->header.stamp;
        got_map_ = true;
        ROS_DEBUG("Updated the map");
      }
    }
  }
}

bool SlamKarto::updateMap() {
  boost::mutex::scoped_lock lock(map_mutex_);

  karto::OccupancyGrid* occ_grid = karto::OccupancyGrid::CreateFromScans(
      line_segment_mapper_->GetAllProcessedScans(), resolution_);

  if (!occ_grid) {
    return false;
  }

  if (!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
      occ_grid->GetCoordinateConverter()->GetOffset();

  if (map_.map.info.width != (unsigned int)width ||
      map_.map.info.height != (unsigned int)height ||
      map_.map.info.origin.position.x != offset.GetX() ||
      map_.map.info.origin.position.y != offset.GetY()) {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value) {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;

        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;

        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;

        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  delete occ_grid;

  return true;
}

void SlamKarto::publishLineSegmentMapVisualization() {
  const karto::LineSegmentPtrHashTable line_segment_hash_table =
      line_segment_mapper_->GetLineSegmentMapManager()->GetLineSegmentMap();

  visualization_msgs::Marker line_segment_map_marker;
  line_segment_map_marker.ns = "karto";
  line_segment_map_marker.id = 0;
  line_segment_map_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_segment_map_marker.scale.x = 0.1;
  line_segment_map_marker.color.r = 1.0;
  line_segment_map_marker.color.g = 0.0;
  line_segment_map_marker.color.b = 0.0;
  line_segment_map_marker.color.a = 1.0;
  line_segment_map_marker.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);

  for (auto hash_iter = line_segment_hash_table.begin();
       hash_iter != line_segment_hash_table.end(); ++hash_iter) {
    // if (iter->second->n < 5) continue;

    const karto::LineSegmentPtr merged_line_segment = hash_iter->second;

    geometry_msgs::Point p_start;
    p_start.x = merged_line_segment->GetStartPoint().GetX();
    p_start.y = merged_line_segment->GetStartPoint().GetY();
    p_start.z = 0;
    line_segment_map_marker.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = merged_line_segment->GetEndPoint().GetX();
    p_end.y = merged_line_segment->GetEndPoint().GetY();
    p_end.z = 0;
    line_segment_map_marker.points.push_back(p_end);
  }
  line_segment_map_marker.header.frame_id = map_frame_;
  line_segment_map_marker.header.stamp = ros::Time::now();
  line_segment_map_marker_publisher_.publish(line_segment_map_marker);
}

bool SlamKarto::addScan(karto::LaserRangeFinder* laser,
                        const sensor_msgs::LaserScan::ConstPtr& scan,
                        karto::Pose2* karto_pose) {
  if (!getOdomPose(karto_pose, scan->header.stamp)) {
    return false;
  }

  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  if (lasers_inverted_[scan->header.frame_id]) {
    for (auto it = scan->ranges.rbegin(); it != scan->ranges.rend(); ++it) {
      readings.push_back(*it);
    }
  } else {
    for (auto it = scan->ranges.begin(); it != scan->ranges.end(); ++it) {
      readings.push_back(*it);
    }
  }

  // create localized range scan
  karto::LineSegmentPtrVector line_segments;
  line_segment_extractor_->extract_lines(readings, laser, &line_segments);

  if (line_segments.empty()) {
    return false;
  }

  karto::LocalizedRangeScan* range_scan =
      new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetTime(scan->header.stamp.toSec());
  range_scan->SetOdometricPose(*karto_pose);
  range_scan->SetCorrectedPose(*karto_pose);

  for (auto iter = line_segments.begin(); iter != line_segments.end(); ++iter) {
    iter->get()->SetScan(range_scan);
  }
  // Add the localized range scan to the mapper
  bool processed;
  if ((processed = line_segment_mapper_->Process(range_scan, line_segments))) {
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();
    tf::Transform corrected_pose_tf(
        tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
        tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));

    // Compute the map->odom transform
    tf::Stamped<tf::Pose> odom_to_map;
    try {
      tf_.transformPose(odom_frame_,
                        tf::Stamped<tf::Pose>(corrected_pose_tf.inverse(),
                                              scan->header.stamp, base_frame_),
                        odom_to_map);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }

    map_to_odom_mutex_.lock();
    map_to_odom_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()))
                       .inverse();
    map_to_odom_mutex_.unlock();

    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  } else {
    delete range_scan;
  }

  return processed;
}

bool SlamKarto::mapCallback(nav_msgs::GetMap::Request& req,
                            nav_msgs::GetMap::Response& res) {
  UNUSED(req);
  boost::mutex::scoped_lock lock(map_mutex_);
  if (got_map_ && map_.map.info.width && map_.map.info.height) {
    res = map_;
    return true;
  }
  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_karto");

  SlamKarto kn;

  ros::spin();

  return 0;
}
