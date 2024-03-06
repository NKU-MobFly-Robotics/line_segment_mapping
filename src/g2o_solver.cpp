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

#include "line_segment_mapping/g2o_solver.h"

#include <unordered_set>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "ros/console.h"
#include "tf/tf.h"

using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;

#ifdef SBA_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
using LinearSolver = g2o::LinearSolverCholmod<BlockSolver::PoseMatrixType>;
#else
#include "g2o/solvers/csparse/linear_solver_csparse.h"
using LinearSolver = g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType>;
#endif

namespace line_segment_mapping {

G2oSolver::G2oSolver() {
  // Initialize the SparseOptimizer
  auto linear_solver = std::make_unique<LinearSolver>();
  linear_solver->setBlockOrdering(false);
  auto block_solver = std::make_unique<BlockSolver>(std::move(linear_solver));
  auto* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer_.setAlgorithm(solver);
}

G2oSolver::~G2oSolver() {
  // freeing the graph memory
  optimizer_.clear();
}

void G2oSolver::Clear() {
  // freeing the graph memory
  LOG(INFO) << "Clearing Optimizer...";
  corrections_.clear();
}

void G2oSolver::Compute() {
  corrections_.clear();

  // Fix the first node in the graph to hold the map in place
  g2o::OptimizableGraph::Vertex* first = optimizer_.vertex(0);
  if (first == nullptr) {
    LOG(ERROR) << "No Node with ID 0 found!";
    return;
  }
  first->setFixed(true);

  // Do the graph optimization
  optimizer_.initializeOptimization();
  int iter = optimizer_.optimize(40);
  if (iter > 0) {
    LOG(INFO) << "Optimization finished after " << iter << " iterations.";
  } else {
    LOG(ERROR) << "Optimization failed, result might be invalid!";
    return;
  }

  // Write the result so it can be used by the mapper
  const g2o::SparseOptimizer::VertexContainer& nodes =
      optimizer_.activeVertices();
  for (const auto* node : nodes) {
    std::array<double, 3> estimate;
    if (node->getEstimateData(estimate.data())) {
      karto::Pose2 pose(estimate[0], estimate[1], estimate[2]);
      corrections_.emplace_back(node->id(), pose);
    } else {
      LOG(ERROR) << "Could not get estimated pose from Optimizer!";
    }
  }
}

void G2oSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* vertex) {
  const karto::Pose2& odom = vertex->GetObject()->GetCorrectedPose();
  auto* pose_vertex = new g2o::VertexSE2();
  pose_vertex->setEstimate(
      g2o::SE2(odom.GetX(), odom.GetY(), odom.GetHeading()));
  pose_vertex->setId(vertex->GetObject()->GetUniqueId());
  optimizer_.addVertex(pose_vertex);
  VLOG(4) << "Adding node " << vertex->GetObject()->GetUniqueId() << ".";
}

void G2oSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* edge) {
  // Create a new edge
  auto* odometry = new g2o::EdgeSE2();

  // Set source and target
  int source_id = edge->GetSource()->GetObject()->GetUniqueId();
  int target_id = edge->GetTarget()->GetObject()->GetUniqueId();
  odometry->vertices()[0] = optimizer_.vertex(source_id);
  odometry->vertices()[1] = optimizer_.vertex(target_id);
  if (odometry->vertices()[0] == nullptr) {
    LOG(ERROR) << "Source vertex with id " << source_id << " does not exist!";
    free(odometry);
    return;
  }
  if (odometry->vertices()[1] == nullptr) {
    LOG(ERROR) << "Target vertex with id " << target_id << " does not exist!";
    free(odometry);
    return;
  }

  // Set the measurement (odometry distance between vertices)
  auto* link_info = dynamic_cast<karto::LinkInfo*>(edge->GetLabel());
  karto::Pose2 diff = link_info->GetPoseDifference();
  g2o::SE2 measurement(diff.GetX(), diff.GetY(), diff.GetHeading());
  odometry->setMeasurement(measurement);

  // Set the covariance of the measurement
  karto::Matrix3 precision_matrix = link_info->GetCovariance().Inverse();
  Eigen::Matrix<double, 3, 3> info;
  info(0, 0) = precision_matrix(0, 0);
  info(0, 1) = info(1, 0) = precision_matrix(0, 1);
  info(0, 2) = info(2, 0) = precision_matrix(0, 2);
  info(1, 1) = precision_matrix(1, 1);
  info(1, 2) = info(2, 1) = precision_matrix(1, 2);
  info(2, 2) = precision_matrix(2, 2);
  odometry->setInformation(info);

  // Add the constraint to the optimizer
  VLOG(4) << "Adding edge from node " << source_id << " to node " << target_id
          << ".";
  optimizer_.addEdge(odometry);
}

const karto::ScanSolver::IdPoseVector& G2oSolver::GetCorrections() const {
  return corrections_;
}

/**
 * Fill out a visualization_msg to display the odometry graph
 * This function also identifies the loop closure edges and adds them to the
 * loop_closure_edges_ data for further processing
 */
void G2oSolver::PublishGraphVisualization(
    visualization_msgs::MarkerArray* marker_array) {
  // Vertices are round, red spheres
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  // Odometry edges are opaque blue line strips
  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  // Loop edges are purple, opacity depends on backend state
  visualization_msgs::Marker loop_edge;
  loop_edge.header.frame_id = "map";
  loop_edge.header.stamp = ros::Time::now();
  loop_edge.action = visualization_msgs::Marker::ADD;
  loop_edge.ns = "karto";
  loop_edge.id = 0;
  loop_edge.type = visualization_msgs::Marker::LINE_STRIP;
  loop_edge.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  loop_edge.scale.x = 0.1;
  loop_edge.scale.y = 0.1;
  loop_edge.scale.z = 0.1;
  loop_edge.color.a = 1.0;
  loop_edge.color.r = 1.0;
  loop_edge.color.g = 0.0;
  loop_edge.color.b = 1.0;

  int id = static_cast<int>(marker_array->markers.size());
  m.action = visualization_msgs::Marker::ADD;

  std::unordered_set<int> vertex_ids;
  // HyperGraph Edges
  const auto& edges = optimizer_.edges();
  for (const auto* edge_it : edges) {
    // Is it a unary edge? Need to skip or else die a bad death
    if (edge_it->vertices().size() < 2) {
      continue;
    }

    const auto* v1 =
        dynamic_cast<const g2o::VertexSE2*>(edge_it->vertices()[0]);
    const auto* v2 =
        dynamic_cast<const g2o::VertexSE2*>(edge_it->vertices()[1]);

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    p1.x = v1->estimate()[0];
    p1.y = v1->estimate()[1];
    p2.x = v2->estimate()[0];
    p2.y = v2->estimate()[1];

    if ((v2->id() - v1->id()) < 70) {
      // not a loop closure
      edge.points.clear();
      edge.points.push_back(p1);
      edge.points.push_back(p2);
      edge.id = id;
      marker_array->markers.push_back(visualization_msgs::Marker(edge));
      id++;
    } else {
      // it's a loop closure
      loop_edge.points.clear();
      loop_edge.points.push_back(p1);
      loop_edge.points.push_back(p2);
      loop_edge.id = id++;

      loop_edge.color.a = 1.0;

      marker_array->markers.push_back(visualization_msgs::Marker(loop_edge));
    }

    // Check the vertices exist, if not add
    if (vertex_ids.find(v2->id()) == vertex_ids.end()) {
      // Add the vertex to the marker array
      m.id = id;
      m.pose.position.x = p2.x;
      m.pose.position.y = p2.y;
      vertex_ids.insert(v2->id());
      marker_array->markers.push_back(visualization_msgs::Marker(m));
      id++;
    }
  }
}

}  // namespace line_segment_mapping
