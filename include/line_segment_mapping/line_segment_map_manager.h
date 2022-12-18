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

#pragma once

#include <vector>

#include "open_karto/Karto.h"

#include "line_segment_mapping/line_segment_feature.h"

namespace karto {
/**
 * An incremental and consistent line segment mapping module, please refer the
 * following paper for more detail J. Wen, X. Zhang, H. Gao, J. Yuan, Y. Fang,
 * "CAE_RLSM: Consistent and efficient redundant line segment merging for online
 * feature map building", IEEE Transactions on Instrumentation and Measurement,
 * 2020, 69(7): 4222-4237.
 */
class LineSegmentMapManager {
 public:
  LineSegmentMapManager() = default;
  virtual ~LineSegmentMapManager() = default;

  /**
   * Incremental line segment mapping based on one-to-one redundant line segment
   * merging strategy
   */
  bool NaiveMerge(const LineSegmentPtrVector& rLineSegments);

  /**
   * Incremental line segment mapping based on one-to-many redundant line
   * segment merging strategy
   */
  bool IncrementalMerge(const LineSegmentPtrVector& rLineSegments);

  /**
   * Global map adjustment based on the optimized robot poses
   */
  void GlobalMapAdjustment();

  const LineSegmentPtrHashTable& GetLineSegmentMap() const {
    return m_LineSegmentMap;
  }

  const LineSegmentPtrVectorHashTable& GetLineSegmentClusters() const {
    return m_LineSegmentClusters;
  }

  const std::vector<int>& GetClustersIndexArray() const {
    return m_ClustersIndexArray;
  }

 private:
  /**
   * Transform the current line segments to the world coordinates, and calculate
   * the overlap with the global line segment map
   */
  bool CalculateOverlap(const LineSegment& prevLineSegment,
                        const LineSegment& currLineSegment);

  void MapAdjustment(int index);

  /**
   * Merge linesegments
   */
  LineSegment* MergeLineSegments(const std::vector<LineSegment>& rLineSegments);

  bool updateCheck();

 private:
  LineSegmentPtrHashTable
      m_LineSegmentMap;  // Global line segment map, wherein each line segment
                         // is represented in the world coordinates
  LineSegmentPtrVectorHashTable
      m_LineSegmentClusters;  // A hashtable used to record the line segment
                              // cluster and the corresponding original line
                              // segments
  std::vector<int> m_ClustersIndexArray;  // An array used to store the indices
                                          // of the line segment cluster
  int m_LineSegmentClustersIndex = 0;     // Index of the line segment cluster
};

}  // namespace karto
