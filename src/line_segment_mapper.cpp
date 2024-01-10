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

#include "line_segment_mapping/line_segment_mapper.h"

#include <chrono>  // NOLINT

namespace line_segment_mapping {
/**
 * Default constructor
 */
LineSegmentMapper::LineSegmentMapper() : Mapper("Mapper") {}

/**
 * Default constructor
 */
LineSegmentMapper::LineSegmentMapper(const std::string& rName)
    : Mapper(rName) {}

bool LineSegmentMapper::Process(karto::LocalizedRangeScan* pScan,
                                const LineSegmentPtrVector& rLineSegments) {
  if (pScan != nullptr) {
    karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == nullptr || pScan == nullptr ||
        pLaserRangeFinder->Validate(pScan) == false) {
      return false;
    }

    if (m_Initialized == false) {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());

      m_pLineSegmentMapManager = std::make_unique<LineSegmentMapManager>();

      m_Initialized = true;
    }

    // get last scan
    // 如果这一帧是第一帧，则pLastScan返回nullptr
    karto::LocalizedRangeScan* pLastScan =
        GetMapperSensorManager()->GetLastScan(pScan->GetSensorName());

    // update scans corrected pose based on last correction
    if (pLastScan != nullptr) {
      karto::Transform lastTransform(pLastScan->GetOdometricPose(),
                                     pLastScan->GetCorrectedPose());
      // 根据码盘数据定位
      pScan->SetCorrectedPose(
          lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // test if scan is outside minimum boundary or if heading is larger then
    // minimum heading
    if (!HasMovedEnough(pScan, pLastScan)) {
      return false;
    }

    karto::Matrix3 covariance;
    covariance.SetToIdentity();

    // correct scan (if not first scan)
    if (getParamUseScanMatching() && pLastScan != nullptr) {
      karto::Pose2 bestPose;
      // 核心一：扫描匹配
      GetSequentialScanMatcher()->MatchScan(
          pScan,
          GetMapperSensorManager()->GetRunningScans(pScan->GetSensorName()),
          bestPose, covariance);
      pScan->SetSensorPose(bestPose);  // 位姿更新
    }

    // add scan to buffer and assign id
    GetMapperSensorManager()->AddScan(pScan);

    if (getParamUseScanMatching()) {
      // add to graph
      GetGraph()->AddVertex(pScan);
      GetGraph()->AddEdges(pScan, covariance);

      GetMapperSensorManager()->AddRunningScan(pScan);

      if (getParamDoLoopClosing()) {
        std::vector<karto::Name> deviceNames =
            GetMapperSensorManager()->GetSensorNames();
        const_forEach(std::vector<karto::Name>, &deviceNames) {
          // 核心二：回环检测
          bool update = GetGraph()->TryCloseLoop(pScan, *iter);
          if (update) {
            const auto start_timestamp = std::chrono::system_clock::now();
            m_pLineSegmentMapManager->GlobalMapAdjustment();
            m_pLineSegmentMapManager->IncrementalMerge(rLineSegments);
            const auto end_timestamp = std::chrono::system_clock::now();

            const auto time_diff = end_timestamp - start_timestamp;
            std::cout << "Global map adjustment costs "
                      << time_diff.count() * 1e3 << "ms\n";
          } else {
            // const auto start_timestamp = std::chrono::system_clock::now();
            if (m_pLineSegmentMapManager->IncrementalMerge(rLineSegments)) {
              // const auto end_timestamp = std::chrono::system_clock::now();
              // const auto time_diff = end_timestamp - start_timestamp;
              // std::cout << "One-to-many incremental line segment merging
              // costs"
              //           << time_diff.count() * 1e3 << "ms\n";
            }
          }
        }
      }
    } else {
      // const auto start_timestamp = std::chrono::system_clock::now();
      if (m_pLineSegmentMapManager->IncrementalMerge(rLineSegments)) {
        // const auto end_timestamp = std::chrono::system_clock::now();
        // const auto time_diff = end_timestamp - start_timestamp;
        // std::cout << "One-to-many incremental line segment merging costs"
        //           << time_diff.count() * 1e3 << "ms\n";
      }
    }

    GetMapperSensorManager()->SetLastScan(pScan);

    return true;
  }

  return false;
}

/**
 * Is the scan sufficiently far from the last scan?
 * @param pScan
 * @param pLastScan
 * @return true if the scans are sufficiently far
 */
bool LineSegmentMapper::HasMovedEnough(karto::LocalizedRangeScan* pScan,
                                       karto::LocalizedRangeScan* pLastScan) {
  // test if first scan
  if (pLastScan == nullptr) {
    return true;
  }

  // test if enough time has passed
  double timeInterval = pScan->GetTime() - pLastScan->GetTime();
  if (timeInterval >= getParamMinimumTimeInterval()) {
    return true;
  }

  karto::Pose2 lastScannerPose =
      pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
  karto::Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

  // test if we have turned enough
  double deltaHeading = karto::math::NormalizeAngle(
      scannerPose.GetHeading() - lastScannerPose.GetHeading());
  if (std::abs(deltaHeading) >= getParamMinimumTravelHeading()) {
    return true;
  }

  // test if we have moved enough
  double squaredTravelDistance =
      lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
  if (squaredTravelDistance >=
      karto::math::Square(getParamMinimumTravelDistance()) - kDoubleEpsilon) {
    return true;
  }

  return false;
}

}  // namespace line_segment_mapping
