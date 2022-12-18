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

#include <memory>
#include <unordered_map>
#include <vector>

#include "glog/logging.h"
#include "open_karto/Karto.h"

namespace karto {
inline double point_to_line_distance(double A, double B, double C, double x,
                                     double y) {
  return fabs(A * x + B * y + C) / sqrt(A * A + B * B);
}

class LineSegment {
 public:
  LineSegment() {}

  LineSegment(const Vector2<double>& startPoint,
              const Vector2<double>& endPoint) {
    m_StartPoint = startPoint;
    m_EndPoint = endPoint;
    m_Barycenter = (m_StartPoint + m_EndPoint) / 2;
    m_DirectionVector = m_EndPoint - m_StartPoint;
    CHECK(!std::isnan(m_DirectionVector.GetX()) &&
          !std::isnan(m_DirectionVector.GetY()));  // 排除NaN值的情况

    m_Length = m_DirectionVector.Length();
    CHECK_GE(m_Length, KT_TOLERANCE);

    m_Heading = atan2(m_DirectionVector.GetY(), m_DirectionVector.GetX());
    CHECK(math::InRange(m_Heading, -KT_PI, KT_PI));
    m_UpdateTimes = 1;

    double A = m_EndPoint.GetY() - m_StartPoint.GetY();
    double B = m_StartPoint.GetX() - m_EndPoint.GetX();
    double C = m_EndPoint.GetX() * m_StartPoint.GetY() -
               m_StartPoint.GetX() * m_EndPoint.GetY();
    m_Role = fabs(C) / sqrt(A * A + B * B);

    double x = -A * C / (A * A + B * B);
    double y = -B * C / (A * A + B * B);
    m_Phi = atan2(y, x);

    m_pScan = nullptr;
    m_ClusterIndex = -1;
  }

  LineSegment(const LineSegment& rOther) {
    m_StartPoint = rOther.m_StartPoint;
    m_EndPoint = rOther.m_EndPoint;
    m_Barycenter = rOther.m_Barycenter;
    m_DirectionVector = rOther.m_DirectionVector;
    m_Length = rOther.m_Length;
    m_Heading = rOther.m_Heading;
    m_UpdateTimes = rOther.m_UpdateTimes;
    m_Role = rOther.m_Role;
    m_Phi = rOther.m_Phi;

    m_pScan = nullptr;
    m_ClusterIndex = -1;
  }

  virtual ~LineSegment() { m_pScan = nullptr; }

 public:
  const Vector2<double>& GetStartPoint() const { return m_StartPoint; }

  const Vector2<double>& GetEndPoint() const { return m_EndPoint; }

  const Vector2<double>& GetBarycenter() const { return m_Barycenter; }

  const Vector2<double>& GetDirectionVector() const {
    return m_DirectionVector;
  }

  double GetHeading() const { return m_Heading; }

  double GetLength() const { return m_Length; }

  int GetUpdateTimes() const { return m_UpdateTimes; }

  void SetUpdateTimes(int updateTimes) { m_UpdateTimes = updateTimes; }

  double GetRole() const { return m_Role; }

  double GetPhi() const { return m_Phi; }

  void SetScan(LocalizedRangeScan* pScan) { m_pScan = pScan; }

  const LocalizedRangeScan* GetScan() const { return m_pScan; }

  void SetLineSegmentClusterIndex(int lineSegmentClusterIndex) {
    m_ClusterIndex = lineSegmentClusterIndex;
  }

  const int& GetLineSegmentClusterIndex() const { return m_ClusterIndex; }

  LineSegment GetGlobalLineSegments() const {
    Pose2 scanPose = m_pScan->GetSensorPose();
    double cosine = cos(scanPose.GetHeading());
    double sine = sin(scanPose.GetHeading());

    Vector2<double> startPoint, endPoint;

    startPoint.SetX(m_StartPoint.GetX() * cosine - m_StartPoint.GetY() * sine +
                    scanPose.GetX());
    startPoint.SetY(m_StartPoint.GetX() * sine + m_StartPoint.GetY() * cosine +
                    scanPose.GetY());
    endPoint.SetX(m_EndPoint.GetX() * cosine - m_EndPoint.GetY() * sine +
                  scanPose.GetX());
    endPoint.SetY(m_EndPoint.GetX() * sine + m_EndPoint.GetY() * cosine +
                  scanPose.GetY());

    LineSegment lineSegment(startPoint, endPoint);
    return lineSegment;
  }

 private:
  Vector2<double> m_StartPoint;
  Vector2<double> m_EndPoint;

  double m_Heading;  // 线段方向
  double m_Length;
  Vector2<double> m_Barycenter;       // 线段的中心点
  Vector2<double> m_DirectionVector;  // 单位方向向量

  const LocalizedRangeScan* m_pScan;  // 该线段关联的激光扫描
  int m_ClusterIndex;

  int m_UpdateTimes;  // 线段被更新的次数，初始为1
  double m_Role;
  double m_Phi;
};

typedef std::vector<LineSegment> LineSegmentVector;
typedef std::shared_ptr<LineSegment> LineSegmentPtr;
typedef std::vector<LineSegmentPtr> LineSegmentPtrVector;
typedef std::unordered_map<int, LineSegmentPtr> LineSegmentPtrHashTable;
typedef std::unordered_map<int, LineSegmentPtrVector>
    LineSegmentPtrVectorHashTable;

}  // namespace karto
