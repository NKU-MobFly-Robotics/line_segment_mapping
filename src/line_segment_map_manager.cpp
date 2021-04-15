#include "line_segment_mapping/line_segment_map_manager.h"

namespace karto
{
bool LineSegmentMapManager::NaiveMerge(const LineSegmentPtrVector& rLineSegments)
{
  bool doUpdate = false;

  // The global line segment map is empty, insert the current line segments directly
  if (m_LineSegmentMap.empty())
  {
    for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
    {
      LineSegment globalLineSegment = (*iter)->GetGlobalLineSegments();
      m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, globalLineSegment));

      LineSegmentPtrVector lineSegmentCluster;
      lineSegmentCluster.push_back(*iter);
      m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

      m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
      m_LineSegmentClustersIndex++;
    }
    doUpdate = true;
  }
  else
  {
    std::vector<bool> remain;
    remain.resize(rLineSegments.size(), true);
    int remainId = 0;
    for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
    {
      for (auto inner_iter = m_LineSegmentMap.begin(); inner_iter != m_LineSegmentMap.end(); ++inner_iter)
      {
        // Obtain the current line segments in world coordinates
        LineSegment globalLineSegment = (*iter)->GetGlobalLineSegments();

        if (CalculateOverlap(inner_iter->second, globalLineSegment))
        {
          LineSegmentVector candidates;
          candidates.push_back(globalLineSegment);

          LineSegmentPtrVector* originalLineSegments = &m_LineSegmentClusters[inner_iter->first];
          for (auto iterator = originalLineSegments->begin(); iterator != originalLineSegments->end(); ++iterator)
          {
            LineSegment tmp = (*iterator)->GetGlobalLineSegments();
            candidates.push_back(tmp);
          }

          // Merge the redundant line segments and update the global line segment map
          LineSegment updatedLine = MergeLineSegments(candidates);
          inner_iter->second = updatedLine;

          originalLineSegments->push_back(*iter);

          remain[remainId] = false;

          doUpdate = true;
          break;  // inner loop end
        }
      }

      remainId++;
    }

    // Insert the line segments extarcted from the current scan into the global line segment map
    // These line segments are new and there are no line segments in the global map that match them
    remainId = 0;
    for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
    {
      if (remain[remainId])
      {
        LineSegment globalLineSegment = (*iter)->GetGlobalLineSegments();
        m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, globalLineSegment));

        LineSegmentPtrVector lineSegmentCluster;
        lineSegmentCluster.push_back(*iter);
        m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

        m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
        m_LineSegmentClustersIndex++;
      }

      remainId++;
    }
  }

  // assert(updateCheck());

  return doUpdate;
}

bool LineSegmentMapManager::IncrementalMerge(const LineSegmentPtrVector& rLineSegments)
{
  bool doUpdate = false;
  bool doMerge = false;
  int index = -1;
  LineSegmentVector candidates;
  std::vector<int> candidateIndex;

  // The global line segment map is empty, insert the current line segments directly
  if (m_LineSegmentMap.empty())
  {
    for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
    {
      LineSegment globalLineSegment = (*iter)->GetGlobalLineSegments();

      m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, globalLineSegment));

      LineSegmentPtrVector lineSegmentCluster;
      lineSegmentCluster.push_back(*iter);
      m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

      m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
      m_LineSegmentClustersIndex++;
    }

    doUpdate = true;
  }
  else
  {
    std::vector<bool> remain;
    remain.resize(rLineSegments.size(), true);
    int remainId = 0;
    for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
    {
      for (auto inner_iter = m_LineSegmentMap.begin(); inner_iter != m_LineSegmentMap.end();)
      {
        LineSegment globalLineSegment = (*iter)->GetGlobalLineSegments();
        if (CalculateOverlap(inner_iter->second, globalLineSegment))
        {
          doUpdate = true;
          doMerge = true;
          remain[remainId] = false;

          LineSegmentPtrVector originalLineSegments = m_LineSegmentClusters[inner_iter->first];
          for (auto iterator = originalLineSegments.begin(); iterator != originalLineSegments.end(); ++iterator)
          {
            LineSegment tmp = (*iterator)->GetGlobalLineSegments();
            candidates.push_back(tmp);
          }

          // It is possible to have more than one line segment in the global map matching this line segment
          // So we record the minimum index of the matched line segments in the global map
          if (index == -1)
          {
            candidates.push_back(globalLineSegment);
            index = inner_iter->first;
            ++inner_iter;
          }
          else
          {
            candidateIndex.push_back(inner_iter->first);
            inner_iter = m_LineSegmentMap.erase(inner_iter);
          }
        }
        else
        {
          ++inner_iter;
        }
      }  // inner loop end

      if (doMerge)
      {
        LineSegment updatedLine = MergeLineSegments(candidates);
        m_LineSegmentMap[index] = updatedLine;

        LineSegmentPtrVector* objectCluster = &m_LineSegmentClusters[index];

        // Move the corresponding original line segments of the merged line segments to the new cluster
        for (auto index_iter = candidateIndex.begin(); index_iter != candidateIndex.end(); ++index_iter)
        {
          LineSegmentPtrVector* candidateCluster = &m_LineSegmentClusters[*index_iter];

          objectCluster->insert(objectCluster->end(), candidateCluster->begin(), candidateCluster->end());
          candidateCluster->clear();
        }
        objectCluster->push_back(*iter);

        bool flag = false;
        for (auto index_iter = m_ClustersIndexArray.begin(); index_iter != m_ClustersIndexArray.end();)
        {
          for (auto candidate_iter = candidateIndex.begin(); candidate_iter != candidateIndex.end();)
          {
            if (*index_iter == *candidate_iter)
            {
              flag = true;
              index_iter = m_ClustersIndexArray.erase(index_iter);
              candidate_iter = candidateIndex.erase(candidate_iter);
              break;
            }
            else
              ++candidate_iter;
          }

          if (flag)
            flag = false;
          else
            ++index_iter;
        }
      }

      // refresh
      doMerge = false;
      index = -1;
      candidates.clear();
      assert(candidateIndex.empty());

      remainId++;
    }

    remainId = 0;
    for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
    {
      if (remain[remainId])
      {
        LineSegment globalLineSegment = (*iter)->GetGlobalLineSegments();
        m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, globalLineSegment));

        LineSegmentPtrVector lineSegmentCluster;
        lineSegmentCluster.push_back(*iter);
        m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

        m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
        m_LineSegmentClustersIndex++;
      }

      remainId++;
    }
  }

  // assert(updateCheck());

  return doUpdate;
}

void LineSegmentMapManager::GlobalMapAdjustment()
{
  std::cout << "Global map adjustment called." << std::endl;

  // #pragma omp parallel for
  for (int i = 0; i < static_cast<int>(m_ClustersIndexArray.size()); i++)
  {
    MapAdjustment(m_ClustersIndexArray[i]);
  }

  std::cout << "Global map adjustment done." << std::endl;
  // assert(updateCheck());
}

bool LineSegmentMapManager::CalculateOverlap(const LineSegment& prevLineSegment, const LineSegment& currLineSegment)
{
  double headingDeviation = math::NormalizeAngle(currLineSegment.GetHeading() - prevLineSegment.GetHeading());
  if (fabs(headingDeviation) > math::DegreesToRadians(4.0))  // The two line segments are not parallel
  {
    return false;
  }

  double A, B, C;

  // Considering whether the slope of a straight line exists or not, it is discussed in two cases
  // and finally transformed into a general formula of a straight line
  if (fabs(fabs(prevLineSegment.GetHeading()) - KT_PI_2) < KT_TOLERANCE)
  {
    A = 1;
    B = 0;
    C = -(prevLineSegment.GetStartPoint().GetX() + prevLineSegment.GetEndPoint().GetX()) / 2.0;
  }
  else
  {
    A = tan(prevLineSegment.GetHeading());
    B = -1;
    C = prevLineSegment.GetStartPoint().GetY() - A * prevLineSegment.GetStartPoint().GetX();
  }

  double dist1 =
      point_to_line_distance(A, B, C, currLineSegment.GetStartPoint().GetX(), currLineSegment.GetStartPoint().GetY());
  double dist2 =
      point_to_line_distance(A, B, C, currLineSegment.GetEndPoint().GetX(), currLineSegment.GetEndPoint().GetY());
  double dist = math::Maximum(dist1, dist2);

  if (dist > 0.10)
  {
    return false;
  }

  // 将线段端点投影到直线上
  // 已知直线M的一般式方程为Ax+By+C=0, 则过点(a, b)且与L垂直的直线N的一般式方程为Bx-Ay+Ab-Ba=0
  // 直线M与直线N的交点坐标：x=(B*B*a-A*B*b-A*C)/(A*A+B*B), y=(A*A*b-A*B*a-B*C)/(A*A+B*B)
  double x, y;
  Vector2<double> prevStartPoint, prevEndPoint, currStartPoint, currEndPoint;

  prevStartPoint.SetX(prevLineSegment.GetStartPoint().GetX());
  prevStartPoint.SetY(prevLineSegment.GetStartPoint().GetY());
  prevEndPoint.SetX(prevLineSegment.GetEndPoint().GetX());
  prevEndPoint.SetY(prevLineSegment.GetEndPoint().GetY());

  x = (B * B * currLineSegment.GetStartPoint().GetX() - A * B * currLineSegment.GetStartPoint().GetY() - A * C) /
      (A * A + B * B);
  y = (A * A * currLineSegment.GetStartPoint().GetY() - A * B * currLineSegment.GetStartPoint().GetX() - B * C) /
      (A * A + B * B);
  currStartPoint.SetX(x);
  currStartPoint.SetY(y);

  x = (B * B * currLineSegment.GetEndPoint().GetX() - A * B * currLineSegment.GetEndPoint().GetY() - A * C) /
      (A * A + B * B);
  y = (A * A * currLineSegment.GetEndPoint().GetY() - A * B * currLineSegment.GetEndPoint().GetX() - B * C) /
      (A * A + B * B);
  currEndPoint.SetX(x);
  currEndPoint.SetY(y);

  // 当前线段起点指向参考线段起点的方向向量
  Vector2<double> vector1 = currStartPoint - prevStartPoint;
  // 当前线段起点指向参考线段终点的方向向量
  Vector2<double> vector2 = currStartPoint - prevEndPoint;
  // 当前线段终点指向参考线段起点的方向向量
  Vector2<double> vector3 = currEndPoint - prevStartPoint;
  // 当前线段终点指向参考线段终点的方向向量
  Vector2<double> vector4 = currEndPoint - prevEndPoint;

  if (vector1.GetX() * vector2.GetX() + vector1.GetY() * vector2.GetY() > 0)  // 说明当前线段的起点落在参考线段的两侧
  {
    if (vector1.Length() < vector2.Length())  // 说明当前线段的起点落在参考线段起点的外侧
    {
      if (vector3.GetX() * vector4.GetX() + vector3.GetY() * vector4.GetY() >
          0)  // 说明当前线段的终点落在参考线段的两侧
      {
        if (vector3.Length() < vector4.Length())  // 说明当前线段的终点落在参考线段起点的外侧
        {
          if (vector3.Length() <= 0.10)
            return true;
          else
            return false;
        }
        else  // 说明当前线段的终点落在参考线段终点的外侧
        {
          return true;
        }
      }
      else  // 说明当前线段的终点落在参考线段的内部(包含端点)
      {
        return true;
      }
    }
    else  // 说明当前线段的起点落在参考线段终点的外侧
    {
      if (vector2.Length() <= 0.10)
        return true;
      else
        return false;
    }
  }
  else  // 说明当前线段的起点落在参考线段的内部(包含端点)
  {
    return true;
  }
}

void LineSegmentMapManager::MapAdjustment(int index)
{
  LineSegmentPtrVector lineSegmentCluster = m_LineSegmentClusters[index];
  LineSegmentVector globalLineSegments;

  for (auto iter = lineSegmentCluster.begin(); iter != lineSegmentCluster.end(); ++iter)
  {
    LineSegment globalLineSegment = (*iter)->GetGlobalLineSegments();
    globalLineSegments.push_back(globalLineSegment);
  }

  m_LineSegmentMap[index] = MergeLineSegments(globalLineSegments);
}

/**
 * Merge linesegments
 */
LineSegment LineSegmentMapManager::MergeLineSegments(const LineSegmentVector& rLineSegments)
{
  assert(!rLineSegments.empty());
  Vector2<double> directionVector;
  Vector2<double> centralPoint;
  int updateRate = 0;
  for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
  {
    directionVector += iter->GetDirectionVector();
    centralPoint += iter->GetBarycenter();
    updateRate += iter->GetUpdateTimes();
  }
  // 计算线段的单位方向向量
  double vectorLength = directionVector.Length();  // 归一化因子
  assert(vectorLength > KT_TOLERANCE);

  double heading = atan2(directionVector.GetY(), directionVector.GetX());
  assert(math::InRange(heading, -KT_PI, KT_PI));

  Vector2<double> barycenter = centralPoint / static_cast<int>(rLineSegments.size());

  double A, B, C;
  // 考虑直线斜率存在与否，分两种情况讨论，最终化为直线的一般式
  if (fabs(fabs(heading) - KT_PI_2) < KT_TOLERANCE)
  {
    A = 1;
    B = 0;
    C = -barycenter.GetX();
  }
  else
  {
    A = tan(heading);
    B = -1;
    C = barycenter.GetY() - A * barycenter.GetX();
  }

  PointVectorDouble startPointVector, endPointVector;
  Vector2<double> startPoint, endPoint;
  for (auto iter = rLineSegments.begin(); iter != rLineSegments.end(); ++iter)
  {
    startPoint.SetX((B * B * iter->GetStartPoint().GetX() - A * B * iter->GetStartPoint().GetY() - A * C) /
                    (A * A + B * B));
    startPoint.SetY((A * A * iter->GetStartPoint().GetY() - A * B * iter->GetStartPoint().GetX() - B * C) /
                    (A * A + B * B));
    startPointVector.push_back(startPoint);
    endPoint.SetX((B * B * iter->GetEndPoint().GetX() - A * B * iter->GetEndPoint().GetY() - A * C) / (A * A + B * B));
    endPoint.SetY((A * A * iter->GetEndPoint().GetY() - A * B * iter->GetEndPoint().GetX() - B * C) / (A * A + B * B));
    endPointVector.push_back(endPoint);
  }

  double maximumDistance = 0;
  Vector2<double> updatedStartPoint, updatedEndPoint;
  for (auto iter = startPointVector.begin(); iter != startPointVector.end(); ++iter)
  {
    for (auto inner_iter = endPointVector.begin(); inner_iter != endPointVector.end(); ++inner_iter)
    {
      double distance = (*iter - *inner_iter).Length();
      if (distance > maximumDistance)
      {
        maximumDistance = distance;
        updatedStartPoint = *iter;
        updatedEndPoint = *inner_iter;
      }
    }
  }

  LineSegment lineSegment(updatedStartPoint, updatedEndPoint);
  lineSegment.SetUpdateTimes(updateRate);
  return lineSegment;
}

bool LineSegmentMapManager::updateCheck()
{
  for (auto iter = m_LineSegmentMap.begin(); iter != m_LineSegmentMap.end(); ++iter)
  {
    int updateNum = static_cast<int>(m_LineSegmentClusters[iter->first].size());
    if (updateNum != iter->second.GetUpdateTimes())
    {
      return false;
    }
  }
  return true;
}

}  // namespace karto