#include "line_segment_mapping/line_segment_map.h"

namespace karto
{
void LocalizedRangeScanWithLines::SetGlobalLineSegments()
{
  Pose2 scanPose = GetSensorPose();
  double cosine = cos(scanPose.GetHeading());
  double sine = sin(scanPose.GetHeading());
  m_GlobalLineSegments.clear();
  for (auto iter = m_LocalLineSegments.begin(); iter != m_LocalLineSegments.end(); ++iter)
  {
    Vector2<double> startPoint, endPoint;

    startPoint.SetX(iter->GetStartPoint().GetX() * cosine - iter->GetStartPoint().GetY() * sine + scanPose.GetX());
    startPoint.SetY(iter->GetStartPoint().GetX() * sine + iter->GetStartPoint().GetY() * cosine + scanPose.GetY());
    endPoint.SetX(iter->GetEndPoint().GetX() * cosine - iter->GetEndPoint().GetY() * sine + scanPose.GetX());
    endPoint.SetY(iter->GetEndPoint().GetX() * sine + iter->GetEndPoint().GetY() * cosine + scanPose.GetY());

    LineSegment lineSegment(startPoint, endPoint, iter->GetIndex());
    m_GlobalLineSegments.push_back(lineSegment);
  }
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

bool LineSegmentMap::NaiveMerge(LocalizedRangeScanWithLines* pScan, const LocalizedRangeScanWithLinesVector& rScans)
{
  bool doUpdate = false;

  // 获取在世界坐标系下表达的线段
  LineSegmentVector globalLineSegments = pScan->GetGlobalLineSegments();

  if (m_LineSegmentMap.empty())
  {
    // 初始状态为空，无需进行merge操作
    for (auto global_iter = globalLineSegments.begin(); global_iter != globalLineSegments.end(); ++global_iter)
    {
      m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, *global_iter));

      std::vector<std::pair<int, int> > lineSegmentCluster;
      lineSegmentCluster.push_back(std::make_pair(m_ScanSize, global_iter->GetIndex()));
      m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

      m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
      m_LineSegmentClustersIndex++;
    }
    doUpdate = true;
  }
  else
  {
    for (auto global_iter = globalLineSegments.begin(); global_iter != globalLineSegments.end();)
    {
      bool flag = false;
      for (auto inner_iter = m_LineSegmentMap.begin(); inner_iter != m_LineSegmentMap.end(); ++inner_iter)
      {
        if (CalculateOverlap(inner_iter->second, *global_iter))
        {
          LineSegmentVector candidates;
          // candidates.push_back(inner_iter->second);
          candidates.push_back(*global_iter);
          assert(m_LineSegmentClusters.find(inner_iter->first) != m_LineSegmentClusters.end());
          std::vector<std::pair<int, int> > originalLineSegments =
              (m_LineSegmentClusters.find(inner_iter->first))->second;
          for (auto iterator = originalLineSegments.begin(); iterator != originalLineSegments.end(); ++iterator)
          {
            LocalizedRangeScanWithLines* scan = rScans[iterator->first];
            assert(scan != NULL);
            candidates.push_back(scan->GetGlobalLineSegments().at(iterator->second));
          }

          // 有overlap，需要做merge和update
          LineSegment updatedLine;
          MergeLineSegments(candidates, updatedLine);
          inner_iter->second = updatedLine;

          auto objectCluster = m_LineSegmentClusters.find(inner_iter->first);
          assert(objectCluster != m_LineSegmentClusters.end());
          (objectCluster->second).push_back(std::make_pair(m_ScanSize, global_iter->GetIndex()));

          global_iter = globalLineSegments.erase(global_iter);
          doUpdate = true;
          flag = true;
          break;  // 结束内层循环
        }
      }
      if (!flag)
      {
        ++global_iter;
      }
    }

    for (auto global_iter = globalLineSegments.begin(); global_iter != globalLineSegments.end(); ++global_iter)
    {
      m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, *global_iter));

      std::vector<std::pair<int, int> > lineSegmentCluster;
      lineSegmentCluster.push_back(std::make_pair(m_ScanSize, global_iter->GetIndex()));
      m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

      m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
      m_LineSegmentClustersIndex++;
    }
  }

  // 读取新的一帧激光数据
  m_ScanSize++;
  assert(updateCheck());

  return doUpdate;
}

bool LineSegmentMap::IncrementalMerge(LocalizedRangeScanWithLines* pScan,
                                      const LocalizedRangeScanWithLinesVector& rScans)
{
  bool doUpdate = false;
  bool doMerge = false;
  int index = -1;
  LineSegmentVector candidates;     // 待merge的线段
  std::vector<int> candidateIndex;  // 待merge的线段的索引下标

  // 获取在世界坐标系下表达的线段
  LineSegmentVector globalLineSegments = pScan->GetGlobalLineSegments();

  if (m_FirstScan)
  {
    // 初始状态为空，无需进行merge操作
    for (auto global_iter = globalLineSegments.begin(); global_iter != globalLineSegments.end(); ++global_iter)
    {
      m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, *global_iter));

      std::vector<std::pair<int, int> > lineSegmentCluster;
      lineSegmentCluster.push_back(std::make_pair(m_ScanSize, global_iter->GetIndex()));
      m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

      m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
      m_LineSegmentClustersIndex++;
    }

    doUpdate = true;
    m_FirstScan = false;
  }
  else
  {
    for (auto global_iter = globalLineSegments.begin(); global_iter != globalLineSegments.end();)
    {
      for (auto inner_iter = m_LineSegmentMap.begin(); inner_iter != m_LineSegmentMap.end();)
      {
        if (CalculateOverlap(inner_iter->second, *global_iter))
        {
          doUpdate = true;
          doMerge = true;
          // candidates.push_back(inner_iter->second);
          assert(m_LineSegmentClusters.find(inner_iter->first) != m_LineSegmentClusters.end());
          std::vector<std::pair<int, int> > originalLineSegments =
              (m_LineSegmentClusters.find(inner_iter->first))->second;
          for (auto iterator = originalLineSegments.begin(); iterator != originalLineSegments.end(); ++iterator)
          {
            LocalizedRangeScanWithLines* scan = rScans[iterator->first];
            assert(scan != NULL);
            candidates.push_back(scan->GetGlobalLineSegments().at(iterator->second));
          }

          if (index == -1)  // 第一次检测到
          {
            candidates.push_back(*global_iter);
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
      }  // 内层完成一次遍历

      if (doMerge)
      {
        LineSegment updatedLine;
        MergeLineSegments(candidates, updatedLine);
        m_LineSegmentMap[index] = updatedLine;

        auto objectCluster = m_LineSegmentClusters.find(index);
        assert(objectCluster != m_LineSegmentClusters.end());
        // 将candidateIndex移入index索引对应的数组
        for (std::vector<int>::const_iterator index_iter = candidateIndex.begin(); index_iter != candidateIndex.end();
             ++index_iter)
        {
          auto candidateCluster = m_LineSegmentClusters.find(*index_iter);
          assert(candidateCluster != m_LineSegmentClusters.end());

          for (auto candidateClusterIter = (candidateCluster->second).begin();
               candidateClusterIter != (candidateCluster->second).end(); ++candidateClusterIter)
          {
            (objectCluster->second).push_back(*candidateClusterIter);
          }
        }
        (objectCluster->second).push_back(std::make_pair(m_ScanSize, global_iter->GetIndex()));

        bool flag = false;
        std::vector<int>::iterator index_iter;
        std::map<int, std::vector<std::pair<int, int> > >::iterator cluster_iter;
        for (index_iter = m_ClustersIndexArray.begin(), cluster_iter = m_LineSegmentClusters.begin();
             index_iter != m_ClustersIndexArray.end(), cluster_iter != m_LineSegmentClusters.end();)
        {
          for (auto candidate_iter = candidateIndex.begin(); candidate_iter != candidateIndex.end();)
          {
            if (*index_iter == *candidate_iter)
            {
              flag = true;
              index_iter = m_ClustersIndexArray.erase(index_iter);
              cluster_iter = m_LineSegmentClusters.erase(cluster_iter);
              candidate_iter = candidateIndex.erase(candidate_iter);
              break;
            }
            else
            {
              ++candidate_iter;
            }
          }

          if (flag)
          {
            flag = false;
          }
          else
          {
            ++index_iter;
            ++cluster_iter;
          }
        }

        global_iter = globalLineSegments.erase(global_iter);
      }
      else
      {
        ++global_iter;
      }

      // refresh
      doMerge = false;
      index = -1;
      candidates.clear();
      assert(candidateIndex.empty());
    }

    for (auto global_iter = globalLineSegments.begin(); global_iter != globalLineSegments.end(); ++global_iter)
    {
      m_LineSegmentMap.insert(std::make_pair(m_LineSegmentClustersIndex, *global_iter));

      std::vector<std::pair<int, int> > lineSegmentCluster;
      lineSegmentCluster.push_back(std::make_pair(m_ScanSize, global_iter->GetIndex()));
      m_LineSegmentClusters.insert(std::make_pair(m_LineSegmentClustersIndex, lineSegmentCluster));

      m_ClustersIndexArray.push_back(m_LineSegmentClustersIndex);
      m_LineSegmentClustersIndex++;
    }
  }

  // 读取新的一帧激光数据
  m_ScanSize++;
  assert(updateCheck());

  return doUpdate;
}

void LineSegmentMap::GlobalMapAdjustment(const LocalizedRangeScanWithLinesVector& rScans)
{
  assert(!rScans.empty());
  std::cout << "Global map adjustment called." << std::endl;
  std::vector<std::pair<int, LineSegment> > loopChainIndex;
  // #pragma omp parallel for
  for (int i = 0; i < static_cast<int>(m_ClustersIndexArray.size()); i++)
  {
    MapAdjustment(rScans, m_ClustersIndexArray[i], loopChainIndex);
  }

  bool doMerge = false;
  int index = -1;
  LineSegmentVector candidates;     // 待merge的线段
  std::vector<int> candidateIndex;  // 待merge的线段的索引下标

  for (int i = 0; i < static_cast<int>(loopChainIndex.size()); i++)
  {
    LineSegment temp = loopChainIndex.at(i).second;
    for (auto inner_iter = m_LineSegmentMap.begin(); inner_iter != m_LineSegmentMap.end();)
    {
      if (CalculateOverlap(inner_iter->second, temp))
      {
        doMerge = true;
        // candidates.push_back(inner_iter->second);
        assert(m_LineSegmentClusters.find(inner_iter->first) != m_LineSegmentClusters.end());
        std::vector<std::pair<int, int> > originalLineSegments =
            (m_LineSegmentClusters.find(inner_iter->first))->second;

        for (auto iterator = originalLineSegments.begin(); iterator != originalLineSegments.end(); ++iterator)
        {
          LocalizedRangeScanWithLines* scan = rScans[iterator->first];
          assert(scan != NULL);
          candidates.push_back(scan->GetGlobalLineSegments().at(iterator->second));
        }

        if (index == -1)  // 第一次检测到
        {
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
    }  // 内层完成一次遍历

    if (doMerge)
    {
      LineSegment updatedLine;
      MergeLineSegments(candidates, updatedLine);
      m_LineSegmentMap[index] = updatedLine;

      auto objectCluster = m_LineSegmentClusters.find(index);
      assert(objectCluster != m_LineSegmentClusters.end());
      // 将candidateIndex移入index索引对应的数组
      for (std::vector<int>::const_iterator index_iter = candidateIndex.begin(); index_iter != candidateIndex.end();
           ++index_iter)
      {
        auto candidateCluster = m_LineSegmentClusters.find(*index_iter);
        assert(candidateCluster != m_LineSegmentClusters.end());

        for (auto candidateClusterIter = (candidateCluster->second).begin();
             candidateClusterIter != (candidateCluster->second).end(); ++candidateClusterIter)
        {
          (objectCluster->second).push_back(*candidateClusterIter);
        }
      }

      bool flag = false;
      std::vector<int>::iterator index_iter;
      std::map<int, std::vector<std::pair<int, int> > >::iterator cluster_iter;
      for (index_iter = m_ClustersIndexArray.begin(), cluster_iter = m_LineSegmentClusters.begin();
           index_iter != m_ClustersIndexArray.end(), cluster_iter != m_LineSegmentClusters.end();)
      {
        for (auto candidate_iter = candidateIndex.begin(); candidate_iter != candidateIndex.end();)
        {
          if (*index_iter == *candidate_iter)
          {
            flag = true;
            index_iter = m_ClustersIndexArray.erase(index_iter);
            cluster_iter = m_LineSegmentClusters.erase(cluster_iter);
            candidate_iter = candidateIndex.erase(candidate_iter);
            break;
          }
          else
          {
            ++candidate_iter;
          }
        }

        if (flag)
        {
          flag = false;
        }
        else
        {
          ++index_iter;
          ++cluster_iter;
        }
      }
    }

    // refresh
    doMerge = false;
    index = -1;
    candidates.clear();
    assert(candidateIndex.empty());
  }

  std::cout << "Global map adjustment done." << std::endl;
  assert(updateCheck());
}

bool LineSegmentMap::CalculateOverlap(const LineSegment& prevLineSegment, const LineSegment& currLineSegment)
{
  double headingDeviation = math::NormalizeAngle(currLineSegment.GetHeading() - prevLineSegment.GetHeading());
  if (fabs(headingDeviation) > math::DegreesToRadians(4.0))  // 当前线段与地图中的线段不平行
  {
    return false;
  }

  double A, B, C;

  // 考虑直线斜率存在与否，分两种情况讨论，最终化为直线的一般式
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

  double dist1 = fabs(A * currLineSegment.GetStartPoint().GetX() + B * currLineSegment.GetStartPoint().GetY() + C) /
                 sqrt(A * A + B * B);
  double dist2 = fabs(A * currLineSegment.GetEndPoint().GetX() + B * currLineSegment.GetEndPoint().GetY() + C) /
                 sqrt(A * A + B * B);
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

void LineSegmentMap::MapAdjustment(const LocalizedRangeScanWithLinesVector& rScans, int index,
                                   std::vector<std::pair<int, LineSegment> >& rLoopChainIndex)
{
  std::vector<std::pair<int, int> > lineSegmentCluster = m_LineSegmentClusters.find(index)->second;
  bool withinChain = false;
  int size = static_cast<int>(rScans.size());
  LineSegmentVector globalLineSegments;

  for (int j = 0; j < static_cast<int>(lineSegmentCluster.size()); j++)
  {
    if (size - lineSegmentCluster[j].first <= 70)
    {
      withinChain = true;
    }
    LocalizedRangeScanWithLines* pScan = rScans[lineSegmentCluster[j].first];
    assert(pScan != NULL);
    globalLineSegments.push_back(pScan->GetGlobalLineSegments().at(lineSegmentCluster[j].second));
  }

  LineSegment output;
  MergeLineSegments(globalLineSegments, output);
  m_LineSegmentMap[m_LineSegmentClusters.find(index)->first] = output;

  if (withinChain)
  {
    rLoopChainIndex.push_back(std::pair<int, LineSegment>(m_LineSegmentClusters.find(index)->first, output));
  }
}

/**
 * Merge linesegments
 */
void LineSegmentMap::MergeLineSegments(const LineSegmentVector& lineSegments, LineSegment& output)
{
  assert(!lineSegments.empty());
  Vector2<double> directionVector;
  Vector2<double> centralPoint;
  double role = 0.0;
  double weight = 0.0;
  double thetaY = 0.0;
  double thetaX = 0.0;
  int updateRate = 0;
  for (LineSegmentVector::const_iterator iter = lineSegments.begin(); iter != lineSegments.end(); ++iter)
  {
    directionVector += iter->GetDirectionVector();
    centralPoint += iter->GetBarycenter();

    role += iter->GetRole() * iter->GetLength();
    weight += iter->GetLength();
    thetaX += cos(iter->GetPhi()) * iter->GetLength();
    thetaY += sin(iter->GetPhi()) * iter->GetLength();
    updateRate += iter->GetUpdateTimes();
  }
  // 计算线段的单位方向向量
  double vectorLength = directionVector.Length();  // 归一化因子
  assert(fabs(vectorLength) > KT_TOLERANCE);

  double heading = atan2(directionVector.GetY(), directionVector.GetX());
  assert(math::InRange(heading, -KT_PI, KT_PI));

  Vector2<double> barycenter = centralPoint / static_cast<int>(lineSegments.size());

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
  for (LineSegmentVector::const_iterator iter = lineSegments.begin(); iter != lineSegments.end(); ++iter)
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
  for (PointVectorDouble::const_iterator iter = startPointVector.begin(); iter != startPointVector.end(); ++iter)
  {
    for (PointVectorDouble::const_iterator inner_iter = endPointVector.begin(); inner_iter != endPointVector.end();
         ++inner_iter)
    {
      double distance = iter->Distance(*inner_iter);
      if (distance > maximumDistance)
      {
        maximumDistance = distance;
        updatedStartPoint = *iter;
        updatedEndPoint = *inner_iter;
      }
    }
  }

  output = LineSegment(updatedStartPoint, updatedEndPoint);
  output.SetUpdateTimes(updateRate);
}

bool LineSegmentMap::updateCheck() const
{
  for (auto iter = m_LineSegmentMap.begin(); iter != m_LineSegmentMap.end(); ++iter)
  {
    int updateNum = static_cast<int>(m_LineSegmentClusters.find(iter->first)->second.size());
    if (updateNum != iter->second.GetUpdateTimes())
    {
      return false;
    }
  }
  return true;
}

}  // namespace karto