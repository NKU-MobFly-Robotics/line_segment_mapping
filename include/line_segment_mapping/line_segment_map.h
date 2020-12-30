#ifndef LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_H
#define LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_H

#include "open_karto/Karto.h"
#include "line_segment_mapping/line_extractor.h"

namespace karto
{
class LocalizedRangeScanWithLines : public LocalizedRangeScan
{
public:
  /**
   * Constructs a range scan from the given range finder with the given readings
   */
  LocalizedRangeScanWithLines(const Name& rSensorName, const RangeReadingsVector& rReadings)
    : LocalizedRangeScan(rSensorName, rReadings)
  {
    m_pLineFeature = new LineFeature();

    SetLocalPointReadings();

    m_pLineFeature->extractLines(m_LocalLineSegments, m_LocalReliablePoints);
  }

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScanWithLines()
  {
    if (m_pLineFeature)
      delete m_pLineFeature;
  }

public:
  /**
   * Get point readings in local coordinates
   */
  const PointVectorDouble& GetLocalPointReadings(kt_bool wantFiltered = false) const
  {
    if (wantFiltered == true)
    {
      return m_LocalPointReadings;
    }
    else
    {
      return m_UnfilteredLocalPointReadings;
    }
  }

  /**
   * Get line segments in local coordinates
   */
  const LineSegmentVector& GetLocalLineSegments() const
  {
    return m_LocalLineSegments;
  }

  /**
   * Get line segments in global coordinates
   */
  const LineSegmentVector& GetGlobalLineSegments()
  {
    // transform line segments from local coordinates to global coordinates
    SetGlobalLineSegments();

    return m_GlobalLineSegments;
  }

  /**
   * Get reliable points in local coordinates
   */
  const PointVectorDouble& GetLocalReliablePoints() const
  {
    return m_LocalReliablePoints;
  }

  /**
   * Get reliable points in global coordinates
   */
  const PointVectorDouble& GetGlobalReliablePoints()
  {
    // transform reliable points from local coordinates to global coordinates
    SetGlobalReliablePoints();

    return m_GlobalReliablePoints;
  }

private:
  /**
   * Transform the range readings to point readings in the local coordinates
   */
  void SetLocalPointReadings();

  /**
   * Transform line segments from local coordinates to global cooridnate
   */
  void SetGlobalLineSegments();

  /**
   * Transform line segments from local coordinates to global cooridnate
   */
  void SetGlobalReliablePoints();

private:
  LocalizedRangeScanWithLines(const LocalizedRangeScanWithLines&);
  const LocalizedRangeScanWithLines& operator=(const LocalizedRangeScanWithLines&);

private:
  /**
   * Vector of point readings in local coordinates
   */
  PointVectorDouble m_LocalPointReadings;

  /**
   * Vector of unfiltered point readings in local coordinates
   */
  PointVectorDouble m_UnfilteredLocalPointReadings;

  /**
   * Line segment feature extractor
   */
  LineFeature* m_pLineFeature;

  /**
   * Line segments represented in local coordinates
   */
  LineSegmentVector m_LocalLineSegments;

  /**
   * Line segments represented in global coordinates
   */
  LineSegmentVector m_GlobalLineSegments;

  /**
   * Reliable points represented in local coordinates
   */
  PointVectorDouble m_LocalReliablePoints;

  /**
   * Reliable points represented in global coordinates
   */
  PointVectorDouble m_GlobalReliablePoints;

};  // LocalizedRangeScanWithLines

typedef std::vector<LocalizedRangeScanWithLines*> LocalizedRangeScanWithLinesVector;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The LineSegmentMap is consist of a set of LineSegment
 */
class LineSegmentMap
{
public:
  LineSegmentMap()
  {
    m_LineSegmentMap.clear();
    m_LineSegmentClusters.clear();
    m_ClustersIndexArray.clear();

    m_LineSegmentClustersIndex = 0;  // 初始化
    m_ScanSize = 0;
    m_FirstScan = true;
  }

  ~LineSegmentMap()
  {
  }

  /**
   * 增量式地融合线段特征，采用的策略是one-to-one
   */
  bool NaiveMerge(LocalizedRangeScanWithLines* pScan, const LocalizedRangeScanWithLinesVector& rScans);

  /**
   * 增量式地融合线段特征，采用的策略是one-to-many
   * 在这个模块中，我们同时记录和更新每一帧激光提取的每一条线段所属的标签
   */
  bool IncrementalMerge(LocalizedRangeScanWithLines* pScan, const LocalizedRangeScanWithLinesVector& rScans);

  /**
   * 全局地图调整
   */
  void GlobalMapAdjustment(const LocalizedRangeScanWithLinesVector& rScans);

  const LineSegmentMapContainer& GetLineSegmentMap() const
  {
    return m_LineSegmentMap;
  }

  const std::map<int, std::vector<std::pair<int, int> > >& GetLineSegmentClusters() const
  {
    return m_LineSegmentClusters;
  }

  const std::vector<int>& GetClustersIndexArray() const
  {
    return m_ClustersIndexArray;
  }

  const int& GetScanSize() const
  {
    return m_ScanSize;
  }

private:
  // 将当前线段投影到全局线段地图中，计算重叠长度
  bool CalculateOverlap(const LineSegment& prevLineSegment, const LineSegment& currLineSegment);

  void MapAdjustment(const LocalizedRangeScanWithLinesVector& rScans, int index,
                     std::vector<std::pair<int, LineSegment> >& rLoopChainIndex);

  /**
   * Merge linesegments
   */
  void MergeLineSegments(const LineSegmentVector& lineSegments, LineSegment& output);

  bool updateCheck() const;

private:
  LineSegmentMapContainer m_LineSegmentMap;                                 // 记录全局线段地图
  std::map<int, std::vector<std::pair<int, int> > > m_LineSegmentClusters;  // 记录环境中特征的类别
  std::vector<int> m_ClustersIndexArray;                                    // 存储特征类别索引值的数组
  int m_ScanSize;
  int m_LineSegmentClustersIndex;  // 记录特征类别的索引值
  bool m_FirstScan;
};  // LineSegmentMap

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_H