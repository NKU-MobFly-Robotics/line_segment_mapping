#ifndef LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_H
#define LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_H

#include "open_karto/Karto.h"
#include "line_segment_mapping/line_segment_feature.h"

namespace karto
{
class LocalizedRangeScanWithLines : public LocalizedRangeScan
{
public:
  /**
   * Constructs a range scan from the given range finder with the given readings
   */
  LocalizedRangeScanWithLines(const Name& rSensorName, const RangeReadingsVector& rReadings,
                              const std::vector<LineSegment>& rLineSegments)
    : LocalizedRangeScan(rSensorName, rReadings), m_LocalLineSegments(rLineSegments)
  {
  }

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScanWithLines()
  {
  }

public:
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

private:
  /**
   * Transform line segments from local coordinates to global cooridnate
   */
  void SetGlobalLineSegments();

private:
  LocalizedRangeScanWithLines(const LocalizedRangeScanWithLines&);
  const LocalizedRangeScanWithLines& operator=(const LocalizedRangeScanWithLines&);

private:
  /**
   * Line segments represented in local coordinates
   */
  LineSegmentVector m_LocalLineSegments;

  /**
   * Line segments represented in global coordinates
   */
  LineSegmentVector m_GlobalLineSegments;
};

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
};

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_H