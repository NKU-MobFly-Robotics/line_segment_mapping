#ifndef LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_MANAGER_H
#define LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_MANAGER_H

#include "open_karto/Karto.h"
#include "line_segment_mapping/line_segment_feature.h"

namespace karto
{
/**
 * An incremental and consistent line segment mapping module, please refer the following paper for more detail
 * J. Wen, X. Zhang, H. Gao, J. Yuan, Y. Fang,
 * "CAE_RLSM: Consistent and efficient redundant line segment merging for online feature map building",
 * IEEE Transactions on Instrumentation and Measurement, 2020, 69(7): 4222-4237.
 */
class LineSegmentMapManager
{
public:
  LineSegmentMapManager()
  {
    m_LineSegmentMap.clear();
    m_LineSegmentClusters.clear();
    m_ClustersIndexArray.clear();

    // initialize
    m_LineSegmentClustersIndex = 0;
  }

  virtual ~LineSegmentMapManager()
  {
  }

  /**
   * Incremental line segment mapping based on one-to-one redundant line segment merging strategy
   */
  bool NaiveMerge(const LineSegmentPtrVector& rLineSegments);

  /**
   * Incremental line segment mapping based on one-to-many redundant line segment merging strategy
   */
  bool IncrementalMerge(const LineSegmentPtrVector& rLineSegments);

  /**
   * Global map adjustment based on the optimized robot poses
   */
  void GlobalMapAdjustment();

  const LineSegmentHashTable& GetLineSegmentMap() const
  {
    return m_LineSegmentMap;
  }

  const LineSegmentPtrVectorHashTable& GetLineSegmentClusters() const
  {
    return m_LineSegmentClusters;
  }

  const std::vector<int>& GetClustersIndexArray() const
  {
    return m_ClustersIndexArray;
  }

private:
  /**
   * Transform the current line segments to the world coordinates, and calculate the overlap with the global line
   * segment map
   */
  bool CalculateOverlap(const LineSegment& prevLineSegment, const LineSegment& currLineSegment);

  void MapAdjustment(int index);

  /**
   * Merge linesegments
   */
  LineSegment MergeLineSegments(const std::vector<LineSegment>& rLineSegments);

  bool updateCheck();

private:
  LineSegmentHashTable m_LineSegmentMap;  // Global line segment map, wherein each line segment is represented in the
                                          // world coordinates
  LineSegmentPtrVectorHashTable m_LineSegmentClusters;  // A hashtable used to record the line segment cluster and the
                                                        // corresponding original line segments
  std::vector<int> m_ClustersIndexArray;  // An array used to store the indices of the line segment cluster
  int m_LineSegmentClustersIndex;         // Index of the line segment cluster
};

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAP_MANAGER_H