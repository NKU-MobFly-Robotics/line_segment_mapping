#ifndef LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAPPER_H
#define LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAPPER_H

#include "line_segment_mapping/line_segment_map_manager.h"
#include "open_karto/Mapper.h"

namespace karto
{
class LineSegmentMapper : public Mapper
{
public:
  /**
   * Default constructor
   */
  LineSegmentMapper();

  /**
   * Constructor a mapper with a name
   * @param rName mapper name
   */
  LineSegmentMapper(const std::string& rName);

  /**
   * Destructor
   */
  virtual ~LineSegmentMapper();

public:
  /**
   * Process a localized range scan for incorporation into the map.  The scan must
   * be identified with a range finder device.  Once added to a map, the corrected pose information in the
   * localized scan will be updated to the correct pose as determined by the mapper.
   *
   * @param pScan A localized range scan that has pose information associated directly with the scan data.  The pose
   * is that of the range device originating the scan.  Note that the mapper will set corrected pose
   * information in the scan object.
   *
   * @return true if the scan was added successfully, false otherwise
   */
  virtual kt_bool Process(LocalizedRangeScan* pScan, LineSegmentPtrVector& rLineSegments);

  virtual LineSegmentMapManager* GetLineSegmentMapManager() const
  {
    return m_pLineSegmentMapManager;
  }

private:
  /**
   * Test if the scan is "sufficiently far" from the last scan added.
   * @param pScan scan to be checked
   * @param pLastScan last scan added to mapper
   * @return true if the scan is "sufficiently far" from the last scan added or
   * the scan is the first scan to be added
   */
  kt_bool HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan);

private:
  /**
   * Restrict the copy constructor
   */
  LineSegmentMapper(const LineSegmentMapper&);

  /**
   * Restrict the assignment operator
   */
  const LineSegmentMapper& operator=(const LineSegmentMapper&);

private:
  LineSegmentMapManager* m_pLineSegmentMapManager;  // 新增加的模块
  bool m_Initialized;
};

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAPPER_H