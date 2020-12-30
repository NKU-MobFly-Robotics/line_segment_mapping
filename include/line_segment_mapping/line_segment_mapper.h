#ifndef LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAPPER_H
#define LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAPPER_H

#include "line_segment_mapping/line_segment_map.h"
#include "open_karto/Mapper.h"
#include <boost/shared_ptr.hpp>

namespace karto
{
class LineSegmentMapper
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
  virtual kt_bool Process(LocalizedRangeScanWithLines* pScan);

  /**
   * Returns all processed scans added to the mapper.
   * NOTE: The returned scans have their corrected pose updated.
   * @return list of scans received and processed by the mapper. If no scans have been processed,
   * return an empty list.
   */
  virtual const LocalizedRangeScanVector GetAllProcessedScans() const;

  virtual const LineSegmentMapContainer& GetLineSegmentMap() const;

  /**
   * Set scan optimizer used by mapper when closing the loop
   * @param pSolver
   */
  void SetScanSolver(ScanSolver* pSolver);

public:
  /* Abstract methods for parameter setters and getters */

  /* Getters */
  // General Parameters
  bool getParamUseScanMatching();
  bool getParamUseScanBarycenter();
  double getParamMinimumTimeInterval();
  double getParamMinimumTravelDistance();
  double getParamMinimumTravelHeading();
  int getParamScanBufferSize();
  double getParamScanBufferMaximumScanDistance();
  double getParamLinkMatchMinimumResponseFine();
  double getParamLinkScanMaximumDistance();
  double getParamLoopSearchMaximumDistance();
  bool getParamDoLoopClosing();
  int getParamLoopMatchMinimumChainSize();
  double getParamLoopMatchMaximumVarianceCoarse();
  double getParamLoopMatchMinimumResponseCoarse();
  double getParamLoopMatchMinimumResponseFine();

  // Correlation Parameters - Correlation Parameters
  double getParamCorrelationSearchSpaceDimension();
  double getParamCorrelationSearchSpaceResolution();
  double getParamCorrelationSearchSpaceSmearDeviation();

  // Correlation Parameters - Loop Closure Parameters
  double getParamLoopSearchSpaceDimension();
  double getParamLoopSearchSpaceResolution();
  double getParamLoopSearchSpaceSmearDeviation();

  // Scan Matcher Parameters
  double getParamDistanceVariancePenalty();
  double getParamAngleVariancePenalty();
  double getParamFineSearchAngleOffset();
  double getParamCoarseSearchAngleOffset();
  double getParamCoarseAngleResolution();
  double getParamMinimumAnglePenalty();
  double getParamMinimumDistancePenalty();
  bool getParamUseResponseExpansion();

  /* Setters */
  // General Parameters
  void setParamUseScanMatching(bool b);
  void setParamUseScanBarycenter(bool b);
  void setParamMinimumTimeInterval(double d);
  void setParamMinimumTravelDistance(double d);
  void setParamMinimumTravelHeading(double d);
  void setParamScanBufferSize(int i);
  void setParamScanBufferMaximumScanDistance(double d);
  void setParamLinkMatchMinimumResponseFine(double d);
  void setParamLinkScanMaximumDistance(double d);
  void setParamLoopSearchMaximumDistance(double d);
  void setParamDoLoopClosing(bool b);
  void setParamLoopMatchMinimumChainSize(int i);
  void setParamLoopMatchMaximumVarianceCoarse(double d);
  void setParamLoopMatchMinimumResponseCoarse(double d);
  void setParamLoopMatchMinimumResponseFine(double d);

  // Correlation Parameters - Correlation Parameters
  void setParamCorrelationSearchSpaceDimension(double d);
  void setParamCorrelationSearchSpaceResolution(double d);
  void setParamCorrelationSearchSpaceSmearDeviation(double d);

  // Correlation Parameters - Loop Closure Parameters
  void setParamLoopSearchSpaceDimension(double d);
  void setParamLoopSearchSpaceResolution(double d);
  void setParamLoopSearchSpaceSmearDeviation(double d);

  // Scan Matcher Parameters
  void setParamDistanceVariancePenalty(double d);
  void setParamAngleVariancePenalty(double d);
  void setParamFineSearchAngleOffset(double d);
  void setParamCoarseSearchAngleOffset(double d);
  void setParamCoarseAngleResolution(double d);
  void setParamMinimumAnglePenalty(double d);
  void setParamMinimumDistancePenalty(double d);
  void setParamUseResponseExpansion(bool b);

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
  boost::shared_ptr<Mapper> mapper_ptr_;

  boost::shared_ptr<LineSegmentMap> line_segment_map_ptr_;  // 新增加的模块

  std::vector<std::vector<std::pair<int, LineSegment> > > m_ScanLineClusters;

  LocalizedRangeScanWithLinesVector m_AllLineSegments;
};

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_SEGMENT_MAPPER_H