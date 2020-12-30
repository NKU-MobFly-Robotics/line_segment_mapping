#include "line_segment_mapping/line_segment_mapper.h"

namespace karto
{
/**
 * Default constructor
 */
LineSegmentMapper::LineSegmentMapper()
{
  mapper_ptr_ = boost::shared_ptr<Mapper>(new Mapper());
  line_segment_map_ptr_ = boost::shared_ptr<LineSegmentMap>(new LineSegmentMap());
}

/**
 * Default constructor
 */
LineSegmentMapper::LineSegmentMapper(const std::string& rName)
{
  mapper_ptr_ = boost::shared_ptr<Mapper>(new Mapper(rName));
  line_segment_map_ptr_ = boost::shared_ptr<LineSegmentMap>(new LineSegmentMap());
}

/**
 * Destructor
 */
LineSegmentMapper::~LineSegmentMapper()
{
}

kt_bool LineSegmentMapper::Process(LocalizedRangeScanWithLines* pScan)
{
  if (pScan != NULL)
  {
    LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL || pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (mapper_ptr_->m_Initialized == false)
    {
      // initialize mapper with range threshold from device
      mapper_ptr_->Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    // get last scan
    // 如果这一帧是第一帧，则pLastScan返回NULL
    LocalizedRangeScan* pLastScan = mapper_ptr_->m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());

    // update scans corrected pose based on last correction
    if (pLastScan != NULL)
    {
      Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      // 根据码盘数据定位
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // test if scan is outside minimum boundary or if heading is larger then minimum heading
    if (!mapper_ptr_->HasMovedEnough(pScan, pLastScan))
    {
      return false;
    }

    Matrix3 covariance;
    covariance.SetToIdentity();

    // scan matching
    if (mapper_ptr_->m_pUseScanMatching->GetValue() && pLastScan != NULL)
    {
      Pose2 bestPose;
      // 核心一：扫描匹配
      mapper_ptr_->m_pSequentialScanMatcher->MatchScan(
          pScan, mapper_ptr_->m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()), bestPose, covariance);
      pScan->SetSensorPose(bestPose);  // 位姿更新
    }
    // add scan to buffer and assign id
    mapper_ptr_->m_pMapperSensorManager->AddScan(pScan);

    m_AllLineSegments.push_back(pScan);

    struct timeval start_t, end_t;

    if (mapper_ptr_->m_pUseScanMatching->GetValue())
    {
      // add to graph
      mapper_ptr_->m_pGraph->AddVertex(pScan);
      mapper_ptr_->m_pGraph->AddEdges(pScan, covariance);

      mapper_ptr_->m_pMapperSensorManager->AddRunningScan(pScan);

      if (mapper_ptr_->m_pDoLoopClosing->GetValue())
      {
        std::vector<Name> deviceNames = mapper_ptr_->m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          // 核心二：回环检测
          bool update = mapper_ptr_->m_pGraph->TryCloseLoop(pScan, *iter);
          if (update == true)
          {
            LocalizedRangeScanWithLinesVector scans = m_AllLineSegments;

            gettimeofday(&start_t, 0);
            line_segment_map_ptr_->GlobalMapAdjustment(scans);
            line_segment_map_ptr_->IncrementalMerge(pScan, scans);
            gettimeofday(&end_t, 0);

            double time_interval = 1e6 * (end_t.tv_sec - start_t.tv_sec) + end_t.tv_usec - start_t.tv_usec;
            std::cout << "Global map adjustment costs " << time_interval / 1e3 << "ms\n";
          }
          else
          {
            LocalizedRangeScanWithLinesVector scans = m_AllLineSegments;
            gettimeofday(&start_t, 0);
            if (line_segment_map_ptr_->IncrementalMerge(pScan, scans))
            {
              gettimeofday(&end_t, 0);
              double time_interval = 1e6 * (end_t.tv_sec - start_t.tv_sec) + end_t.tv_usec - start_t.tv_usec;
              // std::cout << "One-to-many incremental line segment merging costs " << time_interval / 1e3 << "ms\n";
            }
          }
        }
      }
      else
      {
        LocalizedRangeScanWithLinesVector scans = m_AllLineSegments;
        gettimeofday(&start_t, 0);
        if (line_segment_map_ptr_->IncrementalMerge(pScan, scans))
        {
          gettimeofday(&end_t, 0);
          double time_interval = 1e6 * (end_t.tv_sec - start_t.tv_sec) + end_t.tv_usec - start_t.tv_usec;
          // std::cout << "One-to-many incremental line segment merging costs " << time_interval / 1e3 << "ms\n";
        }
      }
    }
    else
    {
      LocalizedRangeScanWithLinesVector scans = m_AllLineSegments;
      gettimeofday(&start_t, 0);
      if (line_segment_map_ptr_->IncrementalMerge(pScan, scans))
      {
        gettimeofday(&end_t, 0);
        double time_interval = 1e6 * (end_t.tv_sec - start_t.tv_sec) + end_t.tv_usec - start_t.tv_usec;
        // std::cout << "One-to-many incremental line segment merging costs " << time_interval / 1e3 << "ms\n";
      }
    }

    mapper_ptr_->m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

void LineSegmentMapper::SetScanSolver(ScanSolver* pSolver)
{
  mapper_ptr_->SetScanSolver(pSolver);
}

/**
 * Gets all the processed scans
 * @return all scans
 */
const LocalizedRangeScanVector LineSegmentMapper::GetAllProcessedScans() const
{
  return mapper_ptr_->GetAllProcessedScans();
}

const LineSegmentMapContainer& LineSegmentMapper::GetLineSegmentMap() const
{
  return line_segment_map_ptr_->GetLineSegmentMap();
}

/* Adding in getters and setters here for easy parameter access */

// General Parameters

bool LineSegmentMapper::getParamUseScanMatching()
{
  return static_cast<bool>(mapper_ptr_->m_pUseScanMatching->GetValue());
}

bool LineSegmentMapper::getParamUseScanBarycenter()
{
  return static_cast<bool>(mapper_ptr_->m_pUseScanBarycenter->GetValue());
}

double LineSegmentMapper::getParamMinimumTimeInterval()
{
  return static_cast<double>(mapper_ptr_->m_pMinimumTimeInterval->GetValue());
}

double LineSegmentMapper::getParamMinimumTravelDistance()
{
  return static_cast<double>(mapper_ptr_->m_pMinimumTravelDistance->GetValue());
}

double LineSegmentMapper::getParamMinimumTravelHeading()
{
  return math::RadiansToDegrees(static_cast<double>(mapper_ptr_->m_pMinimumTravelHeading->GetValue()));
}

int LineSegmentMapper::getParamScanBufferSize()
{
  return static_cast<int>(mapper_ptr_->m_pScanBufferSize->GetValue());
}

double LineSegmentMapper::getParamScanBufferMaximumScanDistance()
{
  return static_cast<double>(mapper_ptr_->m_pScanBufferMaximumScanDistance->GetValue());
}

double LineSegmentMapper::getParamLinkMatchMinimumResponseFine()
{
  return static_cast<double>(mapper_ptr_->m_pLinkMatchMinimumResponseFine->GetValue());
}

double LineSegmentMapper::getParamLinkScanMaximumDistance()
{
  return static_cast<double>(mapper_ptr_->m_pLinkScanMaximumDistance->GetValue());
}

double LineSegmentMapper::getParamLoopSearchMaximumDistance()
{
  return static_cast<double>(mapper_ptr_->m_pLoopSearchMaximumDistance->GetValue());
}

bool LineSegmentMapper::getParamDoLoopClosing()
{
  return static_cast<bool>(mapper_ptr_->m_pDoLoopClosing->GetValue());
}

int LineSegmentMapper::getParamLoopMatchMinimumChainSize()
{
  return static_cast<int>(mapper_ptr_->m_pLoopMatchMinimumChainSize->GetValue());
}

double LineSegmentMapper::getParamLoopMatchMaximumVarianceCoarse()
{
  return static_cast<double>(std::sqrt(mapper_ptr_->m_pLoopMatchMaximumVarianceCoarse->GetValue()));
}

double LineSegmentMapper::getParamLoopMatchMinimumResponseCoarse()
{
  return static_cast<double>(mapper_ptr_->m_pLoopMatchMinimumResponseCoarse->GetValue());
}

double LineSegmentMapper::getParamLoopMatchMinimumResponseFine()
{
  return static_cast<double>(mapper_ptr_->m_pLoopMatchMinimumResponseFine->GetValue());
}

// Correlation Parameters - Correlation Parameters

double LineSegmentMapper::getParamCorrelationSearchSpaceDimension()
{
  return static_cast<double>(mapper_ptr_->m_pCorrelationSearchSpaceDimension->GetValue());
}

double LineSegmentMapper::getParamCorrelationSearchSpaceResolution()
{
  return static_cast<double>(mapper_ptr_->m_pCorrelationSearchSpaceResolution->GetValue());
}

double LineSegmentMapper::getParamCorrelationSearchSpaceSmearDeviation()
{
  return static_cast<double>(mapper_ptr_->m_pCorrelationSearchSpaceSmearDeviation->GetValue());
}

// Correlation Parameters - Loop Correlation Parameters

double LineSegmentMapper::getParamLoopSearchSpaceDimension()
{
  return static_cast<double>(mapper_ptr_->m_pLoopSearchSpaceDimension->GetValue());
}

double LineSegmentMapper::getParamLoopSearchSpaceResolution()
{
  return static_cast<double>(mapper_ptr_->m_pLoopSearchSpaceResolution->GetValue());
}

double LineSegmentMapper::getParamLoopSearchSpaceSmearDeviation()
{
  return static_cast<double>(mapper_ptr_->m_pLoopSearchSpaceSmearDeviation->GetValue());
}

// ScanMatcher Parameters

double LineSegmentMapper::getParamDistanceVariancePenalty()
{
  return std::sqrt(static_cast<double>(mapper_ptr_->m_pDistanceVariancePenalty->GetValue()));
}

double LineSegmentMapper::getParamAngleVariancePenalty()
{
  return std::sqrt(static_cast<double>(mapper_ptr_->m_pAngleVariancePenalty->GetValue()));
}

double LineSegmentMapper::getParamFineSearchAngleOffset()
{
  return static_cast<double>(mapper_ptr_->m_pFineSearchAngleOffset->GetValue());
}

double LineSegmentMapper::getParamCoarseSearchAngleOffset()
{
  return static_cast<double>(mapper_ptr_->m_pCoarseSearchAngleOffset->GetValue());
}

double LineSegmentMapper::getParamCoarseAngleResolution()
{
  return static_cast<double>(mapper_ptr_->m_pCoarseAngleResolution->GetValue());
}

double LineSegmentMapper::getParamMinimumAnglePenalty()
{
  return static_cast<double>(mapper_ptr_->m_pMinimumAnglePenalty->GetValue());
}

double LineSegmentMapper::getParamMinimumDistancePenalty()
{
  return static_cast<double>(mapper_ptr_->m_pMinimumDistancePenalty->GetValue());
}

bool LineSegmentMapper::getParamUseResponseExpansion()
{
  return static_cast<bool>(mapper_ptr_->m_pUseResponseExpansion->GetValue());
}

/* Setters for parameters */
// General Parameters
void LineSegmentMapper::setParamUseScanMatching(bool b)
{
  mapper_ptr_->m_pUseScanMatching->SetValue((kt_bool)b);
}

void LineSegmentMapper::setParamUseScanBarycenter(bool b)
{
  mapper_ptr_->m_pUseScanBarycenter->SetValue((kt_bool)b);
}

void LineSegmentMapper::setParamMinimumTimeInterval(double d)
{
  mapper_ptr_->m_pMinimumTimeInterval->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamMinimumTravelDistance(double d)
{
  mapper_ptr_->m_pMinimumTravelDistance->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamMinimumTravelHeading(double d)
{
  mapper_ptr_->m_pMinimumTravelHeading->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamScanBufferSize(int i)
{
  mapper_ptr_->m_pScanBufferSize->SetValue((kt_int32u)i);
}

void LineSegmentMapper::setParamScanBufferMaximumScanDistance(double d)
{
  mapper_ptr_->m_pScanBufferMaximumScanDistance->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamLinkMatchMinimumResponseFine(double d)
{
  mapper_ptr_->m_pLinkMatchMinimumResponseFine->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamLinkScanMaximumDistance(double d)
{
  mapper_ptr_->m_pLinkScanMaximumDistance->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamLoopSearchMaximumDistance(double d)
{
  mapper_ptr_->m_pLoopSearchMaximumDistance->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamDoLoopClosing(bool b)
{
  mapper_ptr_->m_pDoLoopClosing->SetValue((kt_bool)b);
}

void LineSegmentMapper::setParamLoopMatchMinimumChainSize(int i)
{
  mapper_ptr_->m_pLoopMatchMinimumChainSize->SetValue((kt_int32u)i);
}

void LineSegmentMapper::setParamLoopMatchMaximumVarianceCoarse(double d)
{
  mapper_ptr_->m_pLoopMatchMaximumVarianceCoarse->SetValue((kt_double)math::Square(d));
}

void LineSegmentMapper::setParamLoopMatchMinimumResponseCoarse(double d)
{
  mapper_ptr_->m_pLoopMatchMinimumResponseCoarse->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamLoopMatchMinimumResponseFine(double d)
{
  mapper_ptr_->m_pLoopMatchMinimumResponseFine->SetValue((kt_double)d);
}

// Correlation Parameters - Correlation Parameters
void LineSegmentMapper::setParamCorrelationSearchSpaceDimension(double d)
{
  mapper_ptr_->m_pCorrelationSearchSpaceDimension->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamCorrelationSearchSpaceResolution(double d)
{
  mapper_ptr_->m_pCorrelationSearchSpaceResolution->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamCorrelationSearchSpaceSmearDeviation(double d)
{
  mapper_ptr_->m_pCorrelationSearchSpaceSmearDeviation->SetValue((kt_double)d);
}

// Correlation Parameters - Loop Closure Parameters
void LineSegmentMapper::setParamLoopSearchSpaceDimension(double d)
{
  mapper_ptr_->m_pLoopSearchSpaceDimension->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamLoopSearchSpaceResolution(double d)
{
  mapper_ptr_->m_pLoopSearchSpaceResolution->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamLoopSearchSpaceSmearDeviation(double d)
{
  mapper_ptr_->m_pLoopSearchSpaceSmearDeviation->SetValue((kt_double)d);
}

// Scan Matcher Parameters
void LineSegmentMapper::setParamDistanceVariancePenalty(double d)
{
  mapper_ptr_->m_pDistanceVariancePenalty->SetValue((kt_double)math::Square(d));
}

void LineSegmentMapper::setParamAngleVariancePenalty(double d)
{
  mapper_ptr_->m_pAngleVariancePenalty->SetValue((kt_double)math::Square(d));
}

void LineSegmentMapper::setParamFineSearchAngleOffset(double d)
{
  mapper_ptr_->m_pFineSearchAngleOffset->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamCoarseSearchAngleOffset(double d)
{
  mapper_ptr_->m_pCoarseSearchAngleOffset->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamCoarseAngleResolution(double d)
{
  mapper_ptr_->m_pCoarseAngleResolution->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamMinimumAnglePenalty(double d)
{
  mapper_ptr_->m_pMinimumAnglePenalty->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamMinimumDistancePenalty(double d)
{
  mapper_ptr_->m_pMinimumDistancePenalty->SetValue((kt_double)d);
}

void LineSegmentMapper::setParamUseResponseExpansion(bool b)
{
  mapper_ptr_->m_pUseResponseExpansion->SetValue((kt_bool)b);
}

}  // namespace karto