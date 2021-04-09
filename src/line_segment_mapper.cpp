#include "line_segment_mapping/line_segment_mapper.h"
#include <sys/time.h>

namespace karto
{
/**
 * Default constructor
 */
LineSegmentMapper::LineSegmentMapper() : Mapper("Mapper"), m_pLineSegmentMapManager(NULL), m_Initialized(false)
{
}

/**
 * Default constructor
 */
LineSegmentMapper::LineSegmentMapper(const std::string& rName)
  : Mapper(rName), m_pLineSegmentMapManager(NULL), m_Initialized(false)
{
}

/**
 * Destructor
 */
LineSegmentMapper::~LineSegmentMapper()
{
  if (m_pLineSegmentMapManager)
    delete m_pLineSegmentMapManager;
}

kt_bool LineSegmentMapper::Process(LocalizedRangeScan* pScan, LineSegmentPtrVector& rLineSegments)
{
  if (pScan != NULL)
  {
    karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL || pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false)
    {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());

      m_pLineSegmentMapManager = new LineSegmentMapManager();

      m_Initialized = true;
    }

    // get last scan
    // 如果这一帧是第一帧，则pLastScan返回NULL
    LocalizedRangeScan* pLastScan = GetMapperSensorManager()->GetLastScan(pScan->GetSensorName());

    // update scans corrected pose based on last correction
    if (pLastScan != NULL)
    {
      Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      // 根据码盘数据定位
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // test if scan is outside minimum boundary or if heading is larger then minimum heading
    if (!HasMovedEnough(pScan, pLastScan))
    {
      return false;
    }

    Matrix3 covariance;
    covariance.SetToIdentity();

    // correct scan (if not first scan)
    if (getParamUseScanMatching() && pLastScan != NULL)
    {
      Pose2 bestPose;
      // 核心一：扫描匹配
      GetSequentialScanMatcher()->MatchScan(pScan, GetMapperSensorManager()->GetRunningScans(pScan->GetSensorName()),
                                            bestPose, covariance);
      pScan->SetSensorPose(bestPose);  // 位姿更新
    }

    // add scan to buffer and assign id
    GetMapperSensorManager()->AddScan(pScan);

    struct timeval start_t, end_t;

    if (getParamUseScanMatching())
    {
      // add to graph
      GetGraph()->AddVertex(pScan);
      GetGraph()->AddEdges(pScan, covariance);

      GetMapperSensorManager()->AddRunningScan(pScan);

      if (getParamDoLoopClosing())
      {
        std::vector<Name> deviceNames = GetMapperSensorManager()->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          // 核心二：回环检测
          bool update = GetGraph()->TryCloseLoop(pScan, *iter);
          if (update)
          {
            gettimeofday(&start_t, NULL);
            m_pLineSegmentMapManager->GlobalMapAdjustment();
            m_pLineSegmentMapManager->IncrementalMerge(rLineSegments);
            gettimeofday(&end_t, NULL);

            double time_interval = 1e6 * (end_t.tv_sec - start_t.tv_sec) + end_t.tv_usec - start_t.tv_usec;
            std::cout << "Global map adjustment costs " << time_interval / 1e3 << "ms\n";
          }
          else
          {
            // gettimeofday(&start_t, NULL);
            if (m_pLineSegmentMapManager->IncrementalMerge(rLineSegments))
            {
              // gettimeofday(&end_t, NULL);
              // double time_interval = 1e6 * (end_t.tv_sec - start_t.tv_sec) + end_t.tv_usec - start_t.tv_usec;
              // std::cout << "One-to-many incremental line segment merging costs " << time_interval / 1e3 << "ms\n";
            }
          }
        }
      }
    }
    else
    {
      // gettimeofday(&start_t, NULL);
      if (m_pLineSegmentMapManager->IncrementalMerge(rLineSegments))
      {
        // gettimeofday(&end_t, NULL);
        // double time_interval = 1e6 * (end_t.tv_sec - start_t.tv_sec) + end_t.tv_usec - start_t.tv_usec;
        // std::cout << "One-to-many incremental line segment merging costs " << time_interval / 1e3 << "ms\n";
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
kt_bool LineSegmentMapper::HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan)
{
  // test if first scan
  if (pLastScan == NULL)
  {
    return true;
  }

  // test if enough time has passed
  kt_double timeInterval = pScan->GetTime() - pLastScan->GetTime();
  if (timeInterval >= getParamMinimumTimeInterval())
  {
    return true;
  }

  Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
  Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

  // test if we have turned enough
  kt_double deltaHeading = math::NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
  if (fabs(deltaHeading) >= getParamMinimumTravelHeading())
  {
    return true;
  }

  // test if we have moved enough
  kt_double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
  if (squaredTravelDistance >= math::Square(getParamMinimumTravelDistance()) - KT_TOLERANCE)
  {
    return true;
  }

  return false;
}

}  // namespace karto