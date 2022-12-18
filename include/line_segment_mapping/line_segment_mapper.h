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
#include <string>

#include "open_karto/Mapper.h"

#include "line_segment_mapping/line_segment_map_manager.h"

namespace karto {
class LineSegmentMapper : public Mapper {
 public:
  /**
   * Default constructor
   */
  LineSegmentMapper();

  /**
   * Constructor a mapper with a name
   * @param rName mapper name
   */
  explicit LineSegmentMapper(const std::string& rName);

  /**
   * Destructor
   */
  virtual ~LineSegmentMapper() = default;

 public:
  /**
   * Process a localized range scan for incorporation into the map.  The scan
   * must be identified with a range finder device.  Once added to a map, the
   * corrected pose information in the localized scan will be updated to the
   * correct pose as determined by the mapper.
   *
   * @param pScan A localized range scan that has pose information associated
   * directly with the scan data.  The pose is that of the range device
   * originating the scan.  Note that the mapper will set corrected pose
   * information in the scan object.
   *
   * @return true if the scan was added successfully, false otherwise
   */
  virtual kt_bool Process(LocalizedRangeScan* pScan,
                          const LineSegmentPtrVector& rLineSegments);

  virtual LineSegmentMapManager* GetLineSegmentMapManager() const {
    return m_pLineSegmentMapManager.get();
  }

 private:
  /**
   * Test if the scan is "sufficiently far" from the last scan added.
   * @param pScan scan to be checked
   * @param pLastScan last scan added to mapper
   * @return true if the scan is "sufficiently far" from the last scan added or
   * the scan is the first scan to be added
   */
  kt_bool HasMovedEnough(LocalizedRangeScan* pScan,
                         LocalizedRangeScan* pLastScan);

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
  std::unique_ptr<LineSegmentMapManager> m_pLineSegmentMapManager =
      nullptr;  // 新增加的模块
  bool m_Initialized = false;
};

}  // namespace karto
