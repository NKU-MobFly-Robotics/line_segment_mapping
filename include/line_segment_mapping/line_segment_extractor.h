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

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "open_karto/Karto.h"
#include "open_karto/Math.h"

#include "line_segment_mapping/line_segment_feature.h"

namespace line_segment_mapping {

typedef struct PointReadings {
  karto::PointVectorDouble points;
  std::vector<double> ranges;
  std::vector<double> angles;
  int num;
} PointReadings;

// 参数，从launch文件中读入
typedef struct Params {
  double least_thresh;      // 正交拟合阈值
  double min_line_length;   // 拟合线段最短距离
  double predict_distance;  // 真实点与与预测点之间的距离阈值
  int min_line_points;      // 一条线段包含的激光点个数
  int seed_line_points;     // 种子线段包含的激光点个数
  double max_line_gap;      // 一条线段连续两点之间的最大距离
} Params;

// 直线段信息结构体
typedef struct line {
  double a;  // 直线参数
  double b;
  double c;
  int left;  // 直线范围
  int right;
} line;

// 直线方程式结构体
typedef struct least {
  double a;
  double b;
  double c;
} least;

class LineSegmentExtractor {
 public:
  // Constructor
  LineSegmentExtractor();
  // Destructor
  ~LineSegmentExtractor() = default;

 public:
  void extract_lines(const karto::RangeReadingsVector& point_readings,
                     const karto::LaserRangeFinder* laser_range_finder,
                     LineSegmentPtrVector* line_segments);
  // 设置参数
  void set_least_threshold(double least_thresh);
  void set_min_line_length(double min_line_length);
  void set_predict_distance(double predict_distance);
  void set_min_line_points(int min_line_points);
  void set_seed_line_points(int seed_line_points);
  void set_max_line_gap(double max_line_gap);

 private:
  // 初始化参数
  void load_parameters();
  // 通过激光数据的首末索引值进行直线方程的求解
  least least_square(int start, int end, int firstfit,
                     const PointReadings& data);
  // 检测种子直线
  bool detect_line(int start, int num, const PointReadings& data);
  // 通过种子直线，复原出整条直线，并进行最后的求定
  int detect_full_line(int start, const PointReadings& data);
  // 整理整条直线
  void clean_line(const PointReadings& data);
  // 删除小于长度阈值的线段
  bool delete_short_line(int n1, int n2, const PointReadings& data);
  //
  void generate(const PointReadings& data, LineSegmentPtrVector* line_segments);

  // 判断线段端点是否是有序的
  void is_sequential();

  void range_filtering(const karto::RangeReadingsVector& point_readings,
                       const karto::LaserRangeFinder* laser_range_finder,
                       PointReadings* data);
  void process(const PointReadings& data, LineSegmentPtrVector* line_segments);

 private:
  Params params_;
  // 线段结构体信息
  std::vector<line> m_line;
  // 直线拟合中间传递变量，已设为全局变量
  least m_least;
  // 拟合中间变量
  double mid1;
  double mid2;
  double mid3;
  double mid4;
  double mid5;
};

}  // namespace line_segment_mapping
