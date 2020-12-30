#ifndef LINE_SEGMENT_MAPPING_LINE_SEGMENT_EXTRACTOR_H
#define LINE_SEGMENT_MAPPING_LINE_SEGMENT_EXTRACTOR_H

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <assert.h>

#include "open_karto/Math.h"
#include "open_karto/Karto.h"
#include "line_segment_mapping/line_segment_feature.h"

namespace karto
{
typedef struct PointReadings
{
  PointVectorDouble points;
  std::vector<double> ranges;
  std::vector<double> angles;
  int num;
} PointReadings;

//参数，从launch文件中读入
typedef struct Params
{
  double least_thresh;            // 正交拟合阈值
  double min_line_length;         // 拟合线段最短距离
  double predict_distance;        // 真实点与与预测点之间的距离阈值
  unsigned int min_line_points;   // 一条线段包含的激光点个数
  unsigned int seed_line_points;  // 种子线段包含的激光点个数
  double max_line_gap;            // 一条线段连续两点之间的最大距离
} Params;

//直线段信息结构体
typedef struct line
{
  double a;  // 直线参数
  double b;
  double c;
  int left;  // 直线范围
  int right;
} line;

//直线方程式结构体
typedef struct least
{
  double a;
  double b;
  double c;
} least;

class LineSegmentExtractor
{
public:
  // Constructor
  LineSegmentExtractor();
  // Destructor
  ~LineSegmentExtractor();

public:
  void extractLines(const RangeReadingsVector& point_readings, const LaserRangeFinder* laser_range_finder,
                    LineSegmentVector* line_segments);
  //设置参数
  void set_least_threshold(double least_thresh);
  void set_min_line_length(double min_line_length);
  void set_predict_distance(double predict_distance);
  void set_min_line_points(unsigned int min_line_points);
  void set_seed_line_points(unsigned int seed_line_points);
  void set_max_line_gap(double max_line_gap);

private:
  // 初始化参数
  void loadParameters();
  // 通过激光数据的首末索引值进行直线方程的求解
  least leastsquare(int start, int end, int firstfit, const PointReadings* data);
  // 检测种子直线
  bool detectline(const int start, const int num, const PointReadings* data);
  // 通过种子直线，复原出整条直线，并进行最后的求定
  int detectfulline(int start, const PointReadings* data);
  // 整理整条直线
  void cleanline(const PointReadings* data);
  // 删除小于长度阈值的线段
  bool delete_short_line(int n1, int n2, const PointReadings* data);
  //
  void generate(const PointReadings* data, LineSegmentVector* line_segments);

  // 判断线段端点是否是有序的
  void is_sequential();

  void rangeFiltering(const RangeReadingsVector& point_readings, const LaserRangeFinder* laser_range_finder,
                      PointReadings* data);
  void process(const PointReadings* data, LineSegmentVector* line_segments);

private:
  Params params_;
  //线段结构体信息
  std::vector<line> m_line;
  //直线拟合中间传递变量，已设为全局变量
  least m_least;
  //拟合中间变量
  double mid1;
  double mid2;
  double mid3;
  double mid4;
  double mid5;
};

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_SEGMENT_EXTRACTOR_H