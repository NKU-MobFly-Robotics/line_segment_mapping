#ifndef LINE_SEGMENT_MAPPING_LINE_EXTRACTOR_H
#define LINE_SEGMENT_MAPPING_LINE_EXTRACTOR_H

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <assert.h>

#include "open_karto/Math.h"
#include "open_karto/Karto.h"

namespace karto
{
typedef struct _CSData
{
  std::vector<double> bearings;   // 角度值
  std::vector<double> cos_value;  // 余弦值
  std::vector<double> sin_value;  // 正弦值
} CSData;

typedef struct _RangeData
{
  std::vector<double> ranges;  // 距离值
  std::vector<double> xs;      // x坐标
  std::vector<double> ys;      // y坐标
} Rangedata;

//参数，从launch文件中读入
typedef struct _Params
{
  double least_thresh;            // 正交拟合阈值
  double min_line_length;         // 拟合线段最短距离
  double predict_distance;        // 真实点与与预测点之间的距离阈值
  unsigned int min_line_points;   // 一条线段包含的激光点个数
  unsigned int seed_line_points;  // 种子线段包含的激光点个数
  double max_line_gap;            // 一条线段连续两点之间的最大距离
} Params;

//直线段信息结构体
typedef struct _line
{
  double a;  // 直线参数
  double b;
  double c;
  int left;  // 直线范围
  int right;
} line;

//直线方程式结构体
typedef struct _least
{
  double a;
  double b;
  double c;
} least;

class LineSegment
{
public:
  LineSegment()
  {
  }
  ~LineSegment()
  {
  }

  LineSegment(const Vector2<double>& startPoint, const Vector2<double>& endPoint, int index = 0)
  {
    m_StartPoint = startPoint;
    m_EndPoint = endPoint;
    m_Barycenter = (m_StartPoint + m_EndPoint) / 2;
    m_DirectionVector = m_EndPoint - m_StartPoint;
    assert(!std::isnan(m_DirectionVector.GetX()) && !std::isnan(m_DirectionVector.GetY()));  // 排除NaN值的情况

    m_Length = sqrt(m_DirectionVector.Length());
    assert(m_Length >= KT_TOLERANCE);

    m_Heading = atan2(m_DirectionVector.GetY(), m_DirectionVector.GetX());
    assert(math::InRange(m_Heading, -KT_PI, KT_PI));
    m_UpdateTimes = 1;

    double A = m_EndPoint.GetY() - m_StartPoint.GetY();
    double B = m_StartPoint.GetX() - m_EndPoint.GetX();
    double C = m_EndPoint.GetX() * m_StartPoint.GetY() - m_StartPoint.GetX() * m_EndPoint.GetY();
    m_Role = fabs(C) / sqrt(math::Square(A) + math::Square(B));

    double x = -A * C / (math::Square(A) + math::Square(B));
    double y = -B * C / (math::Square(A) + math::Square(B));
    m_Phi = atan2(y, x);

    m_Index = index;
  }

public:
  inline const Vector2<double>& GetStartPoint() const
  {
    return m_StartPoint;
  }

  inline const Vector2<double>& GetEndPoint() const
  {
    return m_EndPoint;
  }

  inline const Vector2<double>& GetBarycenter() const
  {
    return m_Barycenter;
  }

  inline const Vector2<double>& GetDirectionVector() const
  {
    return m_DirectionVector;
  }

  inline const double& GetHeading() const
  {
    return m_Heading;
  }

  inline const double& GetLength() const
  {
    return m_Length;
  }

  inline const int& GetUpdateTimes() const
  {
    return m_UpdateTimes;
  }

  inline void SetUpdateTimes(int updateTimes)
  {
    m_UpdateTimes = updateTimes;
  }

  inline const double& GetRole() const
  {
    return m_Role;
  }

  inline const double& GetPhi() const
  {
    return m_Phi;
  }

  inline const int& GetIndex() const
  {
    return m_Index;
  }

  inline void SetIndex(int index)
  {
    m_Index = index;
  }

private:
  Vector2<double> m_StartPoint;
  Vector2<double> m_EndPoint;

  double m_Heading;  // 线段方向
  double m_Length;
  Vector2<double> m_Barycenter;       // 线段的中心点
  Vector2<double> m_DirectionVector;  // 单位方向向量

  int m_UpdateTimes;  // 线段被更新的次数，初始为1
  double m_Role;
  double m_Phi;
  int m_Index;
};  // LineSegment

typedef std::vector<LineSegment> LineSegmentVector;
typedef std::map<int, LineSegment> LineSegmentMapContainer;

class LineFeature
{
public:
  LineFeature();
  //
  ~LineFeature();

public:
  //设置range，每次都需要传递range消息，在主入口函数的回调函数进行
  void setRangeData(const std::vector<double>& ranges, const std::vector<double>& xs, const std::vector<double>& ys,
                    int size);
  //设置bearing
  void setCosSinData(const std::vector<double>& bearings, const std::vector<double>& cos_value,
                     const std::vector<double>& sin_value, const std::vector<int>& indices);
  //返回直线分割结果
  void extractLines(std::vector<LineSegment>& lineSegments, PointVectorDouble& points);
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
  least leastsquare(int, int, int);
  // 检测种子直线
  bool detectline(const int, const int);
  // 通过种子直线，复原出整条直线，并进行最后的求定
  int detectfulline(const int);
  // 整理整条直线
  void cleanline();
  // 删除小于长度阈值的线段
  bool delete_short_line(int, int);
  //
  void generate(std::vector<LineSegment>& lineSegments, PointVectorDouble& points);
  // 判断是不是可靠的特征点
  bool is_valid_endpoint(int endPointIndex, int neighbouendPointIndex, int frontLineIndex, int backLineIndex);
  // 判断线段端点是否是有序的
  void is_sequential();

private:
  Rangedata range_data_;
  CSData cs_data_;
  Params params_;
  int m_NumberOfRangeReadings;
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
};  // LineFeature

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_EXTRACTOR_H
