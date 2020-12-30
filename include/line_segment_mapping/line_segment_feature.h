#ifndef LINE_SEGMENT_MAPPING_LINE_SEGMENT_FEATURE_H
#define LINE_SEGMENT_MAPPING_LINE_SEGMENT_FEATURE_H

#include "open_karto/Karto.h"

namespace karto
{
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
};

typedef std::vector<LineSegment> LineSegmentVector;
typedef std::map<int, LineSegment> LineSegmentMapContainer;

}  // namespace karto

#endif  // LINE_SEGMENT_MAPPING_LINE_SEGMENT_FEATURE_H