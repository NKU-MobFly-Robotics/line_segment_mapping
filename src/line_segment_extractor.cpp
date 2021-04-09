#include "line_segment_mapping/line_segment_extractor.h"

namespace karto
{
//构造函数
LineSegmentExtractor::LineSegmentExtractor()
{
  loadParameters();
}

//析构函数
LineSegmentExtractor::~LineSegmentExtractor()
{
}

void LineSegmentExtractor::loadParameters()
{
  params_.least_thresh = 0.03;
  params_.min_line_length = 0.6;
  params_.min_line_points = 10;
  params_.predict_distance = 0.1;
  params_.seed_line_points = 6;
  params_.max_line_gap = 0.3;
}

// set paramters
void LineSegmentExtractor::set_least_threshold(double least_thresh)
{
  params_.least_thresh = least_thresh;
}

void LineSegmentExtractor::set_min_line_length(double min_line_length)
{
  params_.min_line_length = min_line_length;
}

void LineSegmentExtractor::set_predict_distance(double predict_distance)
{
  params_.predict_distance = predict_distance;
}

void LineSegmentExtractor::set_min_line_points(int min_line_points)
{
  params_.min_line_points = min_line_points;
}

void LineSegmentExtractor::set_seed_line_points(int seed_line_points)
{
  params_.seed_line_points = seed_line_points;
}

void LineSegmentExtractor::set_max_line_gap(double max_line_gap)
{
  params_.max_line_gap = max_line_gap;
}

void LineSegmentExtractor::rangeFiltering(const RangeReadingsVector& point_readings,
                                          const LaserRangeFinder* laser_range_finder, PointReadings& data)
{
  double range_thr = laser_range_finder->GetRangeThreshold();
  double angle_min = laser_range_finder->GetMinimumAngle();
  double range_min = laser_range_finder->GetMinimumRange();
  double angle_increment = laser_range_finder->GetAngularResolution();

  // compute point readings
  data.points.clear();
  data.ranges.clear();
  data.angles.clear();
  data.num = 0;

  for (int i = 0; i < laser_range_finder->GetNumberOfRangeReadings(); i++)
  {
    double range = point_readings[i];
    double angle = angle_min + i * angle_increment;

    if (!math::InRange(range, range_min, range_thr))
      continue;

    Vector2<double> point;
    point.SetX(range * cos(angle));
    point.SetY(range * sin(angle));

    data.points.push_back(point);
    data.ranges.push_back(range);
    data.angles.push_back(angle);
    data.num++;
  }
}

void LineSegmentExtractor::extractLines(const RangeReadingsVector& point_readings,
                                        const LaserRangeFinder* laser_range_finder, LineSegmentPtrVector& line_segments)
{
  PointReadings data;

  rangeFiltering(point_readings, laser_range_finder, data);

  process(data, line_segments);
}

//一次最小二乘求解直线参数
least LineSegmentExtractor::leastsquare(int start, int end, int firstfit, const PointReadings& data)
{
  double w1 = 0, w2 = 0, w3 = 0;
  least temp;
  double n = end - start + 1;

  if (firstfit == 1)
  {
    mid1 = 0;
    mid2 = 0;
    mid3 = 0;
    mid4 = 0;
    mid5 = 0;
    int k = 0;
    for (k = start; k <= end; k++)
    {
      mid1 += data.points[k].GetX();
      mid2 += data.points[k].GetY();
      mid3 += data.points[k].GetX() * data.points[k].GetX();
      mid4 += data.points[k].GetY() * data.points[k].GetY();
      mid5 += data.points[k].GetX() * data.points[k].GetY();
    }
  }
  else
  {
    if (firstfit == 2)
    {
      mid1 += data.points[end].GetX();
      mid2 += data.points[end].GetY();
      mid3 += data.points[end].GetX() * data.points[end].GetX();
      mid4 += data.points[end].GetY() * data.points[end].GetY();
      mid5 += data.points[end].GetX() * data.points[end].GetY();
    }
    else
    {
      mid1 += data.points[start].GetX();
      mid2 += data.points[start].GetY();
      mid3 += data.points[start].GetX() * data.points[start].GetX();
      mid4 += data.points[start].GetY() * data.points[start].GetY();
      mid5 += data.points[start].GetX() * data.points[start].GetY();
    }
  }
  w1 = n * mid5 - mid1 * mid2;
  w2 = mid2 * mid2 - n * mid4 - mid1 * mid1 + n * mid3;
  w3 = mid1 * mid2 - n * mid5;
  // ax+by+c = 0 等价于 y = kx + b;kx-y + b = 0 //a = k,c = b,b=-1
  if (w1 == 0)
  {
    temp.a = -1;
    temp.b = 0;
    temp.c = mid1 / n;
  }
  else
  {
    temp.a = (-w2 + sqrt(w2 * w2 - 4 * w1 * w3)) / 2.0 / w1;
    temp.b = -1;
    temp.c = (mid2 - temp.a * mid1) / n;
  }
  // temp.a = (n*mid5-mid1*mid2)/(n*mid3-mid1*mid1);
  // temp.b = -1;
  // temp.c = (mid2-temp.a*mid1)/n;

  return temp;
}

//判断下一个点是否在直线上，是，返回true；否则，返回false。
bool LineSegmentExtractor::detectline(int start, int num, const PointReadings& data)
{
  bool flag = false;
  //定义点到直线的垂直误差
  double error1 = 0.0;
  //定义下一点到预测位置的误差
  double error2 = 0.0;
  // 定义连续两点之间的距离
  double dist = 0.0;
  int k = 0;
  //预测下一点位置
  Vector2<double> m_pn;

  //下一点，y = kp*x;
  double kp = 0;
  double theta = 0;
  for (k = start; k < start + num; k++)
  {
    //到直线的垂直距离
    error1 = point_to_line_distance(m_least.a, m_least.b, m_least.c, data.points[k].GetX(), data.points[k].GetY());
    if (error1 > params_.least_thresh)
    {
      return false;
    }

    theta = data.angles[k];
    if (fabs(fabs(theta) - KT_PI_2) < KT_TOLERANCE)
    {
      m_pn.SetX(0);
      m_pn.SetY(m_least.c);
    }
    else
    {
      kp = tan(theta);
      m_pn.SetX(m_least.c / (kp - m_least.a));
      m_pn.SetY(kp * m_pn.GetX());
    }

    //计算到预测点之间的误差
    error2 = (m_pn - data.points[k]).Length();
    if (error2 > params_.predict_distance)  //与预测点之间的距离
    {
      return false;
    }

    //计算连续两点之间的距离
    if (k != start + num - 1)
    {
      dist = (data.points[k + 1] - data.points[k]).Length();
      if (dist > params_.max_line_gap)
      {
        return false;
      }
    }
  }

  return true;
}

//检测完整的直线段
int LineSegmentExtractor::detectfulline(int start, const PointReadings& data)
{
  line m_temp;

  bool flag1 = true;
  bool flag2 = true;
  int n1 = 0;
  int n2 = 0;
  double a = 0;
  double b = 0;
  double c = 0;

  a = m_least.a;
  b = m_least.b;
  c = m_least.c;

  n2 = start + params_.seed_line_points;
  least m_result;
  m_result.a = 0;
  m_result.b = 0;
  m_result.c = 0;
  double dist;

  //向前生长
  while (flag2)
  {
    if (n2 <= (data.num - 1))
    {
      dist = (data.points[n2] - data.points[n2 - 1]).Length();
      if (dist < params_.max_line_gap)
      {
        if (point_to_line_distance(a, b, c, data.points[n2].GetX(), data.points[n2].GetY()) < params_.least_thresh)
        {
          m_least = leastsquare(start, n2, 2, data);

          n2 = n2 + 1;
          a = m_least.a;
          b = m_least.b;
          c = m_least.c;
        }
        else
        {
          flag2 = false;
        }
      }
      else
      {
        flag2 = false;
      }
    }
    else
    {
      flag2 = false;
    }
  }
  n2 = n2 - 1;

  //向后回溯
  n1 = start - 1;
  while (flag1)
  {
    if (n1 >= 0)
    {
      dist = (data.points[n1 + 1] - data.points[n1]).Length();
      if (dist < params_.max_line_gap)
      {
        if (point_to_line_distance(a, b, c, data.points[n1].GetX(), data.points[n1].GetY()) < params_.least_thresh)
        {
          m_least = leastsquare(n1, n2, 3, data);

          n1 = n1 - 1;
          a = m_least.a;
          b = m_least.b;
          c = m_least.c;
        }
        else
        {
          flag1 = false;
        }
      }
      else
      {
        flag1 = false;
      }
    }
    else
    {
      flag1 = false;
    }
  }
  n1 = n1 + 1;

  m_temp.left = n1;
  m_temp.right = n2;
  //此处是统一再做一次拟合，可能以一定步长进行拟合搜索的时候，需要这样完整的拟合过程，此时不需要
  m_result = leastsquare(n1, n2, 1, data);
  m_temp.a = m_result.a;
  m_temp.b = m_result.b;
  m_temp.c = m_result.c;

  if ((n2 - n1 + 1) >= params_.min_line_points)
  {
    if (delete_short_line(n1, n2, data))
    {
      m_line.push_back(m_temp);
    }
    return n2;
  }
  else
  {
    return start;
  }
}

void LineSegmentExtractor::cleanline(const PointReadings& data)
{
  if (m_line.size() < 2)
  {
    return;
  }

  int m = 0;
  int n = 0;
  int m_iter = 0;
  double error1 = 0;
  double error2 = 0;
  int line_temp = 0;
  least temp_least;
  temp_least.a = 0;
  temp_least.b = 0;
  temp_least.c = 0;

  double theta_one_ = 0;
  double theta_two_ = 0;
  double theta_d_ = 0;
  std::size_t q = 0, p = 0;

  for (p = 0; p < m_line.size() - 1; p++)
  {
    m = m_line[p].right;
    for (q = p + 1; q < m_line.size(); q++)
    {
      n = m_line[q].left;
      if (m >= n)
      {
        theta_one_ = atan(m_line[p].a);
        theta_two_ = atan(m_line[q].a);

        theta_d_ = fabs(theta_one_ - theta_two_);

        if ((theta_d_ < 0.1) || (theta_d_ > (KT_PI - 0.1)))
        {
          int _left = math::Minimum(m_line[p].left, m_line[q].left);

          least m_temp = leastsquare(_left, m_line[q].right, 1, data);

          m_line[p].a = m_temp.a;
          m_line[p].b = m_temp.b;
          m_line[p].c = m_temp.c;

          m_line[p].left = _left;
          m_line[p].right = m_line[q].right;

          m_line.erase(m_line.begin() + q);

          m = m_line[p].right;
          q = q - 1;
        }
      }
    }
  }

  //处理有相互链接关系的线段
  for (p = 0; p < (m_line.size() - 1); p++)
  {
    q = p + 1;
    m = m_line[p].right;
    n = m_line[q].left;
    if (m >= n)
    {
      for (m_iter = n; m_iter <= m; m_iter++)
      {
        line_temp = m_iter;
        error1 = point_to_line_distance(m_line[p].a, m_line[p].b, m_line[p].c, data.points[m_iter].GetX(),
                                        data.points[m_iter].GetY());
        error2 = point_to_line_distance(m_line[q].a, m_line[q].b, m_line[q].c, data.points[m_iter].GetX(),
                                        data.points[m_iter].GetY());
        if (error1 > error2)
        {
          break;
        }
      }
      m_line[p].right = m_iter - 1;
      temp_least = leastsquare(m_line[p].left, m_line[p].right, 1, data);
      m_line[p].a = temp_least.a;
      m_line[p].b = temp_least.b;
      m_line[p].c = temp_least.c;

      m_line[q].left = m_iter;
      temp_least = leastsquare(m_line[q].left, m_line[q].right, 1, data);
      m_line[q].a = temp_least.a;
      m_line[q].b = temp_least.b;
      m_line[q].c = temp_least.c;
    }
  }
}

bool LineSegmentExtractor::delete_short_line(int n1, int n2, const PointReadings& data)
{
  //删除一些长度小于0.6m的线段，这个可以视具体情况而定
  double dist = (data.points[n2] - data.points[n1]).Length();
  if (dist < params_.min_line_length)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void LineSegmentExtractor::is_sequential()
{
  for (auto iter = m_line.begin(); iter != m_line.end() - 1; ++iter)
  {
    int m1 = iter->left;
    int n1 = iter->right;
    assert(m1 < n1);
    for (auto inner_iter = iter + 1; inner_iter != m_line.end(); ++inner_iter)
    {
      int m2 = inner_iter->left;
      int n2 = inner_iter->right;
      assert(m2 < n2);
      assert(n1 < m2);
    }
  }
}

void LineSegmentExtractor::generate(const PointReadings& data, LineSegmentPtrVector& line_segments)
{
  Vector2<double> endpoint1;
  Vector2<double> endpoint2;
  line_segments.clear();

  int m = 0, n = 0;
  double k1 = 0, k2 = 0;
  int index = 0;
  for (int i = 0; i < m_line.size(); i++)
  {
    m = m_line[i].left;
    n = m_line[i].right;
    assert(m < n);

    if (m_line[i].b != 0)
    {
      endpoint1.SetX((data.points[m].GetX() / m_line[i].a + data.points[m].GetY() - m_line[i].c) /
                     (m_line[i].a + 1.0 / (m_line[i].a)));
      endpoint1.SetY(m_line[i].a * endpoint1.GetX() + m_line[i].c);
    }
    else
    {
      endpoint1.SetX(data.points[m].GetY());
      endpoint1.SetY(m_line[i].c / m_line[i].a);
    }

    if (m_line[i].b != 0)
    {
      endpoint2.SetX((data.points[n].GetX() / m_line[i].a + data.points[n].GetY() - m_line[i].c) /
                     (m_line[i].a + 1.0 / (m_line[i].a)));
      endpoint2.SetY(m_line[i].a * endpoint2.GetX() + m_line[i].c);
    }
    else
    {
      endpoint2.SetX(data.points[n].GetY());
      endpoint2.SetY(m_line[i].c / m_line[i].a);
    }

    LineSegmentPtr line = LineSegmentPtr(new LineSegment(endpoint1, endpoint2, index));

    index += 1;
    line_segments.push_back(line);
  }
}

//识别主函数
void LineSegmentExtractor::process(const PointReadings& data, LineSegmentPtrVector& line_segments)
{
  int line_include = 0;
  m_line.clear();

  if (data.num < params_.min_line_points)
  {
    return;
  }

  //附近特征点数目
  for (int i = 0; i < (data.num - params_.min_line_points); i++)
  {
    m_least = leastsquare(i, i + params_.seed_line_points - 1, 1, data);
    // std::cout << m_least.a << " " << m_least.b << " " << m_least.c << std::endl;
    if (detectline(i, params_.seed_line_points, data))
    {
      line_include = detectfulline(i, data);
      i = line_include;
    }
  }
  cleanline(data);

  for (std::vector<line>::iterator iter = m_line.begin(); iter != m_line.end();)
  {
    if (!delete_short_line(iter->left, iter->right, data))
    {
      iter = m_line.erase(iter);
    }
    else
    {
      ++iter;
    }
  }

  // std::cout << m_line.size() << std::endl;
  // is_sequential();
  generate(data, line_segments);
}

}  // namespace karto