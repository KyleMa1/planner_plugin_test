// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_RTK_SPLINE_PLANNER__CUBIC_SPLINE_HPP_
#define NAV2_RTK_SPLINE_PLANNER__CUBIC_SPLINE_HPP_

#include <vector>
#include <cmath>
#include <stdexcept>

namespace nav2_rtk_spline_planner
{

/**
 * @brief 1-D natural cubic spline interpolator.
 *
 * Given knots (t_i, y_i), builds piecewise cubic polynomials with
 * continuous second derivatives and "natural" boundary (S''=0 at ends).
 */
class CubicSpline1D
{
public:
  CubicSpline1D() = default;

  /**
   * @brief Build spline from parameter values and function values.
   * @param t Strictly increasing parameter knots
   * @param y Corresponding function values (same size as t)
   */
  void build(const std::vector<double> & t, const std::vector<double> & y);

  /**
   * @brief Evaluate the spline at parameter value s.
   */
  double evaluate(double s) const;

  /**
   * @brief Evaluate the first derivative at parameter value s.
   */
  double evaluateDerivative(double s) const;

  /**
   * @brief Evaluate the second derivative at parameter value s.
   */
  double evaluateSecondDerivative(double s) const;

  double tMin() const {return t_.front();}
  double tMax() const {return t_.back();}
  bool empty() const {return t_.empty();}

private:
  int findSegment(double s) const;

  std::vector<double> t_;
  std::vector<double> a_, b_, c_, d_;
};

/**
 * @brief 2-D cubic spline parameterized by cumulative arc length.
 *
 * Fits separate CubicSpline1D for x(s) and y(s), where s is the
 * cumulative chord-length through the input waypoints.
 */
class CubicSpline2D
{
public:
  CubicSpline2D() = default;

  /**
   * @brief Build the 2-D spline from (x, y) waypoints.
   */
  void build(const std::vector<double> & x, const std::vector<double> & y);

  /**
   * @brief Evaluate position at arc-length parameter s.
   */
  void evaluate(double s, double & x, double & y) const;

  /**
   * @brief Evaluate heading (yaw) at arc-length parameter s.
   */
  double evaluateYaw(double s) const;

  /**
   * @brief Evaluate curvature at arc-length parameter s.
   */
  double evaluateCurvature(double s) const;

  /**
   * @brief Find the arc-length parameter of the closest point to (qx, qy).
   * @param qx Query x
   * @param qy Query y
   * @param s_guess Initial guess (defaults to 0)
   * @param search_radius How far from s_guess to search
   * @return Arc-length parameter of closest point
   */
  double findClosest(
    double qx, double qy,
    double s_guess = 0.0, double search_radius = -1.0) const;

  /**
   * @brief Sample the spline at uniform arc-length intervals.
   * @param ds Step size in meters
   * @param s_start Start parameter
   * @param s_end End parameter
   * @param xs Output x coordinates
   * @param ys Output y coordinates
   * @param yaws Output yaw angles
   */
  void sample(
    double ds,
    double s_start, double s_end,
    std::vector<double> & xs,
    std::vector<double> & ys,
    std::vector<double> & yaws) const;

  double totalLength() const {return s_.empty() ? 0.0 : s_.back();}
  bool empty() const {return sx_.empty();}
  const std::vector<double> & arcLengths() const {return s_;}

private:
  CubicSpline1D sx_;
  CubicSpline1D sy_;
  std::vector<double> s_;
};

}  // namespace nav2_rtk_spline_planner

#endif  // NAV2_RTK_SPLINE_PLANNER__CUBIC_SPLINE_HPP_
