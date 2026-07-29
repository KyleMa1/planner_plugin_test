// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_rtk_spline_planner/cubic_spline.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace nav2_rtk_spline_planner
{

// ============================================================================
// CubicSpline1D — Natural cubic spline (tridiagonal solver)
// ============================================================================

void CubicSpline1D::build(
  const std::vector<double> & t,
  const std::vector<double> & y)
{
  if (t.size() < 2 || t.size() != y.size()) {
    throw std::invalid_argument("CubicSpline1D::build requires >= 2 matching knots");
  }

  const int n = static_cast<int>(t.size());
  t_ = t;
  a_ = y;

  // h[i] = t[i+1] - t[i]
  std::vector<double> h(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    h[i] = t[i + 1] - t[i];
    if (h[i] <= 0.0) {
      throw std::invalid_argument("CubicSpline1D: knots must be strictly increasing");
    }
  }

  // Solve tridiagonal system for c_ (second derivatives / 2)
  // Natural boundary: c_[0] = c_[n-1] = 0
  c_.assign(n, 0.0);

  if (n > 2) {
    const int m = n - 2;
    std::vector<double> rhs(m), diag(m), upper(m), lower(m);

    for (int i = 0; i < m; ++i) {
      diag[i] = 2.0 * (h[i] + h[i + 1]);
      rhs[i] = 3.0 * ((a_[i + 2] - a_[i + 1]) / h[i + 1] -
        (a_[i + 1] - a_[i]) / h[i]);
    }
    for (int i = 0; i < m - 1; ++i) {
      upper[i] = h[i + 1];
      lower[i + 1] = h[i + 1];
    }

    // Thomas algorithm (forward sweep)
    for (int i = 1; i < m; ++i) {
      double w = lower[i] / diag[i - 1];
      diag[i] -= w * upper[i - 1];
      rhs[i] -= w * rhs[i - 1];
    }

    // Back substitution
    c_[m] = rhs[m - 1] / diag[m - 1];
    for (int i = m - 2; i >= 0; --i) {
      c_[i + 1] = (rhs[i] - upper[i] * c_[i + 2]) / diag[i];
    }
  }

  // Compute b_ and d_
  b_.resize(n - 1);
  d_.resize(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    d_[i] = (c_[i + 1] - c_[i]) / (3.0 * h[i]);
    b_[i] = (a_[i + 1] - a_[i]) / h[i] - h[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0;
  }
}

int CubicSpline1D::findSegment(double s) const
{
  if (s <= t_.front()) {return 0;}
  if (s >= t_.back()) {return static_cast<int>(t_.size()) - 2;}

  auto it = std::upper_bound(t_.begin(), t_.end(), s);
  return static_cast<int>(std::distance(t_.begin(), it)) - 1;
}

double CubicSpline1D::evaluate(double s) const
{
  int i = findSegment(s);
  double ds = s - t_[i];
  return a_[i] + b_[i] * ds + c_[i] * ds * ds + d_[i] * ds * ds * ds;
}

double CubicSpline1D::evaluateDerivative(double s) const
{
  int i = findSegment(s);
  double ds = s - t_[i];
  return b_[i] + 2.0 * c_[i] * ds + 3.0 * d_[i] * ds * ds;
}

double CubicSpline1D::evaluateSecondDerivative(double s) const
{
  int i = findSegment(s);
  double ds = s - t_[i];
  return 2.0 * c_[i] + 6.0 * d_[i] * ds;
}

// ============================================================================
// CubicSpline2D — Arc-length parameterized 2-D spline
// ============================================================================

void CubicSpline2D::build(
  const std::vector<double> & x,
  const std::vector<double> & y)
{
  if (x.size() < 2 || x.size() != y.size()) {
    throw std::invalid_argument("CubicSpline2D::build requires >= 2 matching waypoints");
  }

  const int n = static_cast<int>(x.size());

  // Compute cumulative chord-length parameterization
  s_.resize(n);
  s_[0] = 0.0;
  for (int i = 1; i < n; ++i) {
    double dx = x[i] - x[i - 1];
    double dy = y[i] - y[i - 1];
    s_[i] = s_[i - 1] + std::hypot(dx, dy);
  }

  sx_.build(s_, x);
  sy_.build(s_, y);
}

void CubicSpline2D::evaluate(double s, double & x, double & y) const
{
  x = sx_.evaluate(s);
  y = sy_.evaluate(s);
}

double CubicSpline2D::evaluateYaw(double s) const
{
  double dx = sx_.evaluateDerivative(s);
  double dy = sy_.evaluateDerivative(s);
  return std::atan2(dy, dx);
}

double CubicSpline2D::evaluateCurvature(double s) const
{
  double dx = sx_.evaluateDerivative(s);
  double dy = sy_.evaluateDerivative(s);
  double ddx = sx_.evaluateSecondDerivative(s);
  double ddy = sy_.evaluateSecondDerivative(s);
  double denom = std::pow(dx * dx + dy * dy, 1.5);
  if (denom < 1e-12) {return 0.0;}
  return (dx * ddy - dy * ddx) / denom;
}

double CubicSpline2D::findClosest(
  double qx, double qy,
  double s_guess, double search_radius) const
{
  double s_min_bound = 0.0;
  double s_max_bound = totalLength();

  if (search_radius > 0.0) {
    s_min_bound = std::max(0.0, s_guess - search_radius);
    s_max_bound = std::min(totalLength(), s_guess + search_radius);
  }

  // Coarse search: step through at 0.5m resolution
  const double coarse_step = 0.5;
  double best_s = s_min_bound;
  double best_dist2 = std::numeric_limits<double>::max();

  for (double s = s_min_bound; s <= s_max_bound; s += coarse_step) {
    double px, py;
    evaluate(s, px, py);
    double dist2 = (px - qx) * (px - qx) + (py - qy) * (py - qy);
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_s = s;
    }
  }

  // Fine search: golden-section around best_s
  double lo = std::max(s_min_bound, best_s - coarse_step);
  double hi = std::min(s_max_bound, best_s + coarse_step);
  const double gr = 0.6180339887;
  const double tol = 0.01;

  while ((hi - lo) > tol) {
    double s1 = hi - gr * (hi - lo);
    double s2 = lo + gr * (hi - lo);
    double x1, y1, x2, y2;
    evaluate(s1, x1, y1);
    evaluate(s2, x2, y2);
    double d1 = (x1 - qx) * (x1 - qx) + (y1 - qy) * (y1 - qy);
    double d2 = (x2 - qx) * (x2 - qx) + (y2 - qy) * (y2 - qy);
    if (d1 < d2) {
      hi = s2;
    } else {
      lo = s1;
    }
  }

  return (lo + hi) * 0.5;
}

void CubicSpline2D::sample(
  double ds,
  double s_start, double s_end,
  std::vector<double> & xs,
  std::vector<double> & ys,
  std::vector<double> & yaws) const
{
  xs.clear();
  ys.clear();
  yaws.clear();

  if (ds <= 0.0) {return;}

  bool forward = (s_end >= s_start);
  double step = forward ? ds : -ds;
  double s = s_start;

  while ((forward && s <= s_end) || (!forward && s >= s_end)) {
    double x, y;
    evaluate(s, x, y);
    xs.push_back(x);
    ys.push_back(y);
    yaws.push_back(evaluateYaw(s));
    s += step;
  }

  // Ensure the endpoint is included
  if (!xs.empty()) {
    double last_s = forward ? (s - step) : (s - step);
    if (std::abs(last_s - s_end) > 1e-6) {
      double x, y;
      evaluate(s_end, x, y);
      xs.push_back(x);
      ys.push_back(y);
      yaws.push_back(evaluateYaw(s_end));
    }
  }
}

}  // namespace nav2_rtk_spline_planner
