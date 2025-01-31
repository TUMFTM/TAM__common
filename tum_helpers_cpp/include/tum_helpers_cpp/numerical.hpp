// Copyright 2023 Simon Sagmeister
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
namespace tam::helpers::numerical
{
template <typename T>
T integration_step_DoPri45(std::function<T(const double, const T)> f, double t, T x, double h)
{
  /*
  Do a single integration step with the Dormand-Prince integratation scheme.

  This integration scheme is usually used in the RK45 integrator.
  Code was taken from:
      https://stackoverflow.com/questions/54494770/how-to-set-fixed-step-size-with-scipy-integrate
  Additional references:
      https://en.wikipedia.org/wiki/List_of_Runge%E2%80%93Kutta_methods#Embedded_methods
      https://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method


  Args:
      f (std::function): callable function with the signature f(t,x)
      t (double): current time step in seconds
      x (T): the eigen array represeting the state vector
      h (double): step size in seconds for the integration step

  Returns:
      T: new state vector after integration step
  */

  // region taken from github
  // https://stackoverflow.com/questions/54494770/how-to-set-fixed-step-size-with-scipy-integrate
  T k1 = f(t, x);
  T k2 = f(t + 1.0 / 5 * h, x + h * (1.0 / 5 * k1));
  T k3 = f(t + 3.0 / 10 * h, x + h * (3.0 / 40 * k1 + 9.0 / 40 * k2));
  T k4 = f(t + 4.0 / 5 * h, x + h * (44.0 / 45 * k1 - 56.0 / 15 * k2 + 32.0 / 9 * k3));
  T k5 =
    f(t + 8.0 / 9 * h,
      x + h * (19372.0 / 6561 * k1 - 25360.0 / 2187 * k2 + 64448.0 / 6561 * k3 - 212.0 / 729 * k4));
  T k6 =
    f(t + h, x + h * (9017.0 / 3168 * k1 - 355.0 / 33 * k2 + 46732.0 / 5247 * k3 + 49.0 / 176 * k4 -
                      5103.0 / 18656 * k5));
  T v5 =
    35.0 / 384 * k1 + 500.0 / 1113 * k3 + 125.0 / 192 * k4 - 2187.0 / 6784 * k5 + 11.0 / 84 * k6;
  // T k7 = f(t + h, x + h * v5);

  T x_new = x + v5 * h;
  return x_new;
  // endregion
}
/**
 * \brief Calculcate distance between consecutive elements
 *
 * \param E Input vector of size n
 * \return Vector of size n-1 that contains the distance between consecutive elements of E
 */
inline Eigen::VectorXd diff(const Eigen::Ref<const Eigen::VectorXd> E)
{
  return E.bottomRows(E.rows() - 1) - E.topRows(E.rows() - 1);
}
/**
 * \brief Calculate Gradient according to numpy.gradient.
 *
 * See: https://numpy.org/doc/stable/reference/generated/numpy.gradient.html
 * The gradient is computed using second order accurate central differencesin the interior points
 * and first order accurate one-sides (forward or backwards) differences at the
 * boundaries. The returned gradient hence has the same shape as the input array.
 *
 * \param f Input vector of size n containing sample points of f(x)
 * \param x According sample points in x direction for f(x). The length of the vector must match the
 * size of f
 * \return Vector of size n gradient of f
 */
inline Eigen::VectorXd gradient(
  const Eigen::Ref<const Eigen::VectorXd> f, const Eigen::Ref<const Eigen::VectorXd> x)
{
  // Todo (Simon): Check if same dimension
  Eigen::VectorXd out(f.size());

  // Calc diff vector from x
  // (Todo (Simon): optimization if constant diff -> use scalar option) see l1079
  Eigen::VectorXd xd = diff(x);

  // calc middle
  auto dx1 = xd.topRows(xd.rows() - 1).array();
  auto dx2 = xd.bottomRows(xd.rows() - 1).array();
  Eigen::VectorXd a = -(dx2) / (dx1 * (dx1 + dx2));
  Eigen::VectorXd b = (dx2 - dx1) / (dx1 * dx2);
  Eigen::VectorXd c = dx1 / (dx2 * (dx1 + dx2));

  out.middleRows(1, f.rows() - 2) =
    a.array() * f.topRows(f.rows() - 2).array() +
    b.array() * f.middleRows(1, f.rows() - 2).array() +
    c.array() * f.bottomRows(f.rows() - 2).array();
  // calc edges (first order)
  out[0] = (f[1] - f[0]) / xd[0];
  out[out.size() - 1] = (f[f.size() - 1] - f[f.size() - 2]) / xd[xd.size() - 1];

  // return
  return out;
}
template <typename _ForwardIterator>
inline int find_bottom_idx(_ForwardIterator __first, _ForwardIterator __last, const double x)
{
  auto iter_geq = std::lower_bound(__first, __last, x);
  if (iter_geq == __first) {
    return 0;
  }
  if (iter_geq == __last) {
    return iter_geq - __first - 2;
  }
  return iter_geq - __first - 1;
}
// xp muss monoton steigend sein
template <typename Ta, typename Tb>
inline double interp(const double x, const Ta & xp, const Tb & fp)
{
  if ((xp.end() - xp.begin()) != (fp.end() - fp.begin())) {
    throw std::invalid_argument("<tam::helper::geometry::interp> xp.size() != fp.size()");
  }
  int i = find_bottom_idx(xp.begin(), xp.end(), x);
  // f = fp(i) + ((x - xp(i)) / (xp(i + 1) - xp(i))) * (fp(i + 1) - fp(i));
  auto fp_idx = fp.begin();
  auto xp_idx = xp.begin();
  return *(fp_idx + i) + ((x - *(xp_idx + i)) / (*(xp_idx + i + 1) - *(xp_idx + i))) *
                           (*(fp_idx + i + 1) - *(fp_idx + i));
}
template <typename Ta, typename Tb>
inline std::vector<double> interp(const std::vector<double> & x, const Ta & xp, const Tb & fp)
{
  std::vector<double> f;
  f.reserve(x.size());
  for (const auto & x_ : x) {
    f.push_back(interp(x_, xp, fp));
  }
  return f;
}
template <typename Ta, typename Tb>
inline Eigen::MatrixXd interp(
  const Eigen::Ref<const Eigen::MatrixXd> x, const Ta & xp, const Tb & fp)
{
  Eigen::MatrixXd f;
  f.resize(x.rows(), x.cols());
  auto f_it = f.reshaped();
  auto x_it = x.reshaped();
  for (int i = 0; i < f.size(); ++i) {
    f_it(i) = interp(x_it(i), xp, fp);
  }
  return f_it.reshaped(x.rows(), x.cols());
}
template <typename T>
inline double interp_from_idx(const T & in, const float idx)
{
  int idx_0 = std::clamp(static_cast<int>(std::floor(idx)), 0, static_cast<int>(in.size() - 2));
  return in[idx_0] + (idx - idx_0) * (in[idx_0 + 1] - in[idx_0]);
}
template <typename T>
inline T interp_from_idx(const std::vector<T> & in, const float idx)
{
  int idx_0 = std::clamp(static_cast<int>(std::floor(idx)), 0, static_cast<int>(in.size() - 2));
  return in[idx_0] + (idx - idx_0) * (in[idx_0 + 1] - in[idx_0]);
}
template <typename T>
inline double sign(T x)
{
  return ((x > 0) - (x < 0));
}
template <typename T>
inline double clip_absolute(T x, T x_abs_max)
{
  return sign(x) * std::min(std::abs(x), x_abs_max);
}
}  // namespace tam::helpers::numerical
