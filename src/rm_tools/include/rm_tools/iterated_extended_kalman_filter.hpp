#ifndef RM_TOOLS__ITERATED_EXTENDED_KALMAN_FILTER_HPP_
#define RM_TOOLS__ITERATED_EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <deque>
#include <functional>
#include <map>
#include <numeric>
#include <string>
#include <utility>

namespace rm_tools
{

class IteratedExtendedKalmanFilter
{
public:
  Eigen::VectorXd X;
  Eigen::MatrixXd P;

  IteratedExtendedKalmanFilter() = default;

  IteratedExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; },
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; })
  : X(x0),
    P(P0),
    I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())),
    x_add_(std::move(x_add)),
    x_subtract_(std::move(x_subtract))
  {
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["nis_fail"] = 0.0;
    data["nees_fail"] = 0.0;
    data["recent_nis_failures"] = 0.0;
  }

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
  {
    return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
  }

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
  {
    P = F * P * F.transpose() + Q;
    X = f(X);
    return X;
  }

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
    int max_iterations = 5, double stop_threshold = 1e-4)
  {
    return update(
      z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract, max_iterations,
      stop_threshold);
  }

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
    int max_iterations = 5, double stop_threshold = 1e-4)
  {
    auto calculate_H = [H](const Eigen::VectorXd &) { return H; };
    return update(z, R, h, calculate_H, z_subtract, max_iterations, stop_threshold);
  }

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> calculate_H,
    int max_iterations = 5, double stop_threshold = 1e-4)
  {
    return update(
      z, R, h, calculate_H,
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
      max_iterations, stop_threshold);
  }

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> calculate_H,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
    int max_iterations = 5, double stop_threshold = 1e-4)
  {
    const Eigen::VectorXd x_prior = X;
    const Eigen::MatrixXd p_prior = P;

    Eigen::VectorXd x_cur = x_prior;
    Eigen::MatrixXd H;
    Eigen::MatrixXd K;
    Eigen::MatrixXd S;

    const int iterations = std::max(1, max_iterations);
    for (int i = 0; i < iterations; ++i) {
      H = calculate_H(x_cur);
      S = H * p_prior * H.transpose() + R;
      S = (S + S.transpose()) * 0.5;
      S.diagonal().array() += 1e-12;

      Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
      if (ldlt.info() != Eigen::Success) {
        break;
      }

      K = p_prior * H.transpose() * ldlt.solve(
        Eigen::MatrixXd::Identity(z.rows(), z.rows()));

      const Eigen::VectorXd innovation = z_subtract(z, h(x_cur));
      const Eigen::VectorXd dx_prior = x_subtract_(x_prior, x_cur);
      const Eigen::VectorXd dx = K * (innovation - H * dx_prior);
      const Eigen::VectorXd x_next = x_add_(x_prior, dx);

      const Eigen::VectorXd diff = x_subtract_(x_next, x_cur);
      x_cur = x_next;
      if (diff.norm() < stop_threshold) {
        break;
      }
    }

    X = x_cur;
    if (H.size() != 0 && K.size() != 0) {
      const Eigen::MatrixXd IKH = I - K * H;
      P = IKH * p_prior * IKH.transpose() + K * R * K.transpose();
      P = (P + P.transpose()) * 0.5;
    }

    updateDiagnostics(z_subtract(z, h(X)), H, R, x_prior);
    return X;
  }

  std::map<std::string, double> data;
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 100;
  double last_nis = 0.0;

private:
  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_ =
    [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; };
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_subtract_ =
    [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; };

  int nis_count_ = 0;
  int total_count_ = 0;

  void updateDiagnostics(
    const Eigen::VectorXd & residual, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    const Eigen::VectorXd & x_prior)
  {
    if (H.size() == 0) {
      return;
    }

    Eigen::MatrixXd S = H * P * H.transpose() + R;
    S = (S + S.transpose()) * 0.5;
    S.diagonal().array() += 1e-12;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() == Eigen::Success) {
      last_nis = residual.transpose() * ldlt.solve(residual);
      data["nis"] = last_nis;
    }

    if (residual.size() >= 4) {
      data["residual_yaw"] = residual[0];
      data["residual_pitch"] = residual[1];
      data["residual_distance"] = residual[2];
      data["residual_angle"] = residual[3];
    }

    constexpr double nis_threshold = 9.49;
    const bool nis_failed = last_nis > nis_threshold;
    if (nis_failed) {
      ++nis_count_;
      data["nis_fail"] = 1.0;
    } else {
      data["nis_fail"] = 0.0;
    }
    ++total_count_;

    recent_nis_failures.push_back(nis_failed ? 1 : 0);
    if (recent_nis_failures.size() > window_size) {
      recent_nis_failures.pop_front();
    }

    const int recent_failures =
      std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    data["recent_nis_failures"] =
      static_cast<double>(recent_failures) / static_cast<double>(recent_nis_failures.size());

    if (P.rows() == x_prior.rows()) {
      Eigen::LDLT<Eigen::MatrixXd> p_ldlt(P);
      if (p_ldlt.info() == Eigen::Success) {
        const Eigen::VectorXd dx = x_subtract_(X, x_prior);
        data["nees"] = dx.transpose() * p_ldlt.solve(dx);
      }
    }
  }
};

}  // namespace rm_tools

#endif  // RM_TOOLS__ITERATED_EXTENDED_KALMAN_FILTER_HPP_
