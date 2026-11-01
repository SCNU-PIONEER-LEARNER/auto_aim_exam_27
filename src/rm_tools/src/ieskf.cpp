#include "rm_tools/ieskf.hpp"

#include <stdexcept>

namespace rm_tools
{

Iterative_Error_State_KF::Iterative_Error_State_KF(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, int max_iterations,
    double convergence_epsilon,
    StateDifference x_add,
    StateDifference x_subtract)
: X(x0),
    P(P0),
    max_iterations(max_iterations),
    convergence_epsilon(convergence_epsilon),
    I(Eigen::MatrixXd::Identity(P0.rows(), P0.rows())),
    x_add(x_add),
    x_subtract(x_subtract)
{
    if (P0.rows() != P0.cols()) {
        throw std::invalid_argument("Iterative_Error_State_KF P0 must be square");
    }
    if (x0.rows() == 0) {
        throw std::invalid_argument("Iterative_Error_State_KF x0 must not be empty");
    }
    if (x_subtract(x0, x0).rows() != P0.rows()) {
        throw std::invalid_argument("Iterative_Error_State_KF x_subtract must return an error-state vector");
    }
    if (max_iterations <= 0) {
        throw std::invalid_argument("Iterative_Error_State_KF max_iterations must be positive");
    }
    if (convergence_epsilon < 0.0) {
        throw std::invalid_argument("Iterative_Error_State_KF convergence_epsilon must be non-negative");
    }
}

Eigen::VectorXd Iterative_Error_State_KF::predict(
    StateFunction f, const Eigen::MatrixXd & Fx, const Eigen::MatrixXd & Q)
{
    check_predict_dimensions(Fx, Q);

    X = f(X);
    P = Fx * P * Fx.transpose() + Q;
    return X;
}

Eigen::VectorXd Iterative_Error_State_KF::predict(
    StateFunction f, const Eigen::MatrixXd & Fx, const Eigen::MatrixXd & Fi,
    const Eigen::MatrixXd & Qi)
{
    check_initialized();
    if (Fx.rows() != P.rows() || Fx.cols() != P.cols()) {
        throw std::invalid_argument("Iterative_Error_State_KF Fx dimension must match error-state covariance");
    }
    check_noise_dimensions(Fi, Qi);

    X = f(X);
    P = Fx * P * Fx.transpose() + Fi * Qi * Fi.transpose();
    return X;
}

Eigen::VectorXd Iterative_Error_State_KF::update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    StateDifference z_subtract)
{
    if (X.rows() != P.rows()) {
        throw std::invalid_argument(
            "Iterative_Error_State_KF linear update requires state dimension to equal error-state dimension");
    }
    return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd Iterative_Error_State_KF::update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    MeasurementFunction h,
    StateDifference z_subtract)
{
    return update(z, R, h, [&](const Eigen::VectorXd &) { return H; }, z_subtract);
}

Eigen::VectorXd Iterative_Error_State_KF::update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & R,
    MeasurementFunction h,
    MeasurementJacobian jacobian,
    StateDifference z_subtract,
    ResetJacobian reset_jacobian)
{
    check_initialized();

    const Eigen::VectorXd x_prior = X;
    const Eigen::MatrixXd p_prior = P;

    Eigen::VectorXd x_iter = X;
    Eigen::MatrixXd H;
    Eigen::MatrixXd K;

    for (int i = 0; i < max_iterations; ++i) {
        H = jacobian(x_iter);
        check_update_dimensions(z, H, R);

        const Eigen::VectorXd residual = z_subtract(z, h(x_iter));
        if (residual.rows() != z.rows()) {
            throw std::invalid_argument("Iterative_Error_State_KF residual dimension must match measurement dimension");
        }

        const Eigen::MatrixXd S = H * p_prior * H.transpose() + R;
        K = (S.ldlt().solve(H * p_prior)).transpose();

        const Eigen::VectorXd dx_iter = x_subtract(x_iter, x_prior);
        if (dx_iter.rows() != P.rows()) {
            throw std::invalid_argument("Iterative_Error_State_KF x_subtract must return an error-state vector");
        }

        const Eigen::VectorXd dx = K * residual + K * H * dx_iter;
        const Eigen::VectorXd x_next = x_add(x_prior, dx);
        const Eigen::VectorXd step = x_subtract(x_next, x_iter);

        x_iter = x_next;
        if (step.norm() <= convergence_epsilon) {
            break;
        }
    }

    H = jacobian(x_iter);
    check_update_dimensions(z, H, R);

    const Eigen::MatrixXd S = H * p_prior * H.transpose() + R;
    K = (S.ldlt().solve(H * p_prior)).transpose();

    const Eigen::MatrixXd p_update =
        (I - K * H) * p_prior * (I - K * H).transpose() + K * R * K.transpose();
    const Eigen::VectorXd dx_update = x_subtract(x_iter, x_prior);
    if (dx_update.rows() != P.rows()) {
        throw std::invalid_argument("Iterative_Error_State_KF x_subtract must return an error-state vector");
    }
    const Eigen::MatrixXd G = reset_jacobian ? reset_jacobian(dx_update) : I;
    if (G.rows() != P.rows() || G.cols() != P.cols()) {
        throw std::invalid_argument("Iterative_Error_State_KF reset_jacobian dimension must match error-state covariance");
    }

    X = x_iter;
    P = G * p_update * G.transpose();
    return X;
}

void Iterative_Error_State_KF::check_initialized() const
{
    if (X.rows() == 0 || P.rows() == 0) {
        throw std::runtime_error("Iterative_Error_State_KF filter is not initialized");
    }
}

void Iterative_Error_State_KF::check_predict_dimensions(const Eigen::MatrixXd & Fx, const Eigen::MatrixXd & Q) const
{
    check_initialized();
    if (Fx.rows() != P.rows() || Fx.cols() != P.cols()) {
        throw std::invalid_argument("Iterative_Error_State_KF Fx dimension must match error-state covariance");
    }
    if (Q.rows() != P.rows() || Q.cols() != P.cols()) {
        throw std::invalid_argument("Iterative_Error_State_KF Q dimension must match error-state covariance");
    }
}

void Iterative_Error_State_KF::check_noise_dimensions(
    const Eigen::MatrixXd & Fi, const Eigen::MatrixXd & Qi) const
{
    if (Fi.rows() != P.rows()) {
        throw std::invalid_argument("Iterative_Error_State_KF Fi row count must match error-state dimension");
    }
    if (Qi.rows() != Fi.cols() || Qi.cols() != Fi.cols()) {
        throw std::invalid_argument("Iterative_Error_State_KF Qi dimension must match process-noise dimension");
    }
}

void Iterative_Error_State_KF::check_update_dimensions(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R) const
{
    if (H.rows() != z.rows()) {
        throw std::invalid_argument("Iterative_Error_State_KF H row count must match measurement dimension");
    }
    if (H.cols() != P.rows()) {
        throw std::invalid_argument("Iterative_Error_State_KF H column count must match error-state dimension");
    }
    if (R.rows() != z.rows() || R.cols() != z.rows()) {
        throw std::invalid_argument("Iterative_Error_State_KF R dimension must match measurement dimension");
    }
}

}  // namespace rm_tools
