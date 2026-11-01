#ifndef RM_TOOLS__Iterative_Error_State_KF_HPP_
#define RM_TOOLS__Iterative_Error_State_KF_HPP_

#include <Eigen/Dense>

#include <functional>

namespace rm_tools
{

class Iterative_Error_State_KF
{
public:
    using StateFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using StateDifference =
        std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)>;
    using MeasurementFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using MeasurementJacobian = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
    using ResetJacobian = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

    Eigen::VectorXd X;
    Eigen::MatrixXd P;

    int max_iterations = 5;
    double convergence_epsilon = 1e-6;

    Iterative_Error_State_KF() = default;

    Iterative_Error_State_KF(
        const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, int max_iterations = 5,
        double convergence_epsilon = 1e-6,
        StateDifference x_add =
            [](const Eigen::VectorXd & x, const Eigen::VectorXd & dx) { return x + dx; },
        StateDifference x_subtract =
            [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

    Eigen::VectorXd predict(
        StateFunction f, const Eigen::MatrixXd & Fx, const Eigen::MatrixXd & Q);

    Eigen::VectorXd predict(
        StateFunction f, const Eigen::MatrixXd & Fx, const Eigen::MatrixXd & Fi,
        const Eigen::MatrixXd & Qi);

    Eigen::VectorXd update(
        const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
        StateDifference z_subtract =
            [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

    Eigen::VectorXd update(
        const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
        MeasurementFunction h,
        StateDifference z_subtract =
            [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

    Eigen::VectorXd update(
        const Eigen::VectorXd & z, const Eigen::MatrixXd & R,
        MeasurementFunction h,
        MeasurementJacobian jacobian,
        StateDifference z_subtract =
            [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
        ResetJacobian reset_jacobian = nullptr);

private:
    Eigen::MatrixXd I;
    StateDifference x_add;
    StateDifference x_subtract;

    void check_initialized() const;
    void check_predict_dimensions(
        const Eigen::MatrixXd & Fx, const Eigen::MatrixXd & Q) const;
    void check_noise_dimensions(
        const Eigen::MatrixXd & Fi, const Eigen::MatrixXd & Qi) const;
    void check_update_dimensions(
        const Eigen::VectorXd & z, const Eigen::MatrixXd & H,
        const Eigen::MatrixXd & R) const;
};

}  // namespace rm_tools

#endif  // RM_TOOLS__Iterative_Error_State_KF_HPP_
