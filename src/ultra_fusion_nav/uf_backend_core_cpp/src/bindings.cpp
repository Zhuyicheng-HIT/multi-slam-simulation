#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;

namespace {

constexpr int kStateSize = 15;

Matrix3d skew(const Vector3d &value) {
  Matrix3d matrix;
  matrix << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return matrix;
}

Matrix3d symmetric_pseudoinverse(const Matrix3d &matrix) {
  const Matrix3d symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("symmetric eigendecomposition failed");
  }
  const Vector3d values = solver.eigenvalues();
  const double threshold = std::max(1.0e-12, values.cwiseAbs().maxCoeff() * 1.0e-9);
  Vector3d inverse = Vector3d::Zero();
  for (int index = 0; index < 3; ++index) {
    if (std::abs(values(index)) > threshold) {
      inverse(index) = 1.0 / values(index);
    }
  }
  return solver.eigenvectors() * inverse.asDiagonal()
      * solver.eigenvectors().transpose();
}

void shape_conditional_translation_normal(
    Eigen::Matrix<double, 6, 6> &information,
    Eigen::Matrix<double, 6, 1> &gradient,
    const Matrix3d &information_transform) {
  const Matrix3d coupling = information.block<3, 3>(0, 3);
  const Matrix3d rotation = information.block<3, 3>(3, 3);
  const Matrix3d rotation_inverse = symmetric_pseudoinverse(rotation);
  const Matrix3d schur = 0.5 * (
      information.block<3, 3>(0, 0)
      - coupling * rotation_inverse * coupling.transpose()
      + (information.block<3, 3>(0, 0)
      - coupling * rotation_inverse * coupling.transpose()).transpose());
  const Vector3d conditional_gradient = gradient.head<3>()
      - coupling * rotation_inverse * gradient.tail<3>();
  Vector3d conditional_delta = symmetric_pseudoinverse(schur)
      * conditional_gradient;
  conditional_delta = conditional_delta.cwiseMax(-0.5).cwiseMin(0.5);
  const Matrix3d shaped_schur = information_transform * schur
      * information_transform;
  information.block<3, 3>(0, 0) = shaped_schur
      + coupling * rotation_inverse * coupling.transpose();
  information = 0.5 * (information + information.transpose());
  gradient.head<3>() = shaped_schur * conditional_delta
      + coupling * rotation_inverse * gradient.tail<3>();
}

Matrix3d rpy_to_rotation(const Vector3d &rpy) {
  const double cr = std::cos(rpy.x());
  const double sr = std::sin(rpy.x());
  const double cp = std::cos(rpy.y());
  const double sp = std::sin(rpy.y());
  const double cy = std::cos(rpy.z());
  const double sy = std::sin(rpy.z());
  Matrix3d rotation;
  rotation << cy * cp, cy * sp * sr - sy * cr,
      cy * sp * cr + sy * sr, sy * cp,
      sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp,
      cp * sr, cp * cr;
  return rotation;
}

Vector3d rotation_to_rpy(const Matrix3d &rotation) {
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  double roll = 0.0;
  double yaw = 0.0;
  if (std::abs(std::cos(pitch)) > 1.0e-7) {
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    roll = std::atan2(-rotation(1, 2), rotation(1, 1));
  }
  return Vector3d(roll, pitch, yaw);
}

Vector3d so3_log(const Matrix3d &rotation) {
  const double cosine = std::clamp(
      0.5 * (rotation.trace() - 1.0), -1.0, 1.0);
  const double angle = std::acos(cosine);
  Vector3d vector(
      rotation(2, 1) - rotation(1, 2),
      rotation(0, 2) - rotation(2, 0),
      rotation(1, 0) - rotation(0, 1));
  if (angle <= 1.0e-8) {
    return 0.5 * vector;
  }
  if (M_PI - angle <= 1.0e-5) {
    Eigen::SelfAdjointEigenSolver<Matrix3d> solver(
        0.5 * (rotation + Matrix3d::Identity()));
    Vector3d axis = solver.eigenvectors().col(2);
    if (axis.dot(vector) < 0.0) {
      axis = -axis;
    }
    return axis * angle;
  }
  return vector * (0.5 * angle / std::sin(angle));
}

Matrix3d so3_right_jacobian_inverse(const Vector3d &vector) {
  const double angle = vector.norm();
  const Matrix3d matrix = skew(vector);
  if (angle <= 1.0e-6) {
    return Matrix3d::Identity() + 0.5 * matrix
        + (1.0 / 12.0) * matrix * matrix;
  }
  const double coefficient =
      (1.0 - 0.5 * angle / std::tan(0.5 * angle)) / (angle * angle);
  return Matrix3d::Identity() + 0.5 * matrix
      + coefficient * matrix * matrix;
}

Matrix3d so3_left_jacobian_inverse(const Vector3d &vector) {
  const double angle = vector.norm();
  const Matrix3d matrix = skew(vector);
  if (angle <= 1.0e-6) {
    return Matrix3d::Identity() - 0.5 * matrix
        + (1.0 / 12.0) * matrix * matrix;
  }
  const double coefficient =
      (1.0 - 0.5 * angle / std::tan(0.5 * angle)) / (angle * angle);
  return Matrix3d::Identity() - 0.5 * matrix
      + coefficient * matrix * matrix;
}

Matrix3d so3_exp(const Vector3d &vector) {
  const double angle = vector.norm();
  const Matrix3d matrix = skew(vector);
  if (angle <= 1.0e-8) {
    return Matrix3d::Identity() + matrix + 0.5 * matrix * matrix;
  }
  return Matrix3d::Identity()
      + (std::sin(angle) / angle) * matrix
      + ((1.0 - std::cos(angle)) / (angle * angle)) * matrix * matrix;
}

MatrixXd state_plus_batch(
    const Eigen::Ref<const MatrixXd> &states,
    const Eigen::Ref<const MatrixXd> &increments) {
  if (states.cols() != kStateSize || increments.cols() != kStateSize
      || states.rows() != increments.rows() || states.rows() < 1
      || !states.allFinite() || !increments.allFinite()) {
    throw std::invalid_argument(
        "manifold states and increments must be finite Nx15 matrices");
  }
  MatrixXd updated = states + increments;
  for (Eigen::Index index = 0; index < states.rows(); ++index) {
    const Matrix3d rotation =
        rpy_to_rotation(states.row(index).segment<3>(3).transpose())
        * so3_exp(increments.row(index).segment<3>(3).transpose());
    updated.row(index).segment<3>(3) = rotation_to_rpy(rotation).transpose();
  }
  if (!updated.allFinite()) {
    throw std::invalid_argument("manifold state update produced non-finite values");
  }
  return updated;
}

Matrix3d so3_right_jacobian(const Vector3d &vector) {
  const double angle = vector.norm();
  const Matrix3d matrix = skew(vector);
  if (angle <= 1.0e-6) {
    return Matrix3d::Identity() - 0.5 * matrix
        + (1.0 / 6.0) * matrix * matrix;
  }
  return Matrix3d::Identity()
      - ((1.0 - std::cos(angle)) / (angle * angle)) * matrix
      + ((angle - std::sin(angle)) / (angle * angle * angle))
          * matrix * matrix;
}

struct ImuLinearization {
  Eigen::Matrix<double, 15, 1> residual;
  Eigen::Matrix<double, 15, 15> jacobian_i;
  Eigen::Matrix<double, 15, 15> jacobian_j;
};

ImuLinearization linearize_imu(
    const VectorXd &state_i,
    const VectorXd &state_j,
    const Vector3d &gravity,
    double dt_s,
    const Vector3d &delta_position,
    const Vector3d &delta_velocity,
    const Eigen::Vector4d &delta_quaternion,
    const Vector3d &accel_bias_linearization,
    const Vector3d &gyro_bias_linearization,
    const Matrix3d &position_accel,
    const Matrix3d &position_gyro,
    const Matrix3d &velocity_accel,
    const Matrix3d &velocity_gyro,
    const Matrix3d &rotation_gyro) {
  if (state_i.size() != kStateSize || state_j.size() != kStateSize
      || !state_i.allFinite() || !state_j.allFinite()
      || !gravity.allFinite() || !std::isfinite(dt_s) || dt_s <= 0.0
      || !delta_position.allFinite() || !delta_velocity.allFinite()
      || !delta_quaternion.allFinite()
      || !accel_bias_linearization.allFinite()
      || !gyro_bias_linearization.allFinite()
      || !position_accel.allFinite() || !position_gyro.allFinite()
      || !velocity_accel.allFinite() || !velocity_gyro.allFinite()
      || !rotation_gyro.allFinite()) {
    throw std::invalid_argument("IMU preintegration inputs are invalid");
  }
  Eigen::Quaterniond delta_rotation_quaternion(
      delta_quaternion(0), delta_quaternion(1),
      delta_quaternion(2), delta_quaternion(3));
  if (delta_rotation_quaternion.norm() <= 1.0e-12) {
    throw std::invalid_argument("IMU delta quaternion has zero norm");
  }
  delta_rotation_quaternion.normalize();

  const Matrix3d rotation_i = rpy_to_rotation(state_i.segment<3>(3));
  const Matrix3d rotation_j = rpy_to_rotation(state_j.segment<3>(3));
  const Matrix3d rotation_i_transpose = rotation_i.transpose();
  const Vector3d accel_delta =
      state_i.segment<3>(9) - accel_bias_linearization;
  const Vector3d gyro_delta =
      state_i.segment<3>(12) - gyro_bias_linearization;
  const Vector3d corrected_position = delta_position
      + position_accel * accel_delta + position_gyro * gyro_delta;
  const Vector3d corrected_velocity = delta_velocity
      + velocity_accel * accel_delta + velocity_gyro * gyro_delta;
  const Vector3d rotation_bias_vector = rotation_gyro * gyro_delta;
  const Matrix3d corrected_rotation =
      delta_rotation_quaternion.toRotationMatrix()
      * so3_exp(rotation_bias_vector);
  const Vector3d position_delta_world = state_j.head<3>() - state_i.head<3>()
      - state_i.segment<3>(6) * dt_s - 0.5 * gravity * dt_s * dt_s;
  const Vector3d velocity_delta_world = state_j.segment<3>(6)
      - state_i.segment<3>(6) - gravity * dt_s;
  const Vector3d position_delta_body =
      rotation_i_transpose * position_delta_world;
  const Vector3d velocity_delta_body =
      rotation_i_transpose * velocity_delta_world;
  const Vector3d rotation_residual = so3_log(
      corrected_rotation.transpose() * rotation_i_transpose * rotation_j);

  ImuLinearization result;
  result.residual << position_delta_body - corrected_position,
      velocity_delta_body - corrected_velocity, rotation_residual,
      state_j.segment<3>(9) - state_i.segment<3>(9),
      state_j.segment<3>(12) - state_i.segment<3>(12);
  result.jacobian_i.setZero();
  result.jacobian_j.setZero();
  result.jacobian_i.block<3, 3>(0, 0) = -rotation_i_transpose;
  result.jacobian_i.block<3, 3>(0, 3) = skew(position_delta_body);
  result.jacobian_i.block<3, 3>(0, 6) = -rotation_i_transpose * dt_s;
  result.jacobian_i.block<3, 3>(0, 9) = -position_accel;
  result.jacobian_i.block<3, 3>(0, 12) = -position_gyro;
  result.jacobian_j.block<3, 3>(0, 0) = rotation_i_transpose;

  result.jacobian_i.block<3, 3>(3, 3) = skew(velocity_delta_body);
  result.jacobian_i.block<3, 3>(3, 6) = -rotation_i_transpose;
  result.jacobian_i.block<3, 3>(3, 9) = -velocity_accel;
  result.jacobian_i.block<3, 3>(3, 12) = -velocity_gyro;
  result.jacobian_j.block<3, 3>(3, 6) = rotation_i_transpose;

  const Matrix3d left_inverse =
      so3_left_jacobian_inverse(rotation_residual);
  const Matrix3d right_inverse =
      so3_right_jacobian_inverse(rotation_residual);
  result.jacobian_i.block<3, 3>(6, 3) =
      -left_inverse * corrected_rotation.transpose();
  result.jacobian_i.block<3, 3>(6, 12) =
      -left_inverse * so3_right_jacobian(rotation_bias_vector) * rotation_gyro;
  result.jacobian_j.block<3, 3>(6, 3) = right_inverse;

  result.jacobian_i.block<3, 3>(9, 9) = -Matrix3d::Identity();
  result.jacobian_j.block<3, 3>(9, 9) = Matrix3d::Identity();
  result.jacobian_i.block<3, 3>(12, 12) = -Matrix3d::Identity();
  result.jacobian_j.block<3, 3>(12, 12) = Matrix3d::Identity();
  return result;
}

std::tuple<MatrixXd, VectorXd, double> imu_preintegrated_normal(
    const Eigen::Ref<const VectorXd> &state_i,
    const Eigen::Ref<const VectorXd> &state_j,
    const Eigen::Ref<const Vector3d> &gravity,
    double dt_s,
    const Eigen::Ref<const Vector3d> &delta_position,
    const Eigen::Ref<const Vector3d> &delta_velocity,
    const Eigen::Ref<const Eigen::Vector4d> &delta_quaternion,
    const Eigen::Ref<const Vector3d> &accel_bias_linearization,
    const Eigen::Ref<const Vector3d> &gyro_bias_linearization,
    const Eigen::Ref<const Matrix3d> &position_accel,
    const Eigen::Ref<const Matrix3d> &position_gyro,
    const Eigen::Ref<const Matrix3d> &velocity_accel,
    const Eigen::Ref<const Matrix3d> &velocity_gyro,
    const Eigen::Ref<const Matrix3d> &rotation_gyro,
    const Eigen::Ref<const MatrixXd> &information_matrix,
    double effective_weight) {
  if (information_matrix.rows() != kStateSize
      || information_matrix.cols() != kStateSize
      || !information_matrix.allFinite() || !std::isfinite(effective_weight)
      || effective_weight < 0.0) {
    throw std::invalid_argument("IMU information matrix is invalid");
  }
  const ImuLinearization value = linearize_imu(
      state_i, state_j, gravity, dt_s, delta_position, delta_velocity,
      delta_quaternion, accel_bias_linearization, gyro_bias_linearization,
      position_accel, position_gyro, velocity_accel, velocity_gyro,
      rotation_gyro);
  const MatrixXd information = effective_weight * information_matrix;
  const VectorXd weighted_residual = information * value.residual;
  MatrixXd hessian = MatrixXd::Zero(2 * kStateSize, 2 * kStateSize);
  VectorXd gradient = VectorXd::Zero(2 * kStateSize);
  hessian.block(0, 0, kStateSize, kStateSize).noalias() =
      value.jacobian_i.transpose() * information * value.jacobian_i;
  hessian.block(0, kStateSize, kStateSize, kStateSize).noalias() =
      value.jacobian_i.transpose() * information * value.jacobian_j;
  hessian.block(kStateSize, 0, kStateSize, kStateSize).noalias() =
      value.jacobian_j.transpose() * information * value.jacobian_i;
  hessian.block(kStateSize, kStateSize, kStateSize, kStateSize).noalias() =
      value.jacobian_j.transpose() * information * value.jacobian_j;
  gradient.head(kStateSize).noalias() =
      value.jacobian_i.transpose() * weighted_residual;
  gradient.tail(kStateSize).noalias() =
      value.jacobian_j.transpose() * weighted_residual;
  const double cost = 0.5 * value.residual.dot(weighted_residual);
  return {hessian, gradient, cost};
}

std::tuple<MatrixXd, VectorXd, double> imu_preintegrated_graph_normal(
    const Eigen::Ref<const MatrixXd> &states,
    const Eigen::Ref<const Eigen::MatrixXi> &state_indices,
    const Eigen::Ref<const Vector3d> &gravity,
    const Eigen::Ref<const VectorXd> &dt_s,
    const Eigen::Ref<const MatrixXd> &delta_position,
    const Eigen::Ref<const MatrixXd> &delta_velocity,
    const Eigen::Ref<const MatrixXd> &delta_quaternion,
    const Eigen::Ref<const MatrixXd> &accel_bias_linearization,
    const Eigen::Ref<const MatrixXd> &gyro_bias_linearization,
    const Eigen::Ref<const MatrixXd> &position_accel,
    const Eigen::Ref<const MatrixXd> &position_gyro,
    const Eigen::Ref<const MatrixXd> &velocity_accel,
    const Eigen::Ref<const MatrixXd> &velocity_gyro,
    const Eigen::Ref<const MatrixXd> &rotation_gyro,
    const Eigen::Ref<const MatrixXd> &information_matrices,
    const Eigen::Ref<const VectorXd> &effective_weights) {
  const Eigen::Index factor_count = state_indices.rows();
  if (states.rows() < 2 || states.cols() != kStateSize
      || state_indices.cols() != 2 || factor_count < 1
      || dt_s.size() != factor_count
      || delta_position.rows() != factor_count
      || delta_position.cols() != 3
      || delta_velocity.rows() != factor_count
      || delta_velocity.cols() != 3
      || delta_quaternion.rows() != factor_count
      || delta_quaternion.cols() != 4
      || accel_bias_linearization.rows() != factor_count
      || accel_bias_linearization.cols() != 3
      || gyro_bias_linearization.rows() != factor_count
      || gyro_bias_linearization.cols() != 3
      || position_accel.rows() != factor_count
      || position_accel.cols() != 9
      || position_gyro.rows() != factor_count
      || position_gyro.cols() != 9
      || velocity_accel.rows() != factor_count
      || velocity_accel.cols() != 9
      || velocity_gyro.rows() != factor_count
      || velocity_gyro.cols() != 9
      || rotation_gyro.rows() != factor_count
      || rotation_gyro.cols() != 9
      || information_matrices.rows() != factor_count
      || information_matrices.cols() != kStateSize * kStateSize
      || effective_weights.size() != factor_count
      || !states.allFinite() || !gravity.allFinite()
      || !dt_s.allFinite() || !delta_position.allFinite()
      || !delta_velocity.allFinite() || !delta_quaternion.allFinite()
      || !accel_bias_linearization.allFinite()
      || !gyro_bias_linearization.allFinite()
      || !position_accel.allFinite() || !position_gyro.allFinite()
      || !velocity_accel.allFinite() || !velocity_gyro.allFinite()
      || !rotation_gyro.allFinite() || !information_matrices.allFinite()
      || !effective_weights.allFinite()
      || (effective_weights.array() < 0.0).any()) {
    throw std::invalid_argument(
        "batched IMU normal-equation inputs are invalid");
  }

  const Eigen::Index dimension = states.rows() * kStateSize;
  MatrixXd hessian = MatrixXd::Zero(dimension, dimension);
  VectorXd gradient = VectorXd::Zero(dimension);
  double cost = 0.0;
  for (Eigen::Index factor = 0; factor < factor_count; ++factor) {
    const int previous = state_indices(factor, 0);
    const int current = state_indices(factor, 1);
    if (previous < 0 || current < 0 || previous >= states.rows()
        || current >= states.rows() || previous == current) {
      throw std::invalid_argument(
          "batched IMU factor has an invalid state index");
    }
    const VectorXd state_i = states.row(previous).transpose();
    const VectorXd state_j = states.row(current).transpose();
    Matrix3d factor_position_accel;
    Matrix3d factor_position_gyro;
    Matrix3d factor_velocity_accel;
    Matrix3d factor_velocity_gyro;
    Matrix3d factor_rotation_gyro;
    Eigen::Matrix<double, kStateSize, kStateSize> factor_information;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        const int flat = 3 * row + column;
        factor_position_accel(row, column) = position_accel(factor, flat);
        factor_position_gyro(row, column) = position_gyro(factor, flat);
        factor_velocity_accel(row, column) = velocity_accel(factor, flat);
        factor_velocity_gyro(row, column) = velocity_gyro(factor, flat);
        factor_rotation_gyro(row, column) = rotation_gyro(factor, flat);
      }
    }
    for (int row = 0; row < kStateSize; ++row) {
      for (int column = 0; column < kStateSize; ++column) {
        factor_information(row, column) =
            information_matrices(factor, kStateSize * row + column);
      }
    }
    const ImuLinearization value = linearize_imu(
        state_i, state_j, gravity, dt_s(factor),
        delta_position.row(factor).transpose(),
        delta_velocity.row(factor).transpose(),
        delta_quaternion.row(factor).transpose(),
        accel_bias_linearization.row(factor).transpose(),
        gyro_bias_linearization.row(factor).transpose(),
        factor_position_accel, factor_position_gyro,
        factor_velocity_accel, factor_velocity_gyro,
        factor_rotation_gyro);
    const MatrixXd information =
        effective_weights(factor) * factor_information;
    const VectorXd weighted_residual = information * value.residual;
    const Eigen::Index previous_offset = previous * kStateSize;
    const Eigen::Index current_offset = current * kStateSize;
    hessian.block(previous_offset, previous_offset, kStateSize, kStateSize)
        .noalias() += value.jacobian_i.transpose()
            * information * value.jacobian_i;
    hessian.block(previous_offset, current_offset, kStateSize, kStateSize)
        .noalias() += value.jacobian_i.transpose()
            * information * value.jacobian_j;
    hessian.block(current_offset, previous_offset, kStateSize, kStateSize)
        .noalias() += value.jacobian_j.transpose()
            * information * value.jacobian_i;
    hessian.block(current_offset, current_offset, kStateSize, kStateSize)
        .noalias() += value.jacobian_j.transpose()
            * information * value.jacobian_j;
    gradient.segment(previous_offset, kStateSize).noalias() +=
        value.jacobian_i.transpose() * weighted_residual;
    gradient.segment(current_offset, kStateSize).noalias() +=
        value.jacobian_j.transpose() * weighted_residual;
    cost += 0.5 * value.residual.dot(weighted_residual);
  }
  return {hessian, gradient, cost};
}

double imu_preintegrated_cost(
    const Eigen::Ref<const VectorXd> &state_i,
    const Eigen::Ref<const VectorXd> &state_j,
    const Eigen::Ref<const Vector3d> &gravity,
    double dt_s,
    const Eigen::Ref<const Vector3d> &delta_position,
    const Eigen::Ref<const Vector3d> &delta_velocity,
    const Eigen::Ref<const Eigen::Vector4d> &delta_quaternion,
    const Eigen::Ref<const Vector3d> &accel_bias_linearization,
    const Eigen::Ref<const Vector3d> &gyro_bias_linearization,
    const Eigen::Ref<const Matrix3d> &position_accel,
    const Eigen::Ref<const Matrix3d> &position_gyro,
    const Eigen::Ref<const Matrix3d> &velocity_accel,
    const Eigen::Ref<const Matrix3d> &velocity_gyro,
    const Eigen::Ref<const Matrix3d> &rotation_gyro,
    const Eigen::Ref<const MatrixXd> &information_matrix,
    double effective_weight) {
  if (information_matrix.rows() != kStateSize
      || information_matrix.cols() != kStateSize
      || !information_matrix.allFinite() || !std::isfinite(effective_weight)
      || effective_weight < 0.0) {
    throw std::invalid_argument("IMU information matrix is invalid");
  }
  const ImuLinearization value = linearize_imu(
      state_i, state_j, gravity, dt_s, delta_position, delta_velocity,
      delta_quaternion, accel_bias_linearization, gyro_bias_linearization,
      position_accel, position_gyro, velocity_accel, velocity_gyro,
      rotation_gyro);
  return 0.5 * effective_weight
      * value.residual.dot(information_matrix * value.residual);
}

void require_finite(const MatrixXd &value, const char *name) {
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void validate_lidar_inputs(
    const VectorXd &pose,
    const MatrixXd &lidar_points,
    const MatrixXd &plane_normals,
    const MatrixXd &plane_points,
    const Matrix3d &lidar_to_body_rotation,
    const Vector3d &lidar_to_body_translation,
    const VectorXd &variance,
    double effective_weight,
    double huber_delta) {
  const Eigen::Index count = lidar_points.rows();
  if (pose.size() != 6 || lidar_points.cols() != 3 || count <= 0
      || plane_normals.rows() != count || plane_normals.cols() != 3
      || plane_points.rows() != count || plane_points.cols() != 3
      || variance.size() != count) {
    throw std::invalid_argument("LiDAR normal inputs have incompatible shapes");
  }
  require_finite(pose, "pose");
  require_finite(lidar_points, "lidar points");
  require_finite(plane_normals, "plane normals");
  require_finite(plane_points, "plane points");
  if (!lidar_to_body_rotation.allFinite()
      || !lidar_to_body_translation.allFinite()
      || !variance.allFinite() || (variance.array() <= 0.0).any()
      || !std::isfinite(effective_weight) || effective_weight < 0.0
      || !std::isfinite(huber_delta) || huber_delta < 0.0) {
    throw std::invalid_argument("LiDAR normal inputs are not physically valid");
  }
}

std::tuple<MatrixXd, VectorXd, double> lidar_point_plane_normal_impl(
    const Eigen::Ref<const VectorXd> &pose,
    const Eigen::Ref<const MatrixXd> &lidar_points,
    const Eigen::Ref<const MatrixXd> &plane_normals,
    const Eigen::Ref<const MatrixXd> &plane_points,
    const Eigen::Ref<const Matrix3d> &lidar_to_body_rotation,
    const Eigen::Ref<const Vector3d> &lidar_to_body_translation,
    const Vector3d &translation_jacobian_scale,
    const Eigen::Ref<const VectorXd> &variance,
    double effective_weight,
    double huber_delta) {
  validate_lidar_inputs(
      pose, lidar_points, plane_normals, plane_points,
      lidar_to_body_rotation, lidar_to_body_translation, variance,
      effective_weight, huber_delta);
  const Matrix3d rotation = rpy_to_rotation(pose.segment<3>(3));
  MatrixXd hessian = MatrixXd::Zero(6, 6);
  VectorXd gradient = VectorXd::Zero(6);
  double cost = 0.0;
  for (Eigen::Index index = 0; index < lidar_points.rows(); ++index) {
    const Vector3d body_point =
        lidar_to_body_rotation * lidar_points.row(index).transpose()
        + lidar_to_body_translation;
    const Vector3d normal = plane_normals.row(index).transpose();
    const Vector3d world_point = rotation * body_point + pose.head<3>();
    const double residual = normal.dot(
        world_point - plane_points.row(index).transpose());
    const double standardized = residual / std::sqrt(variance(index));
    const double absolute = std::abs(standardized);
    double loss = 0.5 * standardized * standardized;
    double robust_weight = 1.0;
    if (huber_delta > 0.0 && absolute > huber_delta) {
      loss = huber_delta * (absolute - 0.5 * huber_delta);
      robust_weight = huber_delta / absolute;
    }
    const double information =
        effective_weight * robust_weight / variance(index);
    Eigen::Matrix<double, 6, 1> jacobian;
    jacobian.head<3>() = normal.cwiseProduct(translation_jacobian_scale);
    jacobian.tail<3>() = body_point.cross(rotation.transpose() * normal);
    hessian.noalias() += information * jacobian * jacobian.transpose();
    gradient.noalias() += information * residual * jacobian;
    cost += effective_weight * loss;
  }
  return {hessian, gradient, cost};
}

std::tuple<MatrixXd, VectorXd, double> lidar_point_plane_normal(
    const Eigen::Ref<const VectorXd> &pose,
    const Eigen::Ref<const MatrixXd> &lidar_points,
    const Eigen::Ref<const MatrixXd> &plane_normals,
    const Eigen::Ref<const MatrixXd> &plane_points,
    const Eigen::Ref<const Matrix3d> &lidar_to_body_rotation,
    const Eigen::Ref<const Vector3d> &lidar_to_body_translation,
    const Eigen::Ref<const VectorXd> &variance,
    double effective_weight,
    double huber_delta) {
  return lidar_point_plane_normal_impl(
      pose, lidar_points, plane_normals, plane_points,
      lidar_to_body_rotation, lidar_to_body_translation, Vector3d::Ones(),
      variance, effective_weight, huber_delta);
}

std::tuple<MatrixXd, VectorXd, double> lidar_point_plane_normal_axis_scaled(
    const Eigen::Ref<const VectorXd> &pose,
    const Eigen::Ref<const MatrixXd> &lidar_points,
    const Eigen::Ref<const MatrixXd> &plane_normals,
    const Eigen::Ref<const MatrixXd> &plane_points,
    const Eigen::Ref<const Matrix3d> &lidar_to_body_rotation,
    const Eigen::Ref<const Vector3d> &lidar_to_body_translation,
    const Eigen::Ref<const Vector3d> &translation_information_scale,
    const Eigen::Ref<const VectorXd> &variance,
    double effective_weight,
    double huber_delta) {
  if (!translation_information_scale.allFinite()
      || (translation_information_scale.array() < 0.0).any()
      || (translation_information_scale.array() > 1.0).any()) {
    throw std::invalid_argument(
        "LiDAR translation information scale must be within [0, 1]");
  }
  const Vector3d translation_jacobian_scale =
      translation_information_scale.array().sqrt().matrix();
  return lidar_point_plane_normal_impl(
      pose, lidar_points, plane_normals, plane_points,
      lidar_to_body_rotation, lidar_to_body_translation,
      translation_jacobian_scale, variance, effective_weight, huber_delta);
}

double lidar_point_plane_cost(
    const Eigen::Ref<const VectorXd> &pose,
    const Eigen::Ref<const MatrixXd> &lidar_points,
    const Eigen::Ref<const MatrixXd> &plane_normals,
    const Eigen::Ref<const MatrixXd> &plane_points,
    const Eigen::Ref<const Matrix3d> &lidar_to_body_rotation,
    const Eigen::Ref<const Vector3d> &lidar_to_body_translation,
    const Eigen::Ref<const VectorXd> &variance,
    double effective_weight,
    double huber_delta) {
  validate_lidar_inputs(
      pose, lidar_points, plane_normals, plane_points,
      lidar_to_body_rotation, lidar_to_body_translation, variance,
      effective_weight, huber_delta);
  const Matrix3d rotation = rpy_to_rotation(pose.segment<3>(3));
  double cost = 0.0;
  for (Eigen::Index index = 0; index < lidar_points.rows(); ++index) {
    const Vector3d body_point =
        lidar_to_body_rotation * lidar_points.row(index).transpose()
        + lidar_to_body_translation;
    const Vector3d world_point = rotation * body_point + pose.head<3>();
    const double residual = plane_normals.row(index).dot(
        world_point - plane_points.row(index).transpose());
    const double standardized = residual / std::sqrt(variance(index));
    const double absolute = std::abs(standardized);
    const double loss =
        huber_delta > 0.0 && absolute > huber_delta
        ? huber_delta * (absolute - 0.5 * huber_delta)
        : 0.5 * standardized * standardized;
    cost += effective_weight * loss;
  }
  return cost;
}

std::tuple<MatrixXd, VectorXd, double>
lidar_point_plane_graph_normal_axis_scaled(
    const Eigen::Ref<const MatrixXd> &states,
    const Eigen::Ref<const Eigen::VectorXi> &state_indices,
    const Eigen::Ref<const Eigen::VectorXi> &factor_offsets,
    const Eigen::Ref<const MatrixXd> &lidar_points,
    const Eigen::Ref<const MatrixXd> &plane_normals,
    const Eigen::Ref<const MatrixXd> &plane_points,
    const Eigen::Ref<const MatrixXd> &lidar_to_body_rotations,
    const Eigen::Ref<const MatrixXd> &lidar_to_body_translations,
    const Eigen::Ref<const MatrixXd> &translation_information_scales,
    const Eigen::Ref<const VectorXd> &variance,
    const Eigen::Ref<const VectorXd> &effective_weights,
    double huber_delta) {
  const Eigen::Index factor_count = state_indices.size();
  const Eigen::Index point_count = lidar_points.rows();
  if (states.rows() < 1 || states.cols() != kStateSize
      || factor_count < 1 || factor_offsets.size() != factor_count + 1
      || point_count < 1 || lidar_points.cols() != 3
      || plane_normals.rows() != point_count || plane_normals.cols() != 3
      || plane_points.rows() != point_count || plane_points.cols() != 3
      || lidar_to_body_rotations.rows() != factor_count
      || lidar_to_body_rotations.cols() != 9
      || lidar_to_body_translations.rows() != factor_count
      || lidar_to_body_translations.cols() != 3
      || translation_information_scales.rows() != factor_count
      || translation_information_scales.cols() != 3
      || variance.size() != point_count
      || effective_weights.size() != factor_count
      || factor_offsets(0) != 0
      || factor_offsets(factor_count) != point_count
      || !states.allFinite() || !lidar_points.allFinite()
      || !plane_normals.allFinite() || !plane_points.allFinite()
      || !lidar_to_body_rotations.allFinite()
      || !lidar_to_body_translations.allFinite()
      || !translation_information_scales.allFinite()
      || (translation_information_scales.array() < 0.0).any()
      || (translation_information_scales.array() > 1.0).any()
      || !variance.allFinite() || (variance.array() <= 0.0).any()
      || !effective_weights.allFinite()
      || (effective_weights.array() < 0.0).any()
      || !std::isfinite(huber_delta) || huber_delta < 0.0) {
    throw std::invalid_argument(
        "batched LiDAR normal-equation inputs are invalid");
  }
  for (Eigen::Index factor = 0; factor < factor_count; ++factor) {
    if (state_indices(factor) < 0 || state_indices(factor) >= states.rows()
        || factor_offsets(factor) < 0
        || factor_offsets(factor + 1) <= factor_offsets(factor)) {
      throw std::invalid_argument(
          "batched LiDAR factor has invalid state or point indices");
    }
  }

  const Eigen::Index dimension = states.rows() * kStateSize;
  MatrixXd hessian = MatrixXd::Zero(dimension, dimension);
  VectorXd gradient = VectorXd::Zero(dimension);
  double cost = 0.0;
  for (Eigen::Index factor = 0; factor < factor_count; ++factor) {
    const int state_index = state_indices(factor);
    Matrix3d lidar_to_body_rotation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        lidar_to_body_rotation(row, column) =
            lidar_to_body_rotations(factor, 3 * row + column);
      }
    }
    const Vector3d lidar_to_body_translation =
        lidar_to_body_translations.row(factor).transpose();
    const Vector3d translation_jacobian_scale =
        translation_information_scales.row(factor).array().sqrt().matrix();
    const VectorXd pose = states.row(state_index).head<6>().transpose();
    const Matrix3d rotation = rpy_to_rotation(pose.segment<3>(3));
    Eigen::Matrix<double, 6, 6> local_hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> local_gradient =
        Eigen::Matrix<double, 6, 1>::Zero();
    for (int point = factor_offsets(factor);
         point < factor_offsets(factor + 1); ++point) {
      const Vector3d body_point =
          lidar_to_body_rotation * lidar_points.row(point).transpose()
          + lidar_to_body_translation;
      const Vector3d normal = plane_normals.row(point).transpose();
      const Vector3d world_point = rotation * body_point + pose.head<3>();
      const double residual = normal.dot(
          world_point - plane_points.row(point).transpose());
      const double standardized = residual / std::sqrt(variance(point));
      const double absolute = std::abs(standardized);
      double loss = 0.5 * standardized * standardized;
      double robust_weight = 1.0;
      if (huber_delta > 0.0 && absolute > huber_delta) {
        loss = huber_delta * (absolute - 0.5 * huber_delta);
        robust_weight = huber_delta / absolute;
      }
      const double information = effective_weights(factor)
          * robust_weight / variance(point);
      Eigen::Matrix<double, 6, 1> jacobian;
      jacobian.head<3>() =
          normal.cwiseProduct(translation_jacobian_scale);
      jacobian.tail<3>() = body_point.cross(rotation.transpose() * normal);
      local_hessian.noalias() +=
          information * jacobian * jacobian.transpose();
      local_gradient.noalias() += information * residual * jacobian;
      cost += effective_weights(factor) * loss;
    }
    const Eigen::Index offset = state_index * kStateSize;
    hessian.block(offset, offset, 6, 6) += local_hessian;
    gradient.segment(offset, 6) += local_gradient;
  }
  return {hessian, gradient, cost};
}

std::tuple<MatrixXd, VectorXd, double> lidar_point_plane_graph_normal(
    const Eigen::Ref<const MatrixXd> &states,
    const Eigen::Ref<const Eigen::VectorXi> &state_indices,
    const Eigen::Ref<const Eigen::VectorXi> &factor_offsets,
    const Eigen::Ref<const MatrixXd> &lidar_points,
    const Eigen::Ref<const MatrixXd> &plane_normals,
    const Eigen::Ref<const MatrixXd> &plane_points,
    const Eigen::Ref<const MatrixXd> &lidar_to_body_rotations,
    const Eigen::Ref<const MatrixXd> &lidar_to_body_translations,
    const Eigen::Ref<const VectorXd> &variance,
    const Eigen::Ref<const VectorXd> &effective_weights,
    double huber_delta) {
  const MatrixXd translation_information_scales = MatrixXd::Ones(
      state_indices.size(), 3);
  return lidar_point_plane_graph_normal_axis_scaled(
      states, state_indices, factor_offsets, lidar_points, plane_normals,
      plane_points, lidar_to_body_rotations, lidar_to_body_translations,
      translation_information_scales, variance, effective_weights,
      huber_delta);
}

std::tuple<MatrixXd, VectorXd, double>
lidar_point_plane_graph_normal_subspace(
    const Eigen::Ref<const MatrixXd> &states,
    const Eigen::Ref<const Eigen::VectorXi> &state_indices,
    const Eigen::Ref<const Eigen::VectorXi> &factor_offsets,
    const Eigen::Ref<const MatrixXd> &lidar_points,
    const Eigen::Ref<const MatrixXd> &plane_normals,
    const Eigen::Ref<const MatrixXd> &plane_points,
    const Eigen::Ref<const MatrixXd> &lidar_to_body_rotations,
    const Eigen::Ref<const MatrixXd> &lidar_to_body_translations,
    const Eigen::Ref<const MatrixXd> &translation_information_transforms,
    const Eigen::Ref<const VectorXd> &variance,
    const Eigen::Ref<const VectorXd> &effective_weights,
    double huber_delta) {
  const Eigen::Index factor_count = state_indices.size();
  const Eigen::Index point_count = lidar_points.rows();
  if (states.rows() < 1 || states.cols() != kStateSize
      || factor_count < 1 || factor_offsets.size() != factor_count + 1
      || point_count < 1 || lidar_points.cols() != 3
      || plane_normals.rows() != point_count || plane_normals.cols() != 3
      || plane_points.rows() != point_count || plane_points.cols() != 3
      || lidar_to_body_rotations.rows() != factor_count
      || lidar_to_body_rotations.cols() != 9
      || lidar_to_body_translations.rows() != factor_count
      || lidar_to_body_translations.cols() != 3
      || translation_information_transforms.rows() != factor_count
      || translation_information_transforms.cols() != 9
      || variance.size() != point_count
      || effective_weights.size() != factor_count
      || factor_offsets(0) != 0
      || factor_offsets(factor_count) != point_count
      || !states.allFinite() || !lidar_points.allFinite()
      || !plane_normals.allFinite() || !plane_points.allFinite()
      || !lidar_to_body_rotations.allFinite()
      || !lidar_to_body_translations.allFinite()
      || !translation_information_transforms.allFinite()
      || !variance.allFinite() || (variance.array() <= 0.0).any()
      || !effective_weights.allFinite()
      || (effective_weights.array() < 0.0).any()
      || !std::isfinite(huber_delta) || huber_delta < 0.0) {
    throw std::invalid_argument(
        "batched LiDAR subspace inputs are invalid");
  }

  const Eigen::Index dimension = states.rows() * kStateSize;
  MatrixXd hessian = MatrixXd::Zero(dimension, dimension);
  VectorXd gradient = VectorXd::Zero(dimension);
  double cost = 0.0;
  for (Eigen::Index factor = 0; factor < factor_count; ++factor) {
    if (state_indices(factor) < 0 || state_indices(factor) >= states.rows()
        || factor_offsets(factor) < 0
        || factor_offsets(factor + 1) <= factor_offsets(factor)) {
      throw std::invalid_argument(
          "batched LiDAR subspace factor has invalid indices");
    }
    Matrix3d lidar_to_body_rotation;
    Matrix3d information_transform;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        lidar_to_body_rotation(row, column) =
            lidar_to_body_rotations(factor, 3 * row + column);
        information_transform(row, column) =
            translation_information_transforms(factor, 3 * row + column);
      }
    }
    const Matrix3d symmetric_transform = 0.5 * (
        information_transform + information_transform.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix3d> transform_solver(
        symmetric_transform);
    if (transform_solver.info() != Eigen::Success
        || transform_solver.eigenvalues().minCoeff() <= 0.0
        || transform_solver.eigenvalues().maxCoeff() > 1.0 + 1.0e-10) {
      throw std::invalid_argument(
          "LiDAR subspace transform eigenvalues must be within (0, 1]");
    }
    const Vector3d lidar_to_body_translation =
        lidar_to_body_translations.row(factor).transpose();
    const int state_index = state_indices(factor);
    const VectorXd pose = states.row(state_index).head<6>().transpose();
    const Matrix3d rotation = rpy_to_rotation(pose.segment<3>(3));
    Eigen::Matrix<double, 6, 6> local_hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> local_gradient =
        Eigen::Matrix<double, 6, 1>::Zero();
    for (int point = factor_offsets(factor);
         point < factor_offsets(factor + 1); ++point) {
      const Vector3d body_point =
          lidar_to_body_rotation * lidar_points.row(point).transpose()
          + lidar_to_body_translation;
      const Vector3d normal = plane_normals.row(point).transpose();
      const Vector3d world_point = rotation * body_point + pose.head<3>();
      const double residual = normal.dot(
          world_point - plane_points.row(point).transpose());
      const double standardized = residual / std::sqrt(variance(point));
      const double absolute = std::abs(standardized);
      double loss = 0.5 * standardized * standardized;
      double robust_weight = 1.0;
      if (huber_delta > 0.0 && absolute > huber_delta) {
        loss = huber_delta * (absolute - 0.5 * huber_delta);
        robust_weight = huber_delta / absolute;
      }
      const double information = effective_weights(factor)
          * robust_weight / variance(point);
      Eigen::Matrix<double, 6, 1> jacobian;
      jacobian.head<3>() = normal;
      jacobian.tail<3>() = body_point.cross(rotation.transpose() * normal);
      local_hessian.noalias() +=
          information * jacobian * jacobian.transpose();
      local_gradient.noalias() += information * residual * jacobian;
      cost += effective_weights(factor) * loss;
    }
    if (!symmetric_transform.isApprox(Matrix3d::Identity(), 1.0e-14)) {
      shape_conditional_translation_normal(
          local_hessian, local_gradient, symmetric_transform);
    }
    const Eigen::Index offset = state_index * kStateSize;
    hessian.block(offset, offset, 6, 6) += local_hessian;
    gradient.segment(offset, 6) += local_gradient;
  }
  return {hessian, gradient, cost};
}

VectorXd marginal_local_coordinates(
    const MatrixXd &references, const MatrixXd &states) {
  if (references.rows() <= 0 || references.cols() != kStateSize
      || states.rows() != references.rows() || states.cols() != kStateSize) {
    throw std::invalid_argument("marginal-prior states must be matching Nx15 arrays");
  }
  require_finite(references, "marginal references");
  require_finite(states, "marginal states");
  VectorXd local(references.size());
  for (Eigen::Index block = 0; block < references.rows(); ++block) {
    local.segment(block * kStateSize, kStateSize) =
        states.row(block).transpose() - references.row(block).transpose();
    const Matrix3d reference_rotation =
        rpy_to_rotation(references.row(block).segment<3>(3).transpose());
    const Matrix3d state_rotation =
        rpy_to_rotation(states.row(block).segment<3>(3).transpose());
    local.segment<3>(block * kStateSize + 3) =
        so3_log(reference_rotation.transpose() * state_rotation);
  }
  return local;
}

void validate_marginal_inputs(
    const MatrixXd &references,
    const MatrixXd &states,
    const MatrixXd &normal_hessian,
    const VectorXd &normal_gradient) {
  const Eigen::Index dimension = references.rows() * kStateSize;
  if (references.rows() <= 0 || references.cols() != kStateSize
      || states.rows() != references.rows() || states.cols() != kStateSize
      || normal_hessian.rows() != dimension
      || normal_hessian.cols() != dimension
      || normal_gradient.size() != dimension) {
    throw std::invalid_argument("marginal-prior inputs have incompatible shapes");
  }
  require_finite(normal_hessian, "marginal Hessian");
  require_finite(normal_gradient, "marginal gradient");
}

std::tuple<MatrixXd, VectorXd, double> marginal_prior_normal(
    const Eigen::Ref<const MatrixXd> &references,
    const Eigen::Ref<const MatrixXd> &states,
    const Eigen::Ref<const MatrixXd> &normal_hessian,
    const Eigen::Ref<const VectorXd> &normal_gradient) {
  validate_marginal_inputs(
      references, states, normal_hessian, normal_gradient);
  const VectorXd local = marginal_local_coordinates(references, states);
  MatrixXd transformed_hessian = normal_hessian;
  VectorXd transformed_gradient = normal_hessian * local + normal_gradient;
  for (Eigen::Index block = 0; block < references.rows(); ++block) {
    const Eigen::Index offset = block * kStateSize + 3;
    const Matrix3d jacobian = so3_right_jacobian_inverse(
        local.segment<3>(offset));
    transformed_hessian.middleRows(offset, 3) =
        jacobian.transpose() * transformed_hessian.middleRows(offset, 3);
    transformed_gradient.segment<3>(offset) =
        jacobian.transpose() * transformed_gradient.segment<3>(offset);
    transformed_hessian.middleCols(offset, 3) =
        transformed_hessian.middleCols(offset, 3) * jacobian;
  }
  const double cost = 0.5 * local.dot(normal_hessian * local)
      + normal_gradient.dot(local);
  return {transformed_hessian, transformed_gradient, cost};
}

double marginal_prior_cost(
    const Eigen::Ref<const MatrixXd> &references,
    const Eigen::Ref<const MatrixXd> &states,
    const Eigen::Ref<const MatrixXd> &normal_hessian,
    const Eigen::Ref<const VectorXd> &normal_gradient) {
  validate_marginal_inputs(
      references, states, normal_hessian, normal_gradient);
  const VectorXd local = marginal_local_coordinates(references, states);
  return 0.5 * local.dot(normal_hessian * local) + normal_gradient.dot(local);
}

void validate_visual_inputs(
    const VectorXd &anchor_state,
    const VectorXd &current_state,
    const MatrixXd &anchor_normalized,
    const MatrixXd &current_normalized,
    const VectorXd &inverse_depth,
    const VectorXd &variance,
    const Matrix3d &rotation_body_camera,
    const Vector3d &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  const Eigen::Index count = anchor_normalized.rows();
  if (anchor_state.size() != kStateSize
      || current_state.size() != kStateSize
      || count <= 0 || anchor_normalized.cols() != 2
      || current_normalized.rows() != count
      || current_normalized.cols() != 2
      || inverse_depth.size() != count
      || variance.size() != 2 * count) {
    throw std::invalid_argument(
        "visual reprojection inputs have incompatible shapes");
  }
  require_finite(anchor_state, "visual anchor state");
  require_finite(current_state, "visual current state");
  require_finite(anchor_normalized, "visual anchor observations");
  require_finite(current_normalized, "visual current observations");
  require_finite(inverse_depth, "visual inverse depth");
  require_finite(variance, "visual variance");
  if (!rotation_body_camera.allFinite()
      || !translation_body_camera.allFinite()
      || (inverse_depth.array() <= 0.0).any()
      || (variance.array() <= 0.0).any()
      || !std::isfinite(effective_weight) || effective_weight < 0.0
      || !std::isfinite(huber_delta) || huber_delta < 0.0
      || !std::isfinite(minimum_depth) || minimum_depth <= 0.0) {
    throw std::invalid_argument(
        "visual reprojection inputs are not physically valid");
  }
}

std::tuple<MatrixXd, VectorXd, double> visual_reprojection_normal(
    const Eigen::Ref<const VectorXd> &anchor_state,
    const Eigen::Ref<const VectorXd> &current_state,
    const Eigen::Ref<const MatrixXd> &anchor_normalized,
    const Eigen::Ref<const MatrixXd> &current_normalized,
    const Eigen::Ref<const VectorXd> &inverse_depth,
    const Eigen::Ref<const VectorXd> &variance,
    const Eigen::Ref<const Matrix3d> &rotation_body_camera,
    const Eigen::Ref<const Vector3d> &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  validate_visual_inputs(
      anchor_state, current_state, anchor_normalized, current_normalized,
      inverse_depth, variance, rotation_body_camera,
      translation_body_camera, effective_weight, huber_delta, minimum_depth);
  const Matrix3d rotation_anchor =
      rpy_to_rotation(anchor_state.segment<3>(3));
  const Matrix3d rotation_current =
      rpy_to_rotation(current_state.segment<3>(3));
  const Matrix3d body_to_camera = rotation_body_camera.transpose();
  const Matrix3d current_to_camera =
      body_to_camera * rotation_current.transpose();
  MatrixXd hessian = MatrixXd::Zero(2 * kStateSize, 2 * kStateSize);
  VectorXd gradient = VectorXd::Zero(2 * kStateSize);
  double cost = 0.0;
  for (Eigen::Index index = 0; index < anchor_normalized.rows(); ++index) {
    const Vector3d bearing(
        anchor_normalized(index, 0), anchor_normalized(index, 1), 1.0);
    const Vector3d point_camera_anchor = bearing / inverse_depth(index);
    const Vector3d point_body_anchor =
        rotation_body_camera * point_camera_anchor
        + translation_body_camera;
    const Vector3d point_world =
        rotation_anchor * point_body_anchor + anchor_state.head<3>();
    const Vector3d point_body_current = rotation_current.transpose()
        * (point_world - current_state.head<3>());
    const Vector3d point_camera_current = body_to_camera
        * (point_body_current - translation_body_camera);
    const double depth = point_camera_current.z();
    if (!std::isfinite(depth) || depth <= minimum_depth) {
      continue;
    }
    Eigen::Matrix<double, 2, 3> projection_jacobian;
    const double inverse_z = 1.0 / depth;
    const double inverse_z_squared = inverse_z * inverse_z;
    projection_jacobian <<
        inverse_z, 0.0, -point_camera_current.x() * inverse_z_squared,
        0.0, inverse_z, -point_camera_current.y() * inverse_z_squared;
    Eigen::Matrix<double, 2, 30> jacobian;
    jacobian.setZero();
    jacobian.block<2, 3>(0, 0) =
        projection_jacobian * current_to_camera;
    jacobian.block<2, 3>(0, 3) = projection_jacobian
        * (-current_to_camera * rotation_anchor * skew(point_body_anchor));
    jacobian.block<2, 3>(0, kStateSize) =
        -jacobian.block<2, 3>(0, 0);
    jacobian.block<2, 3>(0, kStateSize + 3) = projection_jacobian
        * body_to_camera * skew(point_body_current);
    const Eigen::Vector2d prediction =
        point_camera_current.head<2>() * inverse_z;
    const Eigen::Vector2d residual =
        prediction - current_normalized.row(index).transpose();
    for (int axis = 0; axis < 2; ++axis) {
      const double component_variance = variance(2 * index + axis);
      const double standardized =
          residual(axis) / std::sqrt(component_variance);
      const double absolute = std::abs(standardized);
      double loss = 0.5 * standardized * standardized;
      double robust_weight = 1.0;
      if (huber_delta > 0.0 && absolute > huber_delta) {
        loss = huber_delta * (absolute - 0.5 * huber_delta);
        robust_weight = huber_delta / absolute;
      }
      const double information =
          effective_weight * robust_weight / component_variance;
      const Eigen::Matrix<double, 30, 1> row =
          jacobian.row(axis).transpose();
      hessian.noalias() += information * row * row.transpose();
      gradient.noalias() += information * residual(axis) * row;
      cost += effective_weight * loss;
    }
  }
  return {hessian, gradient, cost};
}

double visual_reprojection_cost(
    const Eigen::Ref<const VectorXd> &anchor_state,
    const Eigen::Ref<const VectorXd> &current_state,
    const Eigen::Ref<const MatrixXd> &anchor_normalized,
    const Eigen::Ref<const MatrixXd> &current_normalized,
    const Eigen::Ref<const VectorXd> &inverse_depth,
    const Eigen::Ref<const VectorXd> &variance,
    const Eigen::Ref<const Matrix3d> &rotation_body_camera,
    const Eigen::Ref<const Vector3d> &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  validate_visual_inputs(
      anchor_state, current_state, anchor_normalized, current_normalized,
      inverse_depth, variance, rotation_body_camera,
      translation_body_camera, effective_weight, huber_delta, minimum_depth);
  const Matrix3d rotation_anchor =
      rpy_to_rotation(anchor_state.segment<3>(3));
  const Matrix3d rotation_current =
      rpy_to_rotation(current_state.segment<3>(3));
  const Matrix3d body_to_camera = rotation_body_camera.transpose();
  double cost = 0.0;
  for (Eigen::Index index = 0; index < anchor_normalized.rows(); ++index) {
    const Vector3d bearing(
        anchor_normalized(index, 0), anchor_normalized(index, 1), 1.0);
    const Vector3d point_camera_anchor = bearing / inverse_depth(index);
    const Vector3d point_body_anchor =
        rotation_body_camera * point_camera_anchor
        + translation_body_camera;
    const Vector3d point_world =
        rotation_anchor * point_body_anchor + anchor_state.head<3>();
    const Vector3d point_body_current = rotation_current.transpose()
        * (point_world - current_state.head<3>());
    const Vector3d point_camera_current = body_to_camera
        * (point_body_current - translation_body_camera);
    const double depth = point_camera_current.z();
    if (!std::isfinite(depth) || depth <= minimum_depth) {
      continue;
    }
    const Eigen::Vector2d prediction =
        point_camera_current.head<2>() / depth;
    const Eigen::Vector2d residual =
        prediction - current_normalized.row(index).transpose();
    for (int axis = 0; axis < 2; ++axis) {
      const double component_variance = variance(2 * index + axis);
      const double standardized =
          residual(axis) / std::sqrt(component_variance);
      const double absolute = std::abs(standardized);
      const double loss =
          huber_delta > 0.0 && absolute > huber_delta
          ? huber_delta * (absolute - 0.5 * huber_delta)
          : 0.5 * standardized * standardized;
      cost += effective_weight * loss;
    }
  }
  return cost;
}

void validate_rgbd_depth_inputs(
    const VectorXd &anchor_state,
    const VectorXd &current_state,
    const MatrixXd &anchor_normalized,
    const VectorXd &anchor_depth,
    const VectorXd &current_depth,
    const VectorXd &variance,
    const Matrix3d &rotation_body_camera,
    const Vector3d &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  const Eigen::Index count = anchor_normalized.rows();
  if (anchor_state.size() != kStateSize
      || current_state.size() != kStateSize
      || count <= 0 || anchor_normalized.cols() != 2
      || anchor_depth.size() != count
      || current_depth.size() != count
      || variance.size() != count) {
    throw std::invalid_argument("RGB-D depth inputs have incompatible shapes");
  }
  require_finite(anchor_state, "RGB-D anchor state");
  require_finite(current_state, "RGB-D current state");
  require_finite(anchor_normalized, "RGB-D anchor observations");
  require_finite(anchor_depth, "RGB-D anchor depth");
  require_finite(current_depth, "RGB-D current depth");
  require_finite(variance, "RGB-D depth variance");
  if (!rotation_body_camera.allFinite()
      || !translation_body_camera.allFinite()
      || (anchor_depth.array() <= 0.0).any()
      || (current_depth.array() <= 0.0).any()
      || (variance.array() <= 0.0).any()
      || !std::isfinite(effective_weight) || effective_weight < 0.0
      || !std::isfinite(huber_delta) || huber_delta < 0.0
      || !std::isfinite(minimum_depth) || minimum_depth <= 0.0) {
    throw std::invalid_argument("RGB-D depth inputs are not physically valid");
  }
}

std::tuple<MatrixXd, VectorXd, double> rgbd_depth_normal(
    const Eigen::Ref<const VectorXd> &anchor_state,
    const Eigen::Ref<const VectorXd> &current_state,
    const Eigen::Ref<const MatrixXd> &anchor_normalized,
    const Eigen::Ref<const VectorXd> &anchor_depth,
    const Eigen::Ref<const VectorXd> &current_depth,
    const Eigen::Ref<const VectorXd> &variance,
    const Eigen::Ref<const Matrix3d> &rotation_body_camera,
    const Eigen::Ref<const Vector3d> &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  validate_rgbd_depth_inputs(
      anchor_state, current_state, anchor_normalized, anchor_depth,
      current_depth, variance, rotation_body_camera,
      translation_body_camera, effective_weight, huber_delta, minimum_depth);
  const Matrix3d rotation_anchor =
      rpy_to_rotation(anchor_state.segment<3>(3));
  const Matrix3d rotation_current =
      rpy_to_rotation(current_state.segment<3>(3));
  const Matrix3d body_to_camera = rotation_body_camera.transpose();
  const Matrix3d current_to_camera =
      body_to_camera * rotation_current.transpose();
  MatrixXd hessian = MatrixXd::Zero(2 * kStateSize, 2 * kStateSize);
  VectorXd gradient = VectorXd::Zero(2 * kStateSize);
  double cost = 0.0;
  for (Eigen::Index index = 0; index < anchor_normalized.rows(); ++index) {
    const Vector3d bearing(
        anchor_normalized(index, 0), anchor_normalized(index, 1), 1.0);
    const Vector3d point_camera_anchor = bearing * anchor_depth(index);
    const Vector3d point_body_anchor =
        rotation_body_camera * point_camera_anchor
        + translation_body_camera;
    const Vector3d point_world =
        rotation_anchor * point_body_anchor + anchor_state.head<3>();
    const Vector3d point_body_current = rotation_current.transpose()
        * (point_world - current_state.head<3>());
    const Vector3d point_camera_current = body_to_camera
        * (point_body_current - translation_body_camera);
    const double predicted_depth = point_camera_current.z();
    if (!std::isfinite(predicted_depth) || predicted_depth <= minimum_depth) {
      continue;
    }
    Eigen::Matrix<double, 1, 30> jacobian;
    jacobian.setZero();
    jacobian.block<1, 3>(0, 0) = current_to_camera.row(2);
    jacobian.block<1, 3>(0, 3) =
        (-current_to_camera * rotation_anchor * skew(point_body_anchor)).row(2);
    jacobian.block<1, 3>(0, kStateSize) = -current_to_camera.row(2);
    jacobian.block<1, 3>(0, kStateSize + 3) =
        (body_to_camera * skew(point_body_current)).row(2);
    const double residual = predicted_depth - current_depth(index);
    const double standardized = residual / std::sqrt(variance(index));
    const double absolute = std::abs(standardized);
    double loss = 0.5 * standardized * standardized;
    double robust_weight = 1.0;
    if (huber_delta > 0.0 && absolute > huber_delta) {
      loss = huber_delta * (absolute - 0.5 * huber_delta);
      robust_weight = huber_delta / absolute;
    }
    const double information =
        effective_weight * robust_weight / variance(index);
    const Eigen::Matrix<double, 30, 1> row = jacobian.transpose();
    hessian.noalias() += information * row * row.transpose();
    gradient.noalias() += information * residual * row;
    cost += effective_weight * loss;
  }
  return {hessian, gradient, cost};
}

double rgbd_depth_cost(
    const Eigen::Ref<const VectorXd> &anchor_state,
    const Eigen::Ref<const VectorXd> &current_state,
    const Eigen::Ref<const MatrixXd> &anchor_normalized,
    const Eigen::Ref<const VectorXd> &anchor_depth,
    const Eigen::Ref<const VectorXd> &current_depth,
    const Eigen::Ref<const VectorXd> &variance,
    const Eigen::Ref<const Matrix3d> &rotation_body_camera,
    const Eigen::Ref<const Vector3d> &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  validate_rgbd_depth_inputs(
      anchor_state, current_state, anchor_normalized, anchor_depth,
      current_depth, variance, rotation_body_camera,
      translation_body_camera, effective_weight, huber_delta, minimum_depth);
  const Matrix3d rotation_anchor =
      rpy_to_rotation(anchor_state.segment<3>(3));
  const Matrix3d rotation_current =
      rpy_to_rotation(current_state.segment<3>(3));
  const Matrix3d body_to_camera = rotation_body_camera.transpose();
  double cost = 0.0;
  for (Eigen::Index index = 0; index < anchor_normalized.rows(); ++index) {
    const Vector3d bearing(
        anchor_normalized(index, 0), anchor_normalized(index, 1), 1.0);
    const Vector3d point_camera_anchor = bearing * anchor_depth(index);
    const Vector3d point_body_anchor =
        rotation_body_camera * point_camera_anchor
        + translation_body_camera;
    const Vector3d point_world =
        rotation_anchor * point_body_anchor + anchor_state.head<3>();
    const Vector3d point_body_current = rotation_current.transpose()
        * (point_world - current_state.head<3>());
    const Vector3d point_camera_current = body_to_camera
        * (point_body_current - translation_body_camera);
    const double predicted_depth = point_camera_current.z();
    if (!std::isfinite(predicted_depth) || predicted_depth <= minimum_depth) {
      continue;
    }
    const double residual = predicted_depth - current_depth(index);
    const double standardized = residual / std::sqrt(variance(index));
    const double absolute = std::abs(standardized);
    const double loss =
        huber_delta > 0.0 && absolute > huber_delta
        ? huber_delta * (absolute - 0.5 * huber_delta)
        : 0.5 * standardized * standardized;
    cost += effective_weight * loss;
  }
  return cost;
}

void validate_rgbd_direct_inputs(
    const VectorXd &anchor_state,
    const VectorXd &current_state,
    const MatrixXd &anchor_normalized,
    const MatrixXd &current_normalized,
    const VectorXd &anchor_depth,
    const VectorXd &current_depth,
    const VectorXd &depth_variance,
    const VectorXd &previous_intensity,
    const VectorXd &current_intensity,
    const MatrixXd &current_gradient,
    const VectorXd &photometric_variance,
    const Matrix3d &rotation_body_camera,
    const Vector3d &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  const Eigen::Index count = anchor_normalized.rows();
  if (anchor_state.size() != kStateSize
      || current_state.size() != kStateSize || count <= 0
      || anchor_normalized.cols() != 2
      || current_normalized.rows() != count
      || current_normalized.cols() != 2
      || anchor_depth.size() != count || current_depth.size() != count
      || depth_variance.size() != count
      || previous_intensity.size() != count
      || current_intensity.size() != count
      || current_gradient.rows() != count || current_gradient.cols() != 2
      || photometric_variance.size() != count) {
    throw std::invalid_argument("RGB-D direct inputs have incompatible shapes");
  }
  require_finite(anchor_state, "RGB-D direct anchor state");
  require_finite(current_state, "RGB-D direct current state");
  require_finite(anchor_normalized, "RGB-D direct anchor observations");
  require_finite(current_normalized, "RGB-D direct current observations");
  require_finite(anchor_depth, "RGB-D direct anchor depth");
  require_finite(current_depth, "RGB-D direct current depth");
  require_finite(depth_variance, "RGB-D direct depth variance");
  require_finite(previous_intensity, "RGB-D direct previous intensity");
  require_finite(current_intensity, "RGB-D direct current intensity");
  require_finite(current_gradient, "RGB-D direct current gradient");
  require_finite(photometric_variance, "RGB-D direct photometric variance");
  if (!rotation_body_camera.allFinite()
      || !translation_body_camera.allFinite()
      || (anchor_depth.array() <= 0.0).any()
      || (current_depth.array() <= 0.0).any()
      || (depth_variance.array() <= 0.0).any()
      || (photometric_variance.array() <= 0.0).any()
      || !std::isfinite(effective_weight) || effective_weight < 0.0
      || !std::isfinite(huber_delta) || huber_delta < 0.0
      || !std::isfinite(minimum_depth) || minimum_depth <= 0.0) {
    throw std::invalid_argument("RGB-D direct inputs are not physically valid");
  }
}

std::tuple<MatrixXd, VectorXd, double> rgbd_direct_normal(
    const Eigen::Ref<const VectorXd> &anchor_state,
    const Eigen::Ref<const VectorXd> &current_state,
    const Eigen::Ref<const MatrixXd> &anchor_normalized,
    const Eigen::Ref<const MatrixXd> &current_normalized,
    const Eigen::Ref<const VectorXd> &anchor_depth,
    const Eigen::Ref<const VectorXd> &current_depth,
    const Eigen::Ref<const VectorXd> &depth_variance,
    const Eigen::Ref<const VectorXd> &previous_intensity,
    const Eigen::Ref<const VectorXd> &current_intensity,
    const Eigen::Ref<const MatrixXd> &current_gradient,
    const Eigen::Ref<const VectorXd> &photometric_variance,
    const Eigen::Ref<const Matrix3d> &rotation_body_camera,
    const Eigen::Ref<const Vector3d> &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  validate_rgbd_direct_inputs(
      anchor_state, current_state, anchor_normalized, current_normalized,
      anchor_depth, current_depth, depth_variance, previous_intensity,
      current_intensity, current_gradient, photometric_variance,
      rotation_body_camera, translation_body_camera, effective_weight,
      huber_delta, minimum_depth);
  const Matrix3d rotation_anchor =
      rpy_to_rotation(anchor_state.segment<3>(3));
  const Matrix3d rotation_current =
      rpy_to_rotation(current_state.segment<3>(3));
  const Matrix3d body_to_camera = rotation_body_camera.transpose();
  const Matrix3d current_to_camera =
      body_to_camera * rotation_current.transpose();
  MatrixXd hessian = MatrixXd::Zero(2 * kStateSize, 2 * kStateSize);
  VectorXd gradient = VectorXd::Zero(2 * kStateSize);
  double cost = 0.0;
  for (Eigen::Index index = 0; index < anchor_normalized.rows(); ++index) {
    const Vector3d bearing(
        anchor_normalized(index, 0), anchor_normalized(index, 1), 1.0);
    const Vector3d point_camera_anchor = bearing * anchor_depth(index);
    const Vector3d point_body_anchor =
        rotation_body_camera * point_camera_anchor
        + translation_body_camera;
    const Vector3d point_world =
        rotation_anchor * point_body_anchor + anchor_state.head<3>();
    const Vector3d point_body_current = rotation_current.transpose()
        * (point_world - current_state.head<3>());
    const Vector3d point_camera_current = body_to_camera
        * (point_body_current - translation_body_camera);
    const double depth = point_camera_current.z();
    if (!std::isfinite(depth) || depth <= minimum_depth) {
      continue;
    }
    const double inverse_z = 1.0 / depth;
    const double inverse_z_squared = inverse_z * inverse_z;
    Eigen::Matrix<double, 2, 3> projection_jacobian;
    projection_jacobian <<
        inverse_z, 0.0, -point_camera_current.x() * inverse_z_squared,
        0.0, inverse_z, -point_camera_current.y() * inverse_z_squared;
    Eigen::Matrix<double, 2, 30> pose_jacobian;
    pose_jacobian.setZero();
    pose_jacobian.block<2, 3>(0, 0) =
        projection_jacobian * current_to_camera;
    pose_jacobian.block<2, 3>(0, 3) = projection_jacobian
        * (-current_to_camera * rotation_anchor * skew(point_body_anchor));
    pose_jacobian.block<2, 3>(0, kStateSize) =
        -pose_jacobian.block<2, 3>(0, 0);
    pose_jacobian.block<2, 3>(0, kStateSize + 3) = projection_jacobian
        * body_to_camera * skew(point_body_current);
    Eigen::Matrix<double, 1, 30> depth_jacobian;
    depth_jacobian.setZero();
    depth_jacobian.block<1, 3>(0, 0) = current_to_camera.row(2);
    depth_jacobian.block<1, 3>(0, 3) =
        (-current_to_camera * rotation_anchor * skew(point_body_anchor)).row(2);
    depth_jacobian.block<1, 3>(0, kStateSize) = -current_to_camera.row(2);
    depth_jacobian.block<1, 3>(0, kStateSize + 3) =
        (body_to_camera * skew(point_body_current)).row(2);
    const Vector2d prediction =
        point_camera_current.head<2>() * inverse_z;
    const Vector2d image_delta =
        prediction - current_normalized.row(index).transpose();
    const double depth_residual = depth - current_depth(index);
    const double photometric_residual =
        current_intensity(index) - previous_intensity(index)
        + current_gradient.row(index).dot(image_delta);
    const Eigen::Matrix<double, 1, 30> photometric_jacobian =
        current_gradient.row(index) * pose_jacobian;
    const std::array<double, 2> residuals = {
        depth_residual, photometric_residual};
    const std::array<double, 2> variances = {
        depth_variance(index), photometric_variance(index)};
    for (int component = 0; component < 2; ++component) {
      const double residual = residuals[component];
      const double variance = variances[component];
      const double standardized = residual / std::sqrt(variance);
      const double absolute = std::abs(standardized);
      double loss = 0.5 * standardized * standardized;
      double robust_weight = 1.0;
      if (huber_delta > 0.0 && absolute > huber_delta) {
        loss = huber_delta * (absolute - 0.5 * huber_delta);
        robust_weight = huber_delta / absolute;
      }
      const double information =
          effective_weight * robust_weight / variance;
      const Eigen::Matrix<double, 30, 1> row =
          (component == 0 ? depth_jacobian : photometric_jacobian).transpose();
      hessian.noalias() += information * row * row.transpose();
      gradient.noalias() += information * residual * row;
      cost += effective_weight * loss;
    }
  }
  return {hessian, gradient, cost};
}

double rgbd_direct_cost(
    const Eigen::Ref<const VectorXd> &anchor_state,
    const Eigen::Ref<const VectorXd> &current_state,
    const Eigen::Ref<const MatrixXd> &anchor_normalized,
    const Eigen::Ref<const MatrixXd> &current_normalized,
    const Eigen::Ref<const VectorXd> &anchor_depth,
    const Eigen::Ref<const VectorXd> &current_depth,
    const Eigen::Ref<const VectorXd> &depth_variance,
    const Eigen::Ref<const VectorXd> &previous_intensity,
    const Eigen::Ref<const VectorXd> &current_intensity,
    const Eigen::Ref<const MatrixXd> &current_gradient,
    const Eigen::Ref<const VectorXd> &photometric_variance,
    const Eigen::Ref<const Matrix3d> &rotation_body_camera,
    const Eigen::Ref<const Vector3d> &translation_body_camera,
    double effective_weight,
    double huber_delta,
    double minimum_depth) {
  validate_rgbd_direct_inputs(
      anchor_state, current_state, anchor_normalized, current_normalized,
      anchor_depth, current_depth, depth_variance, previous_intensity,
      current_intensity, current_gradient, photometric_variance,
      rotation_body_camera, translation_body_camera, effective_weight,
      huber_delta, minimum_depth);
  const Matrix3d rotation_anchor =
      rpy_to_rotation(anchor_state.segment<3>(3));
  const Matrix3d rotation_current =
      rpy_to_rotation(current_state.segment<3>(3));
  const Matrix3d body_to_camera = rotation_body_camera.transpose();
  double cost = 0.0;
  for (Eigen::Index index = 0; index < anchor_normalized.rows(); ++index) {
    const Vector3d bearing(
        anchor_normalized(index, 0), anchor_normalized(index, 1), 1.0);
    const Vector3d point_camera_anchor = bearing * anchor_depth(index);
    const Vector3d point_body_anchor =
        rotation_body_camera * point_camera_anchor
        + translation_body_camera;
    const Vector3d point_world =
        rotation_anchor * point_body_anchor + anchor_state.head<3>();
    const Vector3d point_body_current = rotation_current.transpose()
        * (point_world - current_state.head<3>());
    const Vector3d point_camera_current = body_to_camera
        * (point_body_current - translation_body_camera);
    const double depth = point_camera_current.z();
    if (!std::isfinite(depth) || depth <= minimum_depth) {
      continue;
    }
    const Vector2d prediction = point_camera_current.head<2>() / depth;
    const Vector2d image_delta =
        prediction - current_normalized.row(index).transpose();
    const std::array<double, 2> residuals = {
        depth - current_depth(index),
        current_intensity(index) - previous_intensity(index)
            + current_gradient.row(index).dot(image_delta)};
    const std::array<double, 2> variances = {
        depth_variance(index), photometric_variance(index)};
    for (int component = 0; component < 2; ++component) {
      const double standardized =
          residuals[component] / std::sqrt(variances[component]);
      const double absolute = std::abs(standardized);
      const double loss =
          huber_delta > 0.0 && absolute > huber_delta
          ? huber_delta * (absolute - 0.5 * huber_delta)
          : 0.5 * standardized * standardized;
      cost += effective_weight * loss;
    }
  }
  return cost;
}

}  // namespace

PYBIND11_MODULE(_core, module) {
  module.doc() = "Eigen normal-equation kernels for Ultra-Fusion";
  // Keep the GIL across these fine-grained calls. Releasing and reacquiring it
  // for every factor lets high-rate ROS callbacks starve the optimizer between
  // kernels; each individual Eigen operation is intentionally short.
  module.def("imu_preintegrated_normal", &imu_preintegrated_normal);
  module.def(
      "imu_preintegrated_graph_normal", &imu_preintegrated_graph_normal);
  module.def("imu_preintegrated_cost", &imu_preintegrated_cost);
  module.def("lidar_point_plane_normal", &lidar_point_plane_normal);
  module.def(
      "lidar_point_plane_normal_axis_scaled",
      &lidar_point_plane_normal_axis_scaled);
  module.def(
      "lidar_point_plane_graph_normal", &lidar_point_plane_graph_normal);
  module.def(
      "lidar_point_plane_graph_normal_axis_scaled",
      &lidar_point_plane_graph_normal_axis_scaled);
  module.def(
      "lidar_point_plane_graph_normal_subspace",
      &lidar_point_plane_graph_normal_subspace);
  module.def("lidar_point_plane_cost", &lidar_point_plane_cost);
  module.def("marginal_prior_normal", &marginal_prior_normal);
  module.def("marginal_prior_cost", &marginal_prior_cost);
  module.def("visual_reprojection_normal", &visual_reprojection_normal);
  module.def("visual_reprojection_cost", &visual_reprojection_cost);
  module.def("rgbd_depth_normal", &rgbd_depth_normal);
  module.def("rgbd_depth_cost", &rgbd_depth_cost);
  module.def("rgbd_direct_normal", &rgbd_direct_normal);
  module.def("rgbd_direct_cost", &rgbd_direct_cost);
  module.def("state_plus_batch", &state_plus_batch);
}
