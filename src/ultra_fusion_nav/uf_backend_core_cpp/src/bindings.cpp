#include <algorithm>
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
using Eigen::VectorXd;

namespace {

constexpr int kStateSize = 15;

Matrix3d skew(const Vector3d &value) {
  Matrix3d matrix;
  matrix << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return matrix;
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
    jacobian.head<3>() = normal;
    jacobian.tail<3>() = body_point.cross(rotation.transpose() * normal);
    hessian.noalias() += information * jacobian * jacobian.transpose();
    gradient.noalias() += information * residual * jacobian;
    cost += effective_weight * loss;
  }
  return {hessian, gradient, cost};
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

}  // namespace

PYBIND11_MODULE(_core, module) {
  module.doc() = "Eigen normal-equation kernels for Ultra-Fusion";
  // Keep the GIL across these fine-grained calls. Releasing and reacquiring it
  // for every factor lets high-rate ROS callbacks starve the optimizer between
  // kernels; each individual Eigen operation is intentionally short.
  module.def("imu_preintegrated_normal", &imu_preintegrated_normal);
  module.def("imu_preintegrated_cost", &imu_preintegrated_cost);
  module.def("lidar_point_plane_normal", &lidar_point_plane_normal);
  module.def("lidar_point_plane_cost", &lidar_point_plane_cost);
  module.def("marginal_prior_normal", &marginal_prior_normal);
  module.def("marginal_prior_cost", &marginal_prior_cost);
  module.def("visual_reprojection_normal", &visual_reprojection_normal);
  module.def("visual_reprojection_cost", &visual_reprojection_cost);
  module.def("state_plus_batch", &state_plus_batch);
}
