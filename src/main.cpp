/***************************************************************************************************
* Multi-Camera Calibration Suite was built to help with intrinsic and extrinsic multi-camera
* calibration. It also specifically contains a bundle adjustment module to help with the joint
* calibration of the cameras.
* 
* Copyright (c) 2017 Idiap Research Institute, http://www.idiap.ch/
* Written by Salim Kayal <salim.kayal@idiap.ch>,
* 
* This file is part of Multi-Camera Calibration Suite.
* 
* Multi-Camera Calibration Suite is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 3 as
* published by the Free Software Foundation.
* 
* Multi-Camera Calibration Suite is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with Multi-Camera Calibration Suite. If not, see <http://www.gnu.org/licenses/>.
***************************************************************************************************/

#include "BAProblem.hpp"
#include "SnavelyReprojectionError.hpp"

#include <iostream>
#include <memory>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include <omp.h>

DEFINE_string(input, "", "input file name");
DEFINE_string(output, "", "output file name");
DEFINE_string(intrinsic_adjustment, "fixed", "Options are: fixed, unconstrained, two_pass");

bool setOrdering(BAProblem &problem, ceres::Solver::Options &options, bool fix_intrinsics = false){
  const int num_points = problem.num_points();
  const int point_block_size = problem.point_block_size();
  double* points = problem.mutable_points();
  //const int num_fixed_points = problem.num_fixed_points();
  //double* fixed_points = problem.fixed_points();
  const int num_cameras = problem.num_cameras();
  const int camera_block_size = problem.camera_block_size();
  double* cameras = problem.mutable_cameras();
  ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;
  ceres::ParameterBlockOrdering* inner_ordering = new ceres::ParameterBlockOrdering();
  // The points come before the cameras.
  for (int i = 0; i < num_points; ++i) {
    ordering->AddElementToGroup(points + point_block_size * i, 0);
    inner_ordering->AddElementToGroup(points + point_block_size * i, 0);
  }
  for (int i = 0; i < num_cameras; ++i) {
    ordering->AddElementToGroup(cameras + camera_block_size * i, 1);
    inner_ordering->AddElementToGroup(cameras + camera_block_size * i, 1);
    if (fix_intrinsics == false){
      ordering->AddElementToGroup(cameras + 6 + camera_block_size * i, 2);
      inner_ordering->AddElementToGroup(cameras + 6 + camera_block_size * i, 2);
    }
  }
  options.linear_solver_ordering.reset(ordering);
  if (options.use_inner_iterations){
    options.inner_iteration_ordering.reset(inner_ordering);  
  }
  else{
    delete inner_ordering;
  }
  return true;
}

bool buildCeresProblem(BAProblem &ba_problem, ceres::Problem &problem, bool fix_intrinsics = false){
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  const double* observations = ba_problem.observations();
  const int num_observations = ba_problem.num_observations();
  const double* fixed_observations = ba_problem.fixed_observations();
  const int num_fixed_observations = ba_problem.num_fixed_observations();
  for (int i = 0; i < ba_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1);
    double* mutable_camera = ba_problem.mutable_camera_for_observation(i);
    double* mutable_point = ba_problem.mutable_point_for_observation(i);
    problem.AddResidualBlock(cost_function,
                             loss_function,
                             mutable_camera,
                             mutable_camera + 6,
                             mutable_point);
  }
  for (int i = 0; i < ba_problem.num_fixed_observations(); ++i){
    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(fixed_observations[2 * i + 0],
                                         fixed_observations[2 * i + 1]);
    ceres::LossFunction* loss_function = new ceres::ScaledLoss(
        new ceres::HuberLoss(1),
        (double) num_observations / num_fixed_observations,
        ceres::TAKE_OWNERSHIP);
    double* mutable_camera = ba_problem.mutable_camera_for_fixed_observation(i);
    double* fixed_point = ba_problem.fixed_point_for_observation(i);
    problem.AddResidualBlock(cost_function,
                             loss_function,
                             mutable_camera,
                             mutable_camera + 6,
                             fixed_point);
    problem.SetParameterBlockConstant(fixed_point);
  }
  const int num_cameras = ba_problem.num_cameras();
  const int camera_block_size = ba_problem.camera_block_size();
  double* cameras = ba_problem.mutable_cameras();
  for (int i = 0; i < num_cameras; ++i) {
    if (fix_intrinsics){
      problem.SetParameterBlockConstant(cameras + camera_block_size * i + 6);
    }
    else{
      problem.SetParameterBlockVariable(cameras + camera_block_size * i + 6);
    }
  }
  return true;
}

bool buildCeresOptions(BAProblem &ba_problem, ceres::Solver::Options &options,
                       bool fix_intrinsics = false){
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.preconditioner_type= ceres::SCHUR_JACOBI;
  options.use_inner_iterations = true;
  options.use_nonmonotonic_steps = true;
  options.max_num_iterations = 200000;
  options.num_threads = omp_get_max_threads();
  options.minimizer_progress_to_stdout = true;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-16;
  options.max_num_consecutive_invalid_steps = 20;
  setOrdering(ba_problem, options, fix_intrinsics);
  return true;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage("bundle_adjuster [options] --input=input --output=output --intrinsic_adjustment=[fixed, unconstrained, two_pass]");
  if (FLAGS_input.empty() || FLAGS_output.empty()){
      std::cout << "Usage: " << google::ProgramUsage() << std::endl;
      return 1;
  }
  std::cout << "open file " << FLAGS_input << "\n";
  BAProblem ba_problem(FLAGS_input);
  ceres::Problem* problem = new ceres::Problem;
  bool constrain = FLAGS_intrinsic_adjustment.compare("unconstrained") != 0;
  bool two_pass = FLAGS_intrinsic_adjustment.compare("two_pass") == 0;
  buildCeresProblem(ba_problem, *problem, constrain);
  ceres::Solver::Options options;
  buildCeresOptions(ba_problem, options, constrain);
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << "\n";
  if (two_pass){
    delete problem;
    problem = new ceres::Problem;
    buildCeresProblem(ba_problem, *problem);
    buildCeresOptions(ba_problem, options);
    ceres::Solve(options, problem, &summary);
    std::cout << summary.FullReport() << "\n";
  }
  delete problem;
  if (!ba_problem.WriteFile(FLAGS_output)) {
    std::cerr << "ERROR: unable to open file " << argv[2] << "\n";
    return 1;
  }
  return 0;
}
