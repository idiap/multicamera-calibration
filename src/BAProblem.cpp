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

// Inspired from : ceres solver examples/bal_problem.cc
//
// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)

#include "BAProblem.hpp"

BAProblem::~BAProblem() {
  delete[] point_index_;
  delete[] camera_index_;
  delete[] observations_;
  delete[] parameters_;
}

BAProblem::BAProblem(const std::string filename) {
  bool ret = LoadFile(filename);
  if (!ret) LOG(FATAL) << "Invalid input file";
}

bool BAProblem::LoadFile(const std::string filename) {
  FILE* fptr = fopen(filename.c_str(), "r");
  if (fptr == NULL) {
    return false;
  };

  FscanfOrDie(fptr, "%d", &num_cameras_);
  FscanfOrDie(fptr, "%d", &num_points_);
  FscanfOrDie(fptr, "%d", &num_fixed_points_);
  FscanfOrDie(fptr, "%d", &num_observations_);
  FscanfOrDie(fptr, "%d", &num_fixed_observations_);

  point_index_ = new int[num_observations_ + num_fixed_observations_];
  fixed_point_index_ = point_index_ + num_observations_;
  camera_index_ = new int[num_observations_ + num_fixed_observations_];
  observations_ = new double[2 * num_observations_ + 2 * num_fixed_observations_];

  num_parameters_ = NUM_CAMERA_PARAMETERS * num_cameras_ +
                    NUM_COORDINATES * num_points_ +
                    NUM_COORDINATES * num_fixed_points_;
  parameters_ = new double[num_parameters_];

  for (int i = 0; i < num_observations_ + num_fixed_observations_; ++i) {
    FscanfOrDie(fptr, "%d", camera_index_ + i);
    FscanfOrDie(fptr, "%d", point_index_ + i);
    for (int j = 0; j < 2; ++j) {
      FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
    }
  }

  for (int i = 0; i < num_parameters_; ++i) {
    FscanfOrDie(fptr, "%lf", parameters_ + i);
  }
  return true;
}

bool BAProblem::WriteFile(const std::string filename) {
  FILE* fptr = fopen(filename.c_str(), "w");
  if (fptr == NULL) {
    return false;
  };

  fprintf(fptr, "%d ", num_cameras_);
  fprintf(fptr, "%d ", num_points_);
  fprintf(fptr, "%d ", num_fixed_points_);
  fprintf(fptr, "%d ", num_observations_);
  fprintf(fptr, "%d \n", num_fixed_observations_);

  for (int i = 0; i < num_observations_ + num_fixed_observations_; ++i) {
    fprintf(fptr, "%d ", *(camera_index_ + i));
    fprintf(fptr, "%d ", *(point_index_ + i));
    for (int j = 0; j < 2; ++j) {
      fprintf(fptr, "%lf", *(observations_ + 2*i + j));
      if (j == 0) fprintf(fptr, " ");
      else fprintf(fptr, "\n");
    }
  }

  for (int i = 0; i < num_parameters_; ++i) {
    fprintf(fptr, "%lf\n", *(parameters_ + i));
  }
  return true;
}
