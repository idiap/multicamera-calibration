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

// Inspired from : ceres solver examples/bal_problem.h
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
//
// Class for loading and holding in memory bundle adjustment problems
// from the BAL (Bundle Adjustment in the Large) dataset from the
// University of Washington.
//
// For more details see http://grail.cs.washington.edu/projects/bal/

#ifndef _BA_PROBLEM_HPP_
#define _BA_PROBLEM_HPP_

#include <cstdio>
#include <cstring>
#include "ceres/ceres.h"

class BAProblem {
 public:
  BAProblem(const std::string filename);
  ~BAProblem();
  int num_observations() const {return num_observations_;};
  const double* observations() const {return observations_;};
  int num_fixed_observations() const {return num_fixed_observations_;};
  const double* fixed_observations() const {return observations_ + 2*num_observations_;};
  int num_cameras() const {return num_cameras_;};
  int num_points() const {return num_points_;};
  int num_fixed_points() const {return num_fixed_points_;};
  int point_block_size() const {return NUM_COORDINATES;};
  int camera_block_size() const {return NUM_CAMERA_PARAMETERS;};
  double* mutable_cameras() {return parameters_;};
  double* mutable_points(){return mutable_cameras() + NUM_CAMERA_PARAMETERS * num_cameras_;};
  double* fixed_points(){return mutable_points() + NUM_COORDINATES * num_points_;};
  double* mutable_camera_for_observation(int i){return mutable_cameras() + 
                                                camera_index_[i] * NUM_CAMERA_PARAMETERS;}; 
  double* mutable_camera_for_fixed_observation(int i){return mutable_cameras() +
                                                      camera_index_[i + num_observations_] *
                                                      NUM_CAMERA_PARAMETERS;}; 
  double* mutable_point_for_observation(int i){return mutable_points() +
                                               point_index_[i] * NUM_COORDINATES;}; 
  double* fixed_point_for_observation(int i){return fixed_points() +
                                             point_index_[i + num_observations_] * NUM_COORDINATES;};
  bool LoadFile(const std::string filename);
  bool WriteFile(const std::string filename);

 private:

  static const int NUM_CAMERA_PARAMETERS = 15;
  static const int NUM_COORDINATES = 3;

  template<typename T>
  void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "Invalid UW data file.";
    }
  }

  int num_cameras_;
  int num_points_;
  int num_fixed_points_;
  int num_observations_;
  int num_fixed_observations_;
  int num_parameters_;

  int* point_index_;
  int* fixed_point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
};

#endif /* ifndef _BA_PROBLEM_HPP_ */
