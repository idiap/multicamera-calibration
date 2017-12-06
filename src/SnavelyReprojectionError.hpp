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

// Inspired from : ceres solver examples/SnavelyReprojectionError.h
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
// Templated struct implementing the camera model and residual
// computation for bundle adjustment used by Noah Snavely's Bundler
// SfM system. This is also the camera model/residual for the bundle
// adjustment problems in the BAL dataset. It is templated so that we
// can use Ceres's automatic differentiation to compute analytic
// jacobians.
//
// For details see: http://phototour.cs.washington.edu/bundler/
// and http://grail.cs.washington.edu/projects/bal/

#ifndef _SNAVELY_REPROJECTION_ERROR_HPP_
#define _SNAVELY_REPROJECTION_ERROR_HPP_

#include "ceres/rotation.h"
#include "ceres/ceres.h"


// Templated pinhole camera model. The camera is parameterized using 15 parameters:
// 3 for rotation, 3 for translation, 2 for focal length (x and y), 2 for the principal point, 
// 3 for radial distortion and 2 for tangential distortion.
class SnavelyReprojectionError {
  public:
    SnavelyReprojectionError(double observed_x, double observed_y)
        :observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T* const extrinsics,
                    const T* const intrinsics,
                    const T* const point,
                    T* residuals) const {
      // camera[0,1,2] are the angle-axis rotation.
      T p[3];
      ceres::AngleAxisRotatePoint(extrinsics, point, p);
      // camera[3,4,5] are the translation.
      p[0] += extrinsics[3];
      p[1] += extrinsics[4];
      p[2] += extrinsics[5];
      T xp = p[0] / p[2];
      T yp = p[1] / p[2];
      const T& focalx = intrinsics[0];
      const T& focaly =  intrinsics[1];
      const T& centerx = intrinsics[2];
      const T& centery = intrinsics[3];
      // Apply second and fourth order radial distortion.
      const T& l1 = intrinsics[4];
      const T& l2 = intrinsics[5];
      const T& l3 = intrinsics[6];
      T r2 = xp * xp + yp * yp;
      T r4 = r2 * r2;
      T r6 = r4 * r2;
      T radial_distortion = T(1.0) + r2 * l1 + r4 * l2 + r6 * l3;
      // Apply tengential distortion
      const T& t1 = intrinsics[7];
      const T& t2 = intrinsics[8];
      T a1 = T(2.0) * xp * yp;
      T a2 = r2 + T(2.0) * xp * xp;
      T a3 = r2 + T(2.0) * yp * yp;
      T xpp = xp * radial_distortion + t1 * a1 + t2 * a2;
      T ypp = yp * radial_distortion + t1 * a3 + t2 * a1;
      // Compute final projected point position.
      T predicted_x = focalx * xpp + centerx;
      T predicted_y = focaly * ypp + centery;
      // The error is the difference between the predicted and observed position.
      residuals[0] = predicted_x - T(observed_x);
      residuals[1] = predicted_y - T(observed_y);
      return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
      return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 9, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
    }

  private:
    double observed_x;
    double observed_y;
};

#endif /* ifndef _SNAVELY_REPROJECTION_ERROR_HPP_ */
