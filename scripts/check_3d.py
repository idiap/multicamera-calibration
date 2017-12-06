#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################################################
# Multi-Camera Calibration Suite was built to help with intrinsic and extrinsic multi-camera
# calibration. It also specifically contains a bundle adjustment module to help with the joint
# calibration of the cameras.
# 
# Copyright (c) 2017 Idiap Research Institute, http://www.idiap.ch/
# Written by Salim Kayal <salim.kayal@idiap.ch>,
# 
# This file is part of Multi-Camera Calibration Suite.
# 
# Multi-Camera Calibration Suite is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
# 
# Multi-Camera Calibration Suite is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with Multi-Camera Calibration Suite. If not, see <http://www.gnu.org/licenses/>.
####################################################################################################

__author__ = "Salim Kayal"
__copyright__ = "Copyright 2016, Idiap Research Institute"
__version__ = "1.0"
__maintainer__ = "Salim Kayal"
__email__ = "salim.kayal@idiap.ch"
__status__ = "Production"
__license__ = "GPLv3"

from argparse import ArgumentParser
from glob import iglob
import numpy as np
import cv2
import json

class CalibrationInterface(object):

    @classmethod
    def __init__(self, images, calibrations):
        self.wname = '3D projection'
        self.calibrations = calibrations
        self.camera_matrices = {camera: value['cam_matrix']
                                for camera, value in calibrations.iteritems()}
        self.distortion_coefficients = {camera: value['distortion_coefficients']
                                        for camera, value in calibrations.iteritems()}
        self.projection_matrices = {camera: self.projection_matrix(value['rvec'], value['tvec'])
                                    for camera, value in calibrations.iteritems()}
        self.images = images
        self.cameras = images.keys()
        self.camera = 0
        self.image_points = {}
        self.undistorted_points = {}
        self.point3d = None
        cv2.namedWindow(self.wname)
        cv2.setMouseCallback(self.wname, self.get_points)

    @staticmethod
    def projection_matrix(rvec, tvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        projection = np.hstack((rotation_matrix, tvec))
        return projection

    @classmethod
    def get_points(self, event, x, y, flags, param):
        camera = self.cameras[self.camera]
        camera_matrix = self.camera_matrices[camera]
        distortion_coefficients = self.distortion_coefficients[camera]
        if event == cv2.EVENT_LBUTTONUP:
            pt = (x, y)
            self.image_points[camera] = pt
            pts = np.asarray(((pt,),)).astype(np.float32)
            upt = cv2.undistortPoints(pts, camera_matrix, distortion_coefficients,
                                      None, None, camera_matrix)[0, 0]
            self.undistorted_points[camera] = upt
        elif event == cv2.EVENT_RBUTTONUP:
            try:
                del self.image_points[camera]
                del self.undistorted_points[camera]
            except KeyError:
                pass
        self.refresh()

    @staticmethod
    def solve_projections(projections, points):
        matrA = np.zeros((3*len(projections), 4))
        for view, proj in enumerate(projections):
            x = points[view][0]
            y = points[view][1]
            for val in range(4):
                matrA[view*3+0, val] = x *proj[2,val] - proj[0, val]
                matrA[view*3+1, val] = y *proj[2,val] - proj[1, val]
                matrA[view*3+2, val] = x *proj[1,val] - y * proj[0, val]
        matrW = np.zeros((3*len(projections), 4))
        matrV = np.zeros((4, 4))
        _, _, v = cv2.SVDecomp(matrA)
        point3d = v[3, :]
        point3d = point3d/point3d[3]
        return (point3d[0], point3d[1], point3d[2])

    @staticmethod
    def make_projections(camera_matrices, projection_matrices):
        projections = [] 
        for camera_matrix, projection_matrix in zip(camera_matrices, projection_matrices):
            projections.append(np.matrix(np.matrix(camera_matrix)*np.matrix(projection_matrix)))
        return projections

    @classmethod
    def build_3d_points(self):
        cam_matrices = []
        projections = []
        obspoint = []
        if len(self.undistorted_points) >= 2:
            for camera, point in self.undistorted_points.iteritems():
                obspoint.append(point)
                cam_matrices.append(self.camera_matrices[camera])
                projections.append(self.projection_matrices[camera])
            projs = self.make_projections(cam_matrices, projections)
            self.point3d = self.solve_projections(projs, obspoint)
        else:
            self.point3d = None

    @classmethod
    def run(self):
        while True:
            key = chr(cv2.waitKey(30) & 0xFF)
            self.build_3d_points()
            if key == 'r':
                self.image_points = {}
                self.undistorted_points = {}
            if key == 'n':
                self.camera = (self.camera + 1) % len(self.cameras)
            if key == 'p':
                self.camera = len(self.cameras) - 1 if self.camera == 0 else self.camera - 1
            if key == 'q':
                break
            self.refresh()

    @classmethod
    def refresh(self):
        camera = self.cameras[self.camera]
        frame = np.copy(self.images[camera])
        impoint = self.image_points.get(camera)
        cal = self.calibrations[camera]
        if impoint is not None:
            cv2.circle(frame, impoint, 3, (255, 0, 0), 2)
        elif self.point3d is not None:
            opoint = np.array((self.point3d,))
            impoint, _ = cv2.projectPoints(opoint, cal['rvec'], cal['tvec'], cal['cam_matrix'],
                                           cal['distortion_coefficients'])
            impoint = tuple(impoint[0, 0].astype(np.int16))
            cv2.circle(frame, impoint, 3, (0, 0, 255), 2)
        cv2.imshow(self.wname, frame)

def parse_args():
    parser = ArgumentParser("options")
    parser.add_argument("intrinsic", help="intrinsic calibration pattern")
    parser.add_argument("extrinsic", help="extrinsic calibration pattern")
    parser.add_argument("images", help="images pattern")
    parser.add_argument("cameras", help="list of cameras")
    return parser.parse_args()

def build_calibrations(intrinsic, extrinsic):
    calibration = {}
    with open(intrinsic, 'r') as icf:
        data = json.load(icf)
        calibration['cam_matrix'] = np.array(data['intrinsic'])
        calibration['distortion_coefficients'] = \
                np.array(data['distortion_coefficients'])\
                  .reshape((len(data['distortion_coefficients']), 1))
    with open(extrinsic, 'r') as ecf:
        data = json.load(ecf)
        calibration['tvec'] = np.array(data['tvec'])\
                                         .reshape((len(data['tvec']), 1))
        calibration['rvec'] = np.array(data['rvec'])\
                                         .reshape((len(data['rvec']), 1))
    return calibration

def read_calibration(cameras, intrinsic, extrinsic):
    calibrations = {}
    for camera in cameras:
        intrinsic_path = intrinsic[0] + camera + intrinsic[1]
        extrinsic_path = extrinsic[0] + camera + extrinsic[1]
        calibrations[camera] = build_calibrations(intrinsic_path, extrinsic_path)
    return calibrations

def read_images(cameras, images):
    return {camera: cv2.imread(images[0]+camera+images[1]) for camera in cameras}

def read_data(cameras, intrinsic_path, extrinsic_path, images_path):
    calibrations = read_calibration(cameras, intrinsic_path, extrinsic_path)
    images = read_images(cameras, images_path)
    return images, calibrations

def main():
    opts = parse_args()
    cameras = opts.cameras.split(',')
    intrinsic_path = opts.intrinsic.split('{}')
    extrinsic_path = opts.extrinsic.split('{}')
    images_path = opts.images.split('{}')
    images, calibrations = read_data(cameras, intrinsic_path, extrinsic_path, images_path)
    interface = CalibrationInterface(images, calibrations)
    interface.run()

if __name__ == "__main__":
    main()
