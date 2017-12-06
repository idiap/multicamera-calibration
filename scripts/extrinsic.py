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

from os import path
from argparse import ArgumentParser
import numpy as np
import json
import cv2

class CalibrationInterface(object):
    GET_POINT = 0
    GET_VALUES = 1

    @classmethod
    def __init__(self, camera_matrix, distortion_coefficients, image, object_points=None,
                 image_points=None):
        if None in [object_points, image_points]:
            assert all(points is None for points in [object_points, image_points])
        else:
            assert len(object_points) == len(image_points)
        self.wname = 'calibration interface'
        self.camera_matrix = camera_matrix
        self.distortion_coefficients = distortion_coefficients
        self.subtext = ''
        self.entry = ''
        self.entry_error = False
        self.state = self.GET_POINT
        self.image = image
        self.image_points = [] if image_points is None else image_points
        self.object_points = [] if object_points is None else object_points
        cv2.namedWindow(self.wname)
        cv2.setMouseCallback(self.wname, self.get_points)
        self.refresh()

    @classmethod
    def solve(self):
        assert len(self.object_points) == len(self.image_points)
        object_points = np.array(self.object_points)\
                          .reshape((len(self.object_points), 3, 1)).astype(np.float32)
        image_points = np.array(self.image_points)\
                         .reshape((len(self.object_points), 2, 1)).astype(np.float32)
        retval, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix,
                                          self.distortion_coefficients)
        if retval:
            return rvec, tvec

    @classmethod
    def get_points(self, event, x, y, flags, param):
        if self.state == self.GET_POINT:
            if event == cv2.EVENT_LBUTTONUP:
                self.image_points.append((x, y))
                self.state = self.GET_VALUES
            elif event == cv2.EVENT_RBUTTONUP:
                self.image_points.pop()
                self.object_points.pop()
        self.refresh()

    @classmethod
    def run(self):
        opoint = []
        def manage_key(message, key, opoint):
            self.subtext = 'PLEASE ENTER A NUMBER. ' + message if self.entry_error else message
            if key == chr(8):
                self.entry = self.entry[:-1]
            elif key == chr(10):
                try:
                    opoint.append(float(self.entry))
                    self.entry_error = False
                    self.subtext = ''
                    self.entry = ''
                except ValueError:
                    self.entry_error = True
                    self.entry = ''
            elif key == chr(27):
                opoint = []
                self.entry = ''
                self.subtext = ''
                self.state = self.GET_POINT
                self.image_points.pop()
            elif key == chr(255):
                pass
            else:
                self.entry += key
            return opoint
        while True:
            key = chr(cv2.waitKey(30) & 0xFF)
            if self.state == self.GET_VALUES:
                if len(opoint) == 0:
                    message = 'please enter the real world x in cm'
                    opoint = manage_key(message, key, opoint)
                elif len(opoint) == 1:
                    message = "x value is %f. please enter the real world y in cm" % opoint[0]
                    opoint = manage_key(message, key, opoint)
                else:
                    opoint.append(0.)
                    self.object_points.append(tuple(opoint))
                    opoint = []
                    self.state = self.GET_POINT
            if self.state == self.GET_POINT and key in ['s', 'q']:
                break
            self.refresh()
        return None if key == 'q' else self.solve()

    @classmethod
    def refresh(self):
        frame = np.copy(self.image)
        cv2.putText(frame, self.subtext, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1,
                    (0,0,255))
        cv2.putText(frame, self.entry, (10, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (0,0,255))
        for num, (ipoint, opoint) in enumerate(zip(self.image_points, self.object_points)):
            cv2.circle(frame, ipoint, 3, (255,0,0), 2)
            text = '{}: {}, {}'.format(num, opoint[0], opoint[1])
            cv2.putText(frame, text, (ipoint[0] + 5, ipoint[1] + 5), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        1, (255,0,0))
        cv2.imshow(self.wname, frame)

def parse():
    """Parse command line
    :returns: options
    """
    parser = ArgumentParser()
    parser.add_argument('-p', '--points', help='input points for calibration')
    parser.add_argument('-o', '--output_points', help='output points for future calibration')
    parser.add_argument('intrinsic', help='intrinsic calibration file')
    parser.add_argument('input', help='input image file')
    parser.add_argument('output', help='output extrinsic calibration file')
    return parser.parse_args()

def main():
    """main function"""
    opts = parse()
    image = cv2.imread(opts.input)
    with open(opts.intrinsic, 'r') as cfile:
        calibration = json.load(cfile)
        cam_matrix = np.array(calibration['intrinsic'])
        distortion_coefficients = np.array(calibration['distortion_coefficients'])\
                                    .reshape((len(calibration['distortion_coefficients']), 1))
    if opts.points is not None:
        with open(opts.points, 'r') as pfile:
            points = json.load(pfile)
            object_points = [tuple(point) for point in points['object_points']]
            image_points = [tuple(point) for point in points['image_points']]
            interface = CalibrationInterface(cam_matrix, distortion_coefficients, image,
                                             object_points, image_points)
    else:
        interface = CalibrationInterface(cam_matrix, distortion_coefficients, image)
    results = interface.run()
    if results is not None:
        with open(opts.output, 'w') as ofile:
            json.dump(dict(rvec=results[0].tolist(), tvec=results[1].tolist()), ofile)
    if opts.output_points is not None:
        with open(opts.output_points, 'w') as opfile:
            json.dump(dict(image_points=interface.image_points,
                           object_points=interface.object_points), opfile)

if __name__ == "__main__":
    main()
