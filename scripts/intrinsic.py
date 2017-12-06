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
from shutil import copy
from argparse import ArgumentParser
from glob import glob
import numpy as np
import cv2
import json

def parse():
    """Parse command line
    :returns: options
    """
    parser = ArgumentParser()
    parser.add_argument('-n', '--num', default=20, type=int, help='number to subsample')
    parser.add_argument('-W', '--width', default=4, help='board width')
    parser.add_argument('-H', '--height', default=11, help='board height')
    parser.add_argument('-p', '--pattern', choices=['chessboard', 'circles', 'asymmetric_circles'],
                        default='asymmetric_circles', help='pattern to discover')
    parser.add_argument('-o', '--full_output', help='output folder for all images')
    parser.add_argument('-s', '--select_output', help='output folder for selected images')
    parser.add_argument('input', help='input folder with images')
    parser.add_argument('output', help='output folder for calibration')
    return parser.parse_args()

def filter_images(input_pattern, pattern, width, height):
# termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Arrays to store object points and image points from all the images.
    imgpoints = [] # 2d points in image plane.
    images = glob(input_pattern)
    to_del = []
    for i, fname in enumerate(images):
        #print fname
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        if pattern=='chessboard':
            ret, interest_points = cv2.findChessboardCorners(gray, (width, height), None)
            if ret == True:
                cv2.cornerSubPix(gray, interest_points, (11, 11), (-1, -1), criteria)
        else:
            flag = cv2.CALIB_CB_SYMMETRIC_GRID if pattern=='circles' else \
                   cv2.CALIB_CB_ASYMMETRIC_GRID
            ret, interest_points = cv2.findCirclesGrid(gray, (width, height), None, flag)
        # If found, add object points, image points (after refining them)
        if ret == True:
            cv2.cornerSubPix(gray, interest_points, (11, 11), (-1, -1), criteria)
            imgpoints.append(interest_points)
        else:
            to_del.append(i)
    while len(to_del) > 0:
        last_item = to_del[-1]
        del images[last_item]
        del to_del[-1]
    return images, imgpoints

def mean_dist(pts1, pts2):
    tot_dist = 0
    for pt1, pt2 in zip(pts1, pts2):
        tot_dist += ((pt1[0][0] - pt2[0][0])**2 + (pt1[0][1] - pt2[0][1])**2)**0.5
    return tot_dist/len(pts1)

def select_images(image_points, number_to_select):
    imlist = [0]
    imranks = []
    for _ in range(number_to_select - 1):
        imgpoint = image_points[imlist[-1]]
        imdist = []
        for i in range(len(image_points)):
            imdist.append([i, mean_dist(image_points[i], imgpoint)])
        imdist.sort(key=lambda x: x[1])
        imrank = [(idx, rank) for rank, (idx, _) in enumerate(imdist)]
        imrank.sort(key=lambda x: x[0])
        imrank = [rank for _, rank in imrank]
        imranks.append(imrank)
        minranks = {i:len(image_points) for i in range(len(image_points)) if i not in imlist}
        for imrank in imranks:
            for idx, rank in minranks.iteritems():
                if imrank[idx] < rank:
                    minranks[idx] = imrank[idx]
        max_min_idx = max(minranks, key=minranks.get)
        imlist.append(max_min_idx)
    return imlist

def main():
    opts = parse()
    images, image_points = filter_images(opts.input, opts.pattern, opts.width, opts.height)
    if opts.full_output is not None:
        for image in images:
            copy(image, path.join(opts.full_output, path.basename(image)))
    selected_indices = select_images(image_points, opts.num)
    if opts.select_output is not None:
        for index in selected_indices:
            image = images[index]
            copy(image, path.join(opts.select_output, path.basename(image)))
#compute calibration
    object_points = np.zeros((opts.width*opts.height, 3), np.float32)
    if opts.pattern == 'asymmetric_circles':
        grid = np.mgrid[0:opts.width * 2:2, 0:opts.height]
        grid[0, :, 1::2] += 1
        object_points[:, :2] = grid.T.reshape(-1, 2)
    else:
        object_points[:, :2] = np.mgrid[0:opts.width, 0:opts.height].T.reshape(-1, 2)
    opoints = [object_points for _ in range(len(selected_indices))]
    ipoints = [image_points[idx] for idx in selected_indices]
    image_resolution = cv2.imread(images[0]).shape[:-1][::-1]
    print image_resolution
    _, cam_matrix, distortion, rotation_vectors, translation_vectors = \
            cv2.calibrateCamera(opoints, ipoints, image_resolution)
#save calibration
    with open(opts.output, 'w') as out_file:
        json.dump({'intrinsic':cam_matrix.tolist(),
                   'distortion_coefficients':distortion.squeeze().tolist()},
                  out_file)
# compute retroprojection error
    tot_error = 0
    for i in xrange(len(opoints)):
        ipoints2, _ = cv2.projectPoints(opoints[i], rotation_vectors[i], translation_vectors[i],
                                          cam_matrix, distortion)
        error = cv2.norm(ipoints[i],ipoints2, cv2.NORM_L2)/len(ipoints2)
        tot_error += error
    print "total error :", tot_error, "mean error :", tot_error/len(opoints)
#show results
    pos = 0
    key = None
    while key != ord('q'):
        key = None
        image = cv2.imread(images[pos])
        undistorted = cv2.undistort(image, cam_matrix, distortion, None)
        cv2.imshow('img', undistorted)
        print 'next (n), previous (p) or quit (q)'
        while key not in [ord('q'), ord('n'), ord('p')]:
            key = cv2.waitKey(10)
        pos = ((pos + 1) % len(selected_indices)) \
                if key == ord('n') else \
                (len(selected_indices) - 1 if pos == 0 else pos - 1)

if __name__ == "__main__":
    main()
