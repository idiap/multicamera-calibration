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
from os import path
import numpy as np
import json

def parse_args():
    parser = ArgumentParser("options")
    parser.add_argument("input", help="input file")
    parser.add_argument("intrinsic", help="intrinsic calibration pattern")
    parser.add_argument("extrinsic", help="extrinsic calibration pattern")
    parser.add_argument("cameras", help="list of cameras")
    return parser.parse_args()

def write_ba_solution(input_path, cameras, intrinsic, extrinsic):
    begincam = 0
    endcam = 0
    icam = 0
    with open(input_path, 'r') as solution:
        rvec = np.ndarray((3), dtype=np.float32)
        tvec = np.ndarray((3), dtype=np.float32)
        cmat = np.zeros((3,3), dtype=np.float32)
        dist = np.ndarray((5), dtype=np.float32)
        for i, line in enumerate(solution):
            if i == 0:
                fields = line.split(' ')
                num_cameras = int(fields[0])
                num_obs = int(fields[3])
                num_fixed_obs = int(fields[4])
                begincam = (1 + num_obs + num_fixed_obs) 
                endcam = 15 * num_cameras + begincam
                continue
#ugly but so what...
            if i >= begincam and i < endcam:
                cnt = (i - begincam) % 15
                val = float(line)
                if cnt < 3:
                    rvec[cnt] = val
                elif cnt < 6:
                    tvec[cnt - 3] = val
                elif cnt == 6:
                    cmat[0,0] = val
                elif cnt == 7:
                    cmat[1,1] = val
                elif cnt == 8:
                    cmat[0,2] = val
                elif cnt == 9:
                    cmat[1,2] = val
                elif cnt < 12:
                    dist[cnt - 10] = val
                elif cnt == 12:
                    dist[4] = val
                elif cnt < 15:
                    dist[cnt - 11] = val
                if cnt == 14:
                    camera = cameras[icam]
                    cmat[2,2] = 1
                    icam += 1
                    ipath = intrinsic[0] + camera + intrinsic[1]
                    print ipath
                    epath = extrinsic[0] + camera + extrinsic[1]
                    print epath
                    with open(ipath, 'w') as ifile:
                        json.dump({'intrinsic':cmat.tolist(),
                                   'distortion_coefficients':dist.tolist()},
                                   ifile)
                    with open(epath, 'w') as efile:
                        json.dump({'rvec':rvec.tolist(), 'tvec':tvec.tolist()}, efile)
                    rvec = np.ndarray((3), dtype=np.float32)
                    tvec = np.ndarray((3), dtype=np.float32)
                    cmat = np.zeros((3,3), dtype=np.float32)
                    dist = np.ndarray((5), dtype=np.float32)
            elif i >= endcam:
                break

def main():
    opts = parse_args()
    cameras = opts.cameras.split(',')
    intrinsic = opts.intrinsic.split('{}')
    extrinsic = opts.extrinsic.split('{}')
    write_ba_solution(opts.input, cameras, intrinsic, extrinsic)

if __name__ == "__main__":
    main()
