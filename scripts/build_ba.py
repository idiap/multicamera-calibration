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
from glob import iglob
from collections import defaultdict
import numpy as np
import json
import cv2

def parse_args():
    parser = ArgumentParser("options")
    parser.add_argument("-f", "--fixed-points", help="fixed point file pattern")
    parser.add_argument("intrinsic", help="intrinsic calibration pattern")
    parser.add_argument("extrinsic", help="extrinsic calibration pattern")
    parser.add_argument("observations", help="observation files pattern")
    parser.add_argument("cameras", help="list of cameras")
    parser.add_argument("output", help="output file")
    return parser.parse_args()

def read_annotation(annotation_path):
    annotation_data = {}
    with open(annotation_path, 'r') as pf:
        field = None
        head = None
        feet = None
        offset = 0
        for i, line in enumerate(pf):
            if i == offset:
                if line.split(" ")[0] == "\n":
                    offset += 1
                    continue
            if (i-offset)%4 == 0:
                field = line
            elif (i-offset+1)%4 == 0:
                annotation_data[field] = (feet, head)
            else:
                values = line.split(' ')
                if values[0] == '1':
                    feet = (int(values[1]), int(values[2]))
                elif values[0] == '2':
                    head = (int(values[1]), int(values[2]))
    return annotation_data

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

def build_observations(annotations, camera_matrix, distortion_coefficients):
    observations = {}
    undistorted_observations = {}
    tmp_obs_id = []
    tmp_obs_value = []
    for annotation_path in iglob(annotations):
        frame = path.basename(annotation_path).split('.')[0]
        annotation_data = read_annotation(annotation_path)
        for subject_id, values in annotation_data.iteritems():
            pid = frame+subject_id
            tmp_obs_id.append(pid)
            tmp_obs_value.append(values[0])
            tmp_obs_value.append(values[1])
    tmp_obs_value = np.array([tmp_obs_value]).astype(np.float32)
    tmp_undistorted_values = cv2.undistortPoints(tmp_obs_value, camera_matrix,
                                                 distortion_coefficients, None, None,
                                                 camera_matrix) \
                             if len(tmp_obs_value.shape) == 3 else \
                             np.array([])
    for i, pid in enumerate(tmp_obs_id):
        observations[pid] = (tmp_obs_value[0, 2*i], tmp_obs_value[0, 2*i+1])
        undistorted_observations[pid] = (tmp_undistorted_values[0, 2*i],
                                         tmp_undistorted_values[0, 2*i+1])
    return observations, undistorted_observations

def projection_matrix(rvec, tvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    projection = np.hstack((rotation_matrix, tvec))
    return projection

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

def make_linear_system(camera_matrices, projection_matrices, points):
    projection = None
    u_v = None
    for point, camera_matrix, projection_matrix \
            in zip(points, camera_matrices, projection_matrices):
        if projection is None:
            projection = np.matrix(np.matrix(camera_matrix)*np.matrix(projection_matrix))
            u_v = np.matrix([point[0], point[1], 1]).T
        else:
            projection = np.matrix(np.vstack((projection, 
                np.matrix(np.matrix(camera_matrix)*np.matrix(projection_matrix)))))
            u_v = np.matrix(np.vstack((u_v, np.matrix([point[0], point[1], 1]).T)))
    return projection, u_v

def make_projections(camera_matrices, projection_matrices):
    projections = [] 
    for camera_matrix, projection_matrix in zip(camera_matrices, projection_matrices):
            projections.append(np.matrix(np.matrix(camera_matrix)*np.matrix(projection_matrix)))
    return projections

def get_3d_foot(projection, u_v):
    p = np.matrix(np.hstack((projection[:, 0:2], projection[:, 3])))
    x_y = np.linalg.solve(p, u_v)
    x_y = np.squeeze(np.asarray(x_y))
    return  (x_y[0]/x_y[2], x_y[1]/x_y[2], 0)

def get_3d_head_from_foot(projection, u_v, foot3d):
    b = u_v - foot3d[0] * projection[:, 0] - foot3d[1] * projection[:, 1] - projection[:, 3]
    z, _, _, _ = np.linalg.lstsq(np.matrix(projection[:, 2]), b)
    z = np.squeeze(np.asarray(z))
    return (foot3d[0], foot3d[1], z[()])

def build_3d_points(observations, camera_matrices, projection_matrices, outliers=None):
    points = {}
    for pid, obs in observations.iteritems():
        feet = []
        heads = []
        cam_matrices_f = []
        cam_matrices_h = []
        projections_f = []
        projections_h = []
        obspoint = [feet, heads]
        cam_matrix = [cam_matrices_f, cam_matrices_h]
        projections = [projections_f, projections_h]
        for camera, pts in obs.iteritems():   
            outlier = None if outliers is None else outliers.get(pid)
            filterlist = None if outlier is None else outlier.get(camera)
            for i, pt in enumerate(pts):
                if pt is not None and (filterlist is None or i not in filterlist):
                    obspoint[i].append(pts[i])
                    cam_matrix[i].append(camera_matrices[camera])
                    projections[i].append(projection_matrices[camera])
        projection_f, u_v_f = make_linear_system(cam_matrices_f, projections_f, feet)
        projs_f = make_projections(cam_matrices_f, projections_f)
        projection_h, u_v_h = make_linear_system(cam_matrices_h, projections_h, heads)
        projs_h = make_projections(cam_matrices_h, projections_h)
        head3d = None
        foot3d = None
        if len(feet) == 1:
            print "point " + pid[:-1] + " only in one camera : " + camera
            foot3d = get_3d_foot(projection_f, u_v_f)
            if len(heads) == 1:
                head3d = get_3d_head_from_foot(projection_h, u_v_h, foot3d)
        elif len(feet) > 1:
            foot3d = solve_projections(projs_f, feet)
        if len(heads) > 1:
            head3d = solve_projections(projs_h, heads)
        points[pid] = (foot3d, head3d)
    return points

def check_reprojection_error(cameras, calibrations, observations, points):
    total_diffs = None
    fltr = defaultdict(lambda:defaultdict(list))
    for camera in cameras:
        calibration = calibrations[camera]
        outliers = []
        impoints = []
        objpoints = []
        pids = []
        for pid, point in points.iteritems():
            impoint = observations[pid].get(camera)
            if impoint is not None:
                for i, (ipt, opt) in enumerate(zip(impoint, point)):
                    if None not in [ipt, opt]:
                        pids.append((pid, i))
                        impoints.append(ipt)
                        objpoints.append(opt)
        if len(objpoints) > 0:
            impoints = np.array(impoints)
            objpoints = np.array(objpoints)
            rep_impoints, _ = cv2.projectPoints(objpoints, calibration['rvec'], calibration['tvec'],
                                                calibration['cam_matrix'],
                                                calibration['distortion_coefficients'])
            rep_impoints = np.squeeze(rep_impoints)
            diff = impoints - rep_impoints
            euclidean_dist = np.sqrt(diff[:, 0]*diff[:,0] + diff[:,1]*diff[:,1])
            total_diffs = euclidean_dist if total_diffs is None else np.hstack((total_diffs, euclidean_dist))
            mean_reprojection_error = np.mean(euclidean_dist)
            indices = np.arange(euclidean_dist.shape[0])[euclidean_dist > 5*np.mean(euclidean_dist)]
            for index in indices:
                outliers.append(pids[index])
                fltr[pids[index][0]][camera].append(pids[index][1])
            print camera + " mean reprojection error : ", mean_reprojection_error, 
            print "median :", np.median(euclidean_dist),
            mask = np.ones(euclidean_dist.shape).astype(np.bool)
            mask[indices] = False
            print "filtered mean :", np.mean(euclidean_dist[mask]),
            print "filtered median :", np.median(euclidean_dist[mask])
        else:
            print camera, ' empty'
    print "overall mean reprojection error :", np.mean(total_diffs), "median :", np.median(total_diffs)
    return fltr

def filter_data(observations, outliers):
    new_obs = defaultdict(dict)
    for pid, obs in observations.iteritems():
        for camera, pts in obs.iteritems():   
            outlier = None if outliers is None else outliers.get(pid)
            filterlist = None if outlier is None else outlier.get(camera)
            foot = None
            head = None
            if filterlist is None or 0 not in filterlist:
                foot = pts[0]
            if filterlist is None or 1 not in filterlist:
                head = pts[1]
            new_obs[pid][camera] = (foot, head)
    return new_obs

def read_fixed_points(cameras, fixed_points_pattern):
    fixed_points = defaultdict(dict)
    if fixed_points_pattern is not None:
        for camera in cameras:
            with open(fixed_points_pattern[0] + camera + fixed_points_pattern[1], 'r') as fpf:
                data = json.load(fpf)
                for object_point, image_point in zip(data['object_points'], data['image_points']):
                    fixed_points[tuple(object_point)][camera] = tuple(image_point)
    return fixed_points

def read_calibration(cameras, intrinsic, extrinsic):
    calibrations = {}
    for camera in cameras:
        intrinsic_path = intrinsic[0] + camera + intrinsic[1]
        extrinsic_path = extrinsic[0] + camera + extrinsic[1]
        calibrations[camera] = build_calibrations(intrinsic_path, extrinsic_path)
    return calibrations

def read_observations(cameras, calibrations, annotations):
    observations = defaultdict(dict)
    undistorted_observations = defaultdict(dict)
    for camera in cameras:
        annotations_path = annotations[0] + camera + annotations[1] 
        tmp_observations, tmp_undistorted_observations = \
                build_observations(annotations_path, calibrations[camera]['cam_matrix'],
                                   calibrations[camera]['distortion_coefficients'])
        for pid, value in tmp_observations.iteritems():
            observations[pid][camera] = value
            undistorted_observations[pid][camera] = tmp_undistorted_observations[pid]
    return observations, undistorted_observations

def read_data(cameras, intrinsic, extrinsic, annotations, fixed_points_pattern):
    fixed_points = read_fixed_points(cameras, fixed_points_pattern)
    calibrations = read_calibration(cameras, intrinsic, extrinsic)
    observations, undistorted_observations = read_observations(cameras, calibrations, annotations)
    camera_matrices = {camera: value['cam_matrix'] for camera, value in calibrations.iteritems()}
    projection_matrices = {camera: projection_matrix(value['rvec'], value['tvec'])
                           for camera, value in calibrations.iteritems()}
    points = build_3d_points(undistorted_observations, camera_matrices, projection_matrices)
    outliers = check_reprojection_error(cameras, calibrations, observations, points)
    observations = filter_data(observations, outliers)
    undistorted_observations = filter_data(undistorted_observations, outliers)
    points = build_3d_points(undistorted_observations, camera_matrices, projection_matrices, outliers)
    check_reprojection_error(cameras, calibrations, observations, points)
    return calibrations, observations, points, fixed_points

def write_ba_problem(cameras, observations, calibrations, points, fixed_points, output_path):
    pids = [(key, i) for key, value in points.iteritems()
            for i, val in enumerate(value)
            if val is not None]
    fpids = fixed_points.keys()
    n_obs = sum(1 for key, idx in pids
                for value in observations[key].itervalues()
                if value[idx] is not None)
    n_fobs = sum(1 for projs in fixed_points.itervalues() for proj in projs.itervalues())
    with open(output_path, 'w') as output:
        output.write("{} {} {} {} {}\n".format(len(cameras), len(pids), len(fpids), n_obs, n_fobs))
        for i_cam, camera in enumerate(cameras):
            for i_pid, pid in enumerate(pids):
                cam_obs = observations[pid[0]].get(camera)
                if cam_obs is not None:
                    obs = cam_obs[pid[1]]
                    if obs is not None:
                        output.write("{} {} {} {}\n"\
                              .format(i_cam, i_pid, obs[0], obs[1]))
        for i_cam, camera in enumerate(cameras):
            for i_pid, fpid in enumerate(fpids):
                cam_obs = fixed_points[fpid].get(camera)
                if cam_obs is not None:
                    if cam_obs is not None:
                        output.write("{} {} {} {}\n"\
                              .format(i_cam, i_pid, cam_obs[0], cam_obs[1]))
        for camera in cameras:
            cal = calibrations[camera]
            for val in cal['rvec'].squeeze():
                output.write("{}\n".format(val))
            for val in cal['tvec'].squeeze():
                output.write("{}\n".format(val))
            cam_matrix = cal['cam_matrix'].squeeze()
            distortion_coefficients = cal['distortion_coefficients'].squeeze()
            output.write("{}\n".format(cam_matrix[0, 0]))
            output.write("{}\n".format(cam_matrix[1, 1]))
            output.write("{}\n".format(cam_matrix[0, 2]))
            output.write("{}\n".format(cam_matrix[1, 2]))
            output.write("{}\n".format(distortion_coefficients[0]))
            output.write("{}\n".format(distortion_coefficients[1]))
            output.write("{}\n".format(distortion_coefficients[4]))
            output.write("{}\n".format(distortion_coefficients[2]))
            output.write("{}\n".format(distortion_coefficients[3]))
        for pid in pids:
            point = points[pid[0]][pid[1]]
            for val in point:
                output.write("{}\n".format(val))
        for fpid in fpids:
            for val in fpid:
                output.write("{}\n".format(val))

def main():
    opts = parse_args()
    cameras = opts.cameras.split(',')
    fixed_points_pattern = None if opts.fixed_points is None else opts.fixed_points.split('{}')
    intrinsic = opts.intrinsic.split('{}')
    extrinsic = opts.extrinsic.split('{}')
    annotations = opts.observations.split('{}')
    calibrations, observations, points, fixed_points = read_data(cameras, intrinsic, extrinsic,
                                                                 annotations, fixed_points_pattern)
    write_ba_problem(cameras, observations, calibrations, points, fixed_points, opts.output)

if __name__ == "__main__":
    main()
