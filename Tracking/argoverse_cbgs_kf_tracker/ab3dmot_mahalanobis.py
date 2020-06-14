#!/usr/bin/env python3

from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import pdb
from numba import jit
# from sklearn.utils.linear_assignment_ import linear_assignment
from scipy.spatial import ConvexHull
from scipy.optimize import linear_sum_assignment
import sys
import time

from transform_utils import convert_3dbox_to_8corner
from iou_utils import compute_iou_2d_bboxes


@jit
def poly_area(x, y):
    return 0.5*np.abs(np.dot(x, np.roll(y, 1))-np.dot(y, np.roll(x, 1)))


@jit
def box3d_vol(corners):
    ''' corners: (8,3) no assumption on axis direction '''
    a = np.sqrt(np.sum((corners[0, :] - corners[1, :])**2))
    b = np.sqrt(np.sum((corners[1, :] - corners[2, :])**2))
    c = np.sqrt(np.sum((corners[0, :] - corners[4, :])**2))
    return a*b*c


@jit
def convex_hull_intersection(p1, p2):
    """ Compute area of two convex hull's intersection area.
        p1,p2 are a list of (x,y) tuples of hull vertices.
        return a list of (x,y) for the intersection and its volume
    """
    inter_p = polygon_clip(p1, p2)
    if inter_p is not None:
        hull_inter = ConvexHull(inter_p)
        return inter_p, hull_inter.volume
    else:
        return None, 0.0


def polygon_clip(subjectPolygon, clipPolygon):
    """ Clip a polygon with another polygon.
    Args:
      subjectPolygon: a list of (x,y) 2d points, any polygon.
      clipPolygon: a list of (x,y) 2d points, has to be *convex*
    Note:
      **points have to be counter-clockwise ordered**

    Return:
      a list of (x,y) vertex point for the intersection polygon.
    """
    def inside(p):
        return(cp2[0]-cp1[0])*(p[1]-cp1[1]) > (cp2[1]-cp1[1])*(p[0]-cp1[0])

    def computeIntersection():
        dc = [cp1[0] - cp2[0], cp1[1] - cp2[1]]
        dp = [s[0] - e[0], s[1] - e[1]]
        n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
        n2 = s[0] * e[1] - s[1] * e[0]
        n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
        return [(n1*dp[0] - n2*dc[0]) * n3, (n1*dp[1] - n2*dc[1]) * n3]

    outputList = subjectPolygon
    cp1 = clipPolygon[-1]

    for clipVertex in clipPolygon:
        cp2 = clipVertex
        inputList = outputList
        outputList = []
        s = inputList[-1]

        for subjectVertex in inputList:
            e = subjectVertex
            if inside(e):
                if not inside(s):
                    outputList.append(computeIntersection())
                outputList.append(e)
            elif inside(s):
                outputList.append(computeIntersection())
            s = e
        cp1 = cp2
        if len(outputList) == 0:
            return None
    return(outputList)


def iou3d(corners1, corners2):
    ''' Compute 3D bounding box IoU.

    Input:
        corners1: numpy array (8,3), assume up direction is negative Y
        corners2: numpy array (8,3), assume up direction is negative Y
    Output:
        iou: 3D bounding box IoU
        iou_2d: bird's eye view 2D bounding box IoU

    '''
    # corner points are in counter clockwise order
    rect1 = [(corners1[i, 0], corners1[i, 2]) for i in range(3, -1, -1)]
    rect2 = [(corners2[i, 0], corners2[i, 2]) for i in range(3, -1, -1)]
    area1 = poly_area(np.array(rect1)[:, 0], np.array(rect1)[:, 1])
    area2 = poly_area(np.array(rect2)[:, 0], np.array(rect2)[:, 1])
    inter, inter_area = convex_hull_intersection(rect1, rect2)
    iou_2d = inter_area/(area1+area2-inter_area)
    ymax = min(corners1[0, 1], corners2[0, 1])
    ymin = max(corners1[4, 1], corners2[4, 1])
    inter_vol = inter_area * max(0.0, ymax-ymin)
    vol1 = box3d_vol(corners1)
    vol2 = box3d_vol(corners2)
    iou = inter_vol / (vol1 + vol2 - inter_vol)
    return iou, iou_2d


@jit
def roty(t):
    ''' Rotation about the y-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0,  s],
                     [0,  1,  0],
                     [-s, 0,  c]])


@jit
def rotz(t):
    ''' Rotation about the z-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])


class KalmanBoxTracker(object):
    """
    This class represents the internel state of individual tracked objects observed as bbox.
    """
    count = 0

    def __init__(self, bbox3D, info):
        """
        Initialises a tracker using initial bounding box.
        """
        # define constant velocity model
        self.kf = KalmanFilter(dim_x=10, dim_z=7)
        self.kf.F = np.array([[1, 0, 0, 0, 0, 0, 0, 1, 0, 0],      # state transition matrix
                              [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

        self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],      # measurement function,
                              [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]])

        # with angular velocity
        # self.kf = KalmanFilter(dim_x=11, dim_z=7)
        # self.kf.F = np.array([[1,0,0,0,0,0,0,1,0,0,0],      # state transition matrix
        #                       [0,1,0,0,0,0,0,0,1,0,0],
        #                       [0,0,1,0,0,0,0,0,0,1,0],
        #                       [0,0,0,1,0,0,0,0,0,0,1],
        #                       [0,0,0,0,1,0,0,0,0,0,0],
        #                       [0,0,0,0,0,1,0,0,0,0,0],
        #                       [0,0,0,0,0,0,1,0,0,0,0],
        #                       [0,0,0,0,0,0,0,1,0,0,0],
        #                       [0,0,0,0,0,0,0,0,1,0,0],
        #                       [0,0,0,0,0,0,0,0,0,1,0],
        #                       [0,0,0,0,0,0,0,0,0,0,1]])

        # self.kf.H = np.array([[1,0,0,0,0,0,0,0,0,0,0],      # measurement function,
        #                       [0,1,0,0,0,0,0,0,0,0,0],
        #                       [0,0,1,0,0,0,0,0,0,0,0],
        #                       [0,0,0,1,0,0,0,0,0,0,0],
        #                       [0,0,0,0,1,0,0,0,0,0,0],
        #                       [0,0,0,0,0,1,0,0,0,0,0],
        #                       [0,0,0,0,0,0,1,0,0,0,0]])

        # self.kf.R[0:,0:] *= 10.   # measurement uncertainty
        self.kf.P[7:, 7:] *= 1000.  # state uncertainty, give high uncertainty to the unobservable initial velocities, covariance matrix
        self.kf.P *= 10.

        # self.kf.Q[-1,-1] *= 0.01    # process uncertainty
        self.kf.Q[7:, 7:] *= 0.01
        self.kf.x[:7] = bbox3D.reshape((7, 1))

        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.history = []
        self.hits = 1           # number of total hits including the first detection
        self.hit_streak = 1     # number of continuing hit considering the first detection
        self.first_continuing_hit = 1
        self.still_first = True
        self.age = 0
        self.info = info        # other info

    def update(self, bbox3D, info):
        """ 
        Updates the state vector with observed bbox.
        """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1          # number of continuing hit
        if self.still_first:
            self.first_continuing_hit += 1      # number of continuing hit in the fist time

        # orientation correction
        if self.kf.x[3] >= np.pi:
            self.kf.x[3] -= np.pi * 2    # make the theta still in the range
        if self.kf.x[3] < -np.pi:
            self.kf.x[3] += np.pi * 2

        new_theta = bbox3D[3]
        if new_theta >= np.pi:
            new_theta -= np.pi * 2    # make the theta still in the range
        if new_theta < -np.pi:
            new_theta += np.pi * 2
        bbox3D[3] = new_theta

        predicted_theta = self.kf.x[3]
        # if the angle of two theta is not acute angle
        if abs(new_theta - predicted_theta) > np.pi / 2.0 and abs(new_theta - predicted_theta) < np.pi * 3 / 2.0:
            self.kf.x[3] += np.pi
            if self.kf.x[3] > np.pi:
                # make the theta still in the range
                self.kf.x[3] -= np.pi * 2
            if self.kf.x[3] < -np.pi:
                self.kf.x[3] += np.pi * 2

        # now the angle is acute: < 90 or > 270, convert the case of > 270 to < 90
        if abs(new_theta - self.kf.x[3]) >= np.pi * 3 / 2.0:
            if new_theta > 0:
                self.kf.x[3] += np.pi * 2
            else:
                self.kf.x[3] -= np.pi * 2

        #########################

        self.kf.update(bbox3D)

        if self.kf.x[3] >= np.pi:
            self.kf.x[3] -= np.pi * 2    # make the theta still in the range
        if self.kf.x[3] < -np.pi:
            self.kf.x[3] += np.pi * 2
        self.info = info

    def predict(self):
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """
        self.kf.predict()
        if self.kf.x[3] >= np.pi:
            self.kf.x[3] -= np.pi * 2
        if self.kf.x[3] < -np.pi:
            self.kf.x[3] += np.pi * 2

        self.age += 1
        if(self.time_since_update > 0):
            self.hit_streak = 0
            self.still_first = False
        self.time_since_update += 1
        self.history.append(self.kf.x)
        return self.history[-1]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        return self.kf.x[:7].reshape((7, ))


def angle_in_range(angle):
    '''
    Input angle: -2pi ~ 2pi
    Output angle: -pi ~ pi
    '''
    if angle > np.pi:
        angle -= 2 * np.pi
    if angle < -np.pi:
        angle += 2 * np.pi
    return angle


def diff_orientation_correction(det, trk):
    '''
    return the angle diff = det - trk
    if angle diff > 90 or < -90, rotate trk and update the angle diff
    '''
    diff = det - trk
    diff = angle_in_range(diff)
    if diff > np.pi / 2:
        diff -= np.pi
    if diff < -np.pi / 2:
        diff += np.pi
    diff = angle_in_range(diff)
    return diff


def greedy_match(distance_matrix):
    '''
    Find the one-to-one matching using greedy allgorithm choosing small distance
    distance_matrix: (num_detections, num_tracks)
    '''
    matched_indices = []

    num_detections, num_tracks = distance_matrix.shape
    distance_1d = distance_matrix.reshape(-1)
    index_1d = np.argsort(distance_1d)
    index_2d = np.stack([index_1d // num_tracks, index_1d %
                         num_tracks], axis=1)
    detection_id_matches_to_tracking_id = [-1] * num_detections
    tracking_id_matches_to_detection_id = [-1] * num_tracks
    for sort_i in range(index_2d.shape[0]):
        detection_id = int(index_2d[sort_i][0])
        tracking_id = int(index_2d[sort_i][1])
        if tracking_id_matches_to_detection_id[tracking_id] == -1 and detection_id_matches_to_tracking_id[detection_id] == -1:
            tracking_id_matches_to_detection_id[tracking_id] = detection_id
            detection_id_matches_to_tracking_id[detection_id] = tracking_id
            matched_indices.append([detection_id, tracking_id])

    matched_indices = np.array(matched_indices)
    return matched_indices


def associate_detections_to_trackers(detections, trackers, iou_threshold=0.1,
                                     use_mahalanobis=False, dets=None, trks=None, trks_S=None, mahalanobis_threshold=0.1, print_debug=False, match_algorithm='greedy'):
    """
    Assigns detections to tracked object (both represented as bounding boxes)

    detections:  N x 8 x 3
    trackers:    M x 8 x 3

    dets: N x 7
    trks: M x 7
    trks_S: N x 7 x 7

    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if(len(trackers) == 0):
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 8, 3), dtype=int)
    iou_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)
    distance_matrix = np.zeros(
        (len(detections), len(trackers)), dtype=np.float32)

    if use_mahalanobis:
        assert(dets is not None)
        assert(trks is not None)
        assert(trks_S is not None)

    if use_mahalanobis and print_debug:
        print('dets.shape: ', dets.shape)
        print('dets: ', dets)
        print('trks.shape: ', trks.shape)
        print('trks: ', trks)
        print('trks_S.shape: ', trks_S.shape)
        print('trks_S: ', trks_S)
        S_inv = [np.linalg.inv(S_tmp) for S_tmp in trks_S]  # 7 x 7
        S_inv_diag = [S_inv_tmp.diagonal() for S_inv_tmp in S_inv]  # 7
        print('S_inv_diag: ', S_inv_diag)

    for d, det in enumerate(detections):
        for t, trk in enumerate(trackers):
            if use_mahalanobis:
                S_inv = np.linalg.inv(trks_S[t])  # 7 x 7
                diff = np.expand_dims(dets[d] - trks[t], axis=1)  # 7 x 1
                # manual reversed angle by 180 when diff > 90 or < -90 degree
                corrected_angle_diff = diff_orientation_correction(
                    dets[d][3], trks[t][3])
                diff[3] = corrected_angle_diff
                distance_matrix[d, t] = np.sqrt(
                    np.matmul(np.matmul(diff.T, S_inv), diff)[0][0])
            else:
                # det: 8 x 3, trk: 8 x 3
                iou_matrix[d, t] = iou3d(det, trk)[0]
                distance_matrix = -iou_matrix

    if match_algorithm == 'greedy':
        matched_indices = greedy_match(distance_matrix)
    elif match_algorithm == 'pre_threshold':
        if use_mahalanobis:
            to_max_mask = distance_matrix > mahalanobis_threshold
            distance_matrix[to_max_mask] = mahalanobis_threshold + 1
        else:
            to_max_mask = iou_matrix < iou_threshold
            distance_matrix[to_max_mask] = 0
            iou_matrix[to_max_mask] = 0

        indices = linear_sum_assignment(distance_matrix)
        indices = np.asarray(indices)
        matched_indices = np.transpose(indices)  # houngarian algorithm
    else:
        indices = linear_sum_assignment(distance_matrix)
        indices = np.asarray(indices)
        matched_indices = np.transpose(indices)  # houngarian algorithm

    if print_debug:
        print('distance_matrix.shape: ', distance_matrix.shape)
        print('distance_matrix: ', distance_matrix)
        print('matched_indices: ', matched_indices)

    unmatched_detections = []
    for d, det in enumerate(detections):
        if(d not in matched_indices[:, 0]):
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if len(matched_indices) == 0 or (t not in matched_indices[:, 1]):
            unmatched_trackers.append(t)

    # filter out matched with low IOU
    matches = []
    for m in matched_indices:
        match = True
        if use_mahalanobis:
            if distance_matrix[m[0], m[1]] > mahalanobis_threshold:
                match = False
        else:
            if(iou_matrix[m[0], m[1]] < iou_threshold):
                match = False
        if not match:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    if(len(matches) == 0):
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    if print_debug:
        print('matches: ', matches)
        print('unmatched_detections: ', unmatched_detections)
        print('unmatched_trackers: ', unmatched_trackers)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class AB3DMOT(object):
    # max age will preserve the bbox does not appear no more than 2 frames, interpolate the detection
    def __init__(self, max_age=2, min_hits=3):
        # def __init__(self,max_age=3,min_hits=3):        # ablation study
        # def __init__(self,max_age=1,min_hits=3):
        # def __init__(self,max_age=2,min_hits=1):
        # def __init__(self,max_age=2,min_hits=5):
        """              
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = []
        self.frame_count = 0
        # self.reorder = [3, 4, 5, 6, 2, 1, 0]
        # self.reorder_back = [6, 5, 4, 0, 1, 2, 3]

    def update(self, dets_all):
        """
        Params:
          dets_all: dict
            dets - a numpy array of detections in the format [[x,y,z,theta,l,w,h],[x,y,z,theta,l,w,h],...]
            info: a array of other info for each det
        Requires: this method must be called once for each frame even with empty detections.
        Returns the a similar array, where the last column is the object ID.

        NOTE: The number of objects returned may differ from the number of detections provided.
        """
        dets, info = dets_all['dets'], dets_all['info']         # dets: N x 7, float numpy array
        # dets = dets[:, self.reorder]
        self.frame_count += 1

        # N x 7 , #get predicted locations from existing trackers.
        trks = np.zeros((len(self.trackers), 7))
        to_del = []
        ret = []
        for t, trk in enumerate(trks):
            pos = self.trackers[t].predict().reshape((-1, 1))
            trk[:] = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]
            if(np.any(np.isnan(pos))):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            self.trackers.pop(t)

        dets_8corner = [convert_3dbox_to_8corner(det_tmp) for det_tmp in dets]
        if len(dets_8corner) > 0:
            dets_8corner = np.stack(dets_8corner, axis=0)
        else:
            dets_8corner = []

        trks_8corner = [convert_3dbox_to_8corner(trk_tmp) for trk_tmp in trks]
      
        trks_S = [np.matmul(np.matmul(tracker.kf.H, tracker.kf.P),
                            tracker.kf.H.T) + tracker.kf.R for tracker in self.trackers]

        if len(trks_8corner) > 0:
            trks_8corner = np.stack(trks_8corner, axis=0)
            trks_S = np.stack(trks_S, axis=0)

        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(
            dets_8corner, trks_8corner, use_mahalanobis=True, dets=dets, trks=trks, trks_S=trks_S, mahalanobis_threshold=11, print_debug=True, match_algorithm='greedy')

        # update matched trackers with assigned detections
        for t, trk in enumerate(self.trackers):
            if t not in unmatched_trks:
                d = matched[np.where(matched[:, 1] == t)[
                    0], 0]     # a list of index
                trk.update(dets[d, :][0], info[d, :][0])

        # create and initialise new trackers for unmatched detections
        for i in unmatched_dets:        # a scalar of index
            trk = KalmanBoxTracker(dets[i, :], info[i, :])
            self.trackers.append(trk)
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state()      # bbox location
            # d = d[self.reorder_back]

            if((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
                # +1 as MOT benchmark requires positive
                ret.append(np.concatenate(
                    (d, [trk.id+1], trk.info)).reshape(1, -1))
            i -= 1
            # remove dead tracklet
            if(trk.time_since_update >= self.max_age):
                self.trackers.pop(i)
        if(len(ret) > 0):
            # x, y, z, theta, l, w, h, ID, other info, confidence
            return np.concatenate(ret)
        return np.empty((0, 15))
