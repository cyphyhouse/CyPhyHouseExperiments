"""
This file contains core bloating algorithm for dryvr
"""

from typing import List, Tuple
import numpy as np
import scipy as sp
import scipy.spatial

_TRUE_MIN_CONST = -10
_EPSILON = 1.0e-100


def get_discrepancy_parameters(training_traces: np.ndarray, initial_radii: np.ndarray, method='PWGlobal') -> np.array:
    num_traces = training_traces.shape[0]
    ndims = training_traces.shape[2]  # This includes time
    trace_len = training_traces.shape[1]
    center_trace = training_traces[0, :, :]
    trace_initial_time = center_trace[0, 0]
    x_points = center_trace[:, 0] - trace_initial_time
    assert np.all(training_traces[0, :, 0] == training_traces[1:, :, 0])
    y_points = all_sensitivities_calc(training_traces, initial_radii)
    points = np.zeros((ndims - 1, trace_len, 2))
    points[np.where(initial_radii != 0), 0, 1] = 1.0
    points[:, :, 0] = np.reshape(x_points, (1, x_points.shape[0]))
    points[:, 1:, 1] = y_points
    normalizing_initial_set_radii = initial_radii.copy()
    normalizing_initial_set_radii[np.where(normalizing_initial_set_radii == 0)] = 1.0
    df = np.zeros((trace_len, ndims))
    if method == 'PW':
        return points
    elif method == 'PWGlobal':
        # replace zeros with epsilons
        # points[np.where(points[:, 0, 1] == 0), 0, 1] = 1.0e-100
        # to fit exponentials make y axis log of sensitivity
        points[:, :, 1] = np.maximum(points[:, :, 1], _EPSILON)
        points[:, :, 1] = np.log(points[:, :, 1])
        alldims_linear_separators = []
        for dim_ind in range(1, ndims):
            new_min = min(np.min(points[dim_ind - 1, 1:, 1]) + _TRUE_MIN_CONST, -10)
            if initial_radii[dim_ind - 1] == 0:
                # exclude initial set, then add true minimum points
                new_points = np.row_stack(
                    (np.array((points[dim_ind - 1, 1, 0], new_min)), np.array((points[dim_ind - 1, -1, 0], new_min))))
            else:
                # start from zero, then add true minimum points
                new_points = np.row_stack((points[dim_ind - 1, 0, :],
                                           np.array((points[dim_ind - 1, 0, 0], new_min)),
                                           np.array((points[dim_ind - 1, -1, 0], new_min))))
                df[0, dim_ind] = initial_radii[dim_ind - 1]
                # Tuple order is start_time, end_time, slope, y-intercept
            cur_dim_points = np.concatenate((points[dim_ind - 1, 1:, :], new_points), axis=0)
            cur_hull = sp.spatial.ConvexHull(cur_dim_points)
            linear_separators = []  # type: List[Tuple[float, float, float, float, int, int]]
            vert_inds = list(zip(cur_hull.vertices[:-1], cur_hull.vertices[1:]))
            vert_inds.append((cur_hull.vertices[-1], cur_hull.vertices[0]))
            for end_ind, start_ind in vert_inds:
                if cur_dim_points[start_ind, 1] != new_min and cur_dim_points[end_ind, 1] != new_min:
                    slope = (cur_dim_points[end_ind, 1] - cur_dim_points[start_ind, 1]) / (
                            cur_dim_points[end_ind, 0] - cur_dim_points[start_ind, 0])
                    y_intercept = cur_dim_points[start_ind, 1] - cur_dim_points[start_ind, 0] * slope
                    start_time = cur_dim_points[start_ind, 0]
                    end_time = cur_dim_points[end_ind, 0]
                    assert start_time < end_time
                    if start_time == 0:
                        linear_separators.append((start_time, end_time, slope, y_intercept, 0, end_ind + 1))
                    else:
                        linear_separators.append((start_time, end_time, slope, y_intercept, start_ind + 1, end_ind + 1))
            linear_separators.sort()
            alldims_linear_separators.append(linear_separators)
        return alldims_linear_separators


def get_reachtube(center_trace: np.ndarray, initial_radii, discrepancy_parameters,
                  method='PWGlobal'):  # combine both None variables
    normalizing_initial_set_radii = initial_radii.copy()
    normalizing_initial_set_radii[np.where(normalizing_initial_set_radii == 0)] = 1.0
    trace_len = center_trace.shape[0]
    ndims = center_trace.shape[1]  # This includes time
    if method == 'PWGlobal':
        df = np.zeros((trace_len, ndims))
        alldims_linear_separators = discrepancy_parameters
        ndims = center_trace.shape[1]
        for dim_ind in range(1, ndims):
            prev_val = 0
            prev_ind = 1 if initial_radii[dim_ind - 1] == 0 else 0
            linear_separators = alldims_linear_separators[dim_ind - 1]
            if initial_radii[dim_ind - 1] != 0:
                df[0, dim_ind] = initial_radii[dim_ind - 1]
            for linear_separator in linear_separators:
                _, _, slope, y_intercept, start_ind, end_ind = linear_separator
                assert prev_ind == start_ind
                assert start_ind < end_ind
                segment_t = center_trace[start_ind:end_ind + 1, 0]
                segment_df = normalizing_initial_set_radii[dim_ind - 1] * np.exp(y_intercept) * np.exp(
                    slope * segment_t)
                segment_df[0] = max(segment_df[0], prev_val)
                df[start_ind:end_ind + 1, dim_ind] = segment_df
                prev_val = segment_df[-1]
                prev_ind = end_ind
    else:
        print('Discrepancy computation method,', method, ', is not supported!')
        raise ValueError
    assert (np.all(df >= 0))
    reachtube_segment = np.zeros((trace_len - 1, 2, ndims))
    reachtube_segment[:, 0, :] = np.minimum(center_trace[1:, :] - df[1:, :], center_trace[:-1, :] - df[:-1, :])
    reachtube_segment[:, 1, :] = np.maximum(center_trace[1:, :] + df[1:, :], center_trace[:-1, :] + df[:-1, :])

    return reachtube_segment


def all_sensitivities_calc(training_traces: np.ndarray, initial_radii: np.ndarray):
    num_traces, trace_len, ndims = training_traces.shape
    normalizing_initial_set_radii = initial_radii.copy()
    y_points = np.zeros((normalizing_initial_set_radii.shape[0], trace_len - 1))
    normalizing_initial_set_radii[np.where(normalizing_initial_set_radii == 0)] = 1.0
    for cur_dim_ind in range(1, ndims):
        normalized_initial_points = training_traces[:, 0, 1:] / normalizing_initial_set_radii
        initial_distances = sp.spatial.distance.pdist(normalized_initial_points, 'chebyshev')
        for cur_time_ind in range(1, trace_len):
            y_points[cur_dim_ind - 1, cur_time_ind - 1] = np.max((sp.spatial.distance.pdist(
                np.reshape(training_traces[:, cur_time_ind, cur_dim_ind],
                           (training_traces.shape[0], 1)), 'chebychev')
                                                                  / normalizing_initial_set_radii[
                                                                      cur_dim_ind - 1]) / initial_distances)
    return y_points


