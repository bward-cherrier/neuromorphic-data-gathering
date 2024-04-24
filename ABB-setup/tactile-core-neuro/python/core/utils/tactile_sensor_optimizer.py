# -*- coding: utf-8 -*-
"""
Created on Thu Jul 12 16:09:41 2018

@author: John
"""

from __future__ import division, print_function, unicode_literals

import sys
import logging

import cv2
import numpy as np
import collections
import scipy.io as sio

from core.sensor.tactile_sensor import TactileSensor

np.set_printoptions(precision=2)


def detect_pins(frames, min_threshold, max_threshold, min_area, max_area,
                min_circularity, min_convexity, min_inertia_ratio):
    """ Detect pins using OpenCV blob detector and specified parameters.
    """   
    params = cv2.SimpleBlobDetector_Params()
    params.blobColor = 255
    params.minThreshold = min_threshold
    params.maxThreshold = max_threshold
    params.filterByArea = True
    params.minArea = min_area
    params.maxArea = max_area
    params.filterByCircularity = True
    params.minCircularity = min_circularity
    params.filterByConvexity = True
    params.minConvexity = min_convexity
    params.filterByInertia = True
    params.minInertiaRatio = min_inertia_ratio
    
    detector = cv2.SimpleBlobDetector_create(params)

    pins = []
    for frame in frames:    
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints = detector.detect(frame)       
        pins.append(np.array([k.pt for k in keypoints]))    
    return pins


def param_cost(frames, n_pins, min_threshold, max_threshold, min_area,
               max_area, min_circularity, min_convexity, min_inertia_ratio):
    """ Cost function for OpenCV blob detector parameters for  specified
        frames. Cost is computed as the mean absolute difference between the
        number of pins detected and the target number of pins.
    """
    pins = detect_pins(frames, min_threshold, max_threshold, min_area, max_area,
                       min_circularity, min_convexity, min_inertia_ratio)
    if min_threshold > max_threshold or min_area > max_area:
        cost = np.inf
    else:
        cost = 0.0
        for i in range(len(pins)):
            n_pins_detected = pins[i].shape[0]
            cost += abs(n_pins - n_pins_detected)
        cost /= len(pins)
    return cost


def cross_entropy(cost_func, lower, upper, pop_size=20, elite_size=10,
                  max_iters=10, mu=None, sigma=None, print_elite_costs=True):
    """ Cross-entropy optimizer for 'cost_func' of variables lying between
        'lower' and 'upper' bounds.
    """
    assert np.all(upper >= lower)
    assert elite_size <= pop_size
    
    # Specify mu and sigma based on bounds if not provided
    mu = 0.5 * (lower + upper) if mu is None else mu
    sigma = 0.5 * (upper - lower) if sigma is None else sigma

    n_vars = lower.size    
    alpha, beta = 0.7, 0.9
    q = 7
    
    for i in range(1, max_iters + 1):
        # Generate valid population using rejection sampling
        pop = mu + sigma * np.random.randn(pop_size, n_vars)
        pop = pop[np.logical_and(np.all(pop >= lower.T, axis=1),
                                 np.all(pop <= upper.T, axis=1))]
        while pop.shape[0] < pop_size:
            pop2 = mu + sigma * np.random.randn(pop_size, n_vars)
            pop2 = pop2[np.logical_and(np.all(pop2 >= lower.T, axis=1),
                                       np.all(pop2 <= upper.T, axis=1))]
            pop = np.vstack((pop, pop2))
        pop = pop[:pop_size]
        
        # Evaluate population costs
        cost = np.array([cost_func(pop[j]) for j in range(pop_size)])
        
        # Sort population by cost and select elite members
        order = np.argsort(cost)
        elite = pop[order[:elite_size]]
        elite_cost = cost[order[:elite_size]]
        
        if print_elite_costs:
            print("{}/{} : {}".format(i, max_iters, elite_cost))
        
        # Update search distribution using elite members
        mu = alpha * np.mean(elite, axis=0) + (1 - alpha) * mu
        beta_t = beta - beta * (1 - 1 / float(i)) ** q
        sigma = beta_t * np.std(elite, axis=0) + (1 - beta_t) * sigma
        
        # Check for convergence
        if np.mean(sigma ** 2) < 1e-5:
            break

    # Return best elite sample/cost and mea/std of elite samples
    return elite[0], cost[0], mu, sigma


def optimize(frames, bounds, n_pins):
    """ Two-phase optimizer for tactile sensor blob detector parameters.
    """
    bounds = np.array((bounds['min_threshold'],  
                       bounds['max_threshold'],
                       bounds['min_area'],
                       bounds['max_area'],
                       bounds['min_circularity'],
                       bounds['min_convexity'],
                       bounds['min_inertia_ratio']))
    lower, upper = bounds[:, 0], bounds[:, 1]
    n_frames = frames.shape[0]

    # Perform initial optimization using smaller number of frames
    n_samples = min(1, n_frames)
    print("\nPerforming initial optimization using {} frames ...".format(n_samples))
    print("\nCost of elite samples:")
    idx = np.random.permutation(np.arange(n_frames))[:n_samples]
    cost_func = lambda x : param_cost(frames[idx], n_pins,
                                      min_threshold=x[0],
                                      max_threshold=x[1],
                                      min_area=x[2],
                                      max_area=x[3],
                                      min_circularity=x[4],
                                      min_convexity=x[5],
                                      min_inertia_ratio=x[6])
    _, _, mu, sigma = cross_entropy(cost_func, lower, upper, pop_size=100,
                                    elite_size=10, max_iters=10)

    # Collate interim results
    opt_params = collections.OrderedDict()
    opt_params['min_threshold'] = mu[0]
    opt_params['max_threshold'] = mu[1]
    opt_params['min_area'] = mu[2]
    opt_params['max_area'] = mu[3]
    opt_params['min_circularity'] = mu[4]
    opt_params['min_convexity'] = mu[5]
    opt_params['min_inertia_ratio'] = mu[6]

    # Display interim results
    opt_cost = param_cost(frames, n_pins, **opt_params)
    print("\nOptimal parameters after initial optimization:")
    for k, v in opt_params.items():
        print("{:s} : {:.2f}".format(k, v))
    print("\ncost : {:.2f}".format(opt_cost))

    # Fine-tune using larger number of frames
    n_samples = min(50, n_frames)
    print("\nFine-tuning using {} frames (takes a little longer) ...".format(n_samples))
    print("\nCost of elite samples:")
    idx = np.random.permutation(np.arange(n_frames))[:n_samples]
    cost_func = lambda x : param_cost(frames[idx], n_pins,
                                      min_threshold=x[0],
                                      max_threshold=x[1],
                                      min_area=x[2],
                                      max_area=x[3],
                                      min_circularity=x[4],
                                      min_convexity=x[5],
                                      min_inertia_ratio=x[6]) 
    _, _, mu, sigma = cross_entropy(cost_func, lower, upper, pop_size=20,
                                    elite_size=10, max_iters=5, mu=mu, sigma=sigma)    

    # Collate final results        
    opt_params = collections.OrderedDict()
    opt_params['min_threshold'] = mu[0]
    opt_params['max_threshold'] = mu[1]
    opt_params['min_area'] = mu[2]
    opt_params['max_area'] = mu[3]
    opt_params['min_circularity'] = mu[4]
    opt_params['min_convexity'] = mu[5]
    opt_params['min_inertia_ratio'] = mu[6]

    # Display final results    
    opt_cost = param_cost(frames, n_pins, **opt_params)
    print("\nOptimal parameters after fine-tuning:")
    for k, v in opt_params.items():
        print("{:s} : {:.2f}".format(k, v))
    print("\ncost : {:.2f}".format(opt_cost))
    
    return opt_params

def writeMaskedVid(dataPath, video_source):

def main():
    # Python 2/3 compatibility
    get_input = input
    try:
        get_input = raw_input
    except NameError:
        pass
        
    # Initialise sensor and capture some frames
    get_input("Press Enter to capture frames (approx 3 secs): ")
    sensor = TactileSensor(video_source=0,
                           brightness=150,
                           contrast=10,
                           saturation=0,
                           exposure=-6,
                           pin_tracking=False)    

    frames = sensor.record_frames(100)
    sensor.close()

    # Specify parameter bounds and target number of pins 
    bounds = {'min_threshold' : (100, 200),
              'max_threshold' : (200, 210),
              'min_area' : (0, 200),
              'max_area' : (0, 200),
              'min_circularity' : (0.5, 0.9),
              'min_convexity' : (0.1, 0.9),
              'min_inertia_ratio' : (0.1, 0.9)}

    n_pins = 49

    # Run the optimizer    
    opt_params = optimize(frames, bounds, n_pins)

    # Try out sensor with optimal parameters
    get_input("Press Enter to try out optimized parameters (approx 10 secs): ")
    sensor = TactileSensor(video_source=0,
                           brightness=150,
                           contrast=10,
                           saturation=0,
                           exposure=-6,
                           **opt_params)
    
    pins = sensor.record_pins(300)
    sensor.close()
    print("\nShape of captured pin position array: {}".format(pins.shape))


if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr)
    logging.getLogger(__name__).setLevel(logging.DEBUG)
    main()