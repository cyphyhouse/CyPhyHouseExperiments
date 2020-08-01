"""
The following scenarios are generated by hand

SIMPLE_CORRIDOR - 6 Drones fly single file down the same corridor
LOOP            - 6 Drones fly the same loop
BUSY_CORRIDOR   - 6 Drones fly single file down the same corridor (3 Drones fly one way, 3 fly in the opposite direction)
"""
SIMPLE_CORRIDOR = {'drone0': [(-70.0, -65.0, 1.0), (-45.0, -66.0, 1.0), (-33.0, -65.0, 0.3)],
                   'drone1': [(-70.0, -65.0, 1.0), (-45.0, -66.0, 1.0), (-35.0, -65.0, 0.3)],
                   'drone2': [(-70.0, -65.0, 1.0), (-45.0, -66.0, 1.0), (-37.0, -65.0, 0.3)],
                   'drone3': [(-70.0, -65.0, 1.0), (-45.0, -66.0, 1.0), (-39.0, -65.0, 0.3)],
                   'drone4': [(-70.0, -65.0, 1.0), (-45.0, -66.0, 1.0), (-41.0, -65.0, 0.3)],
                   'drone5': [(-70.0, -65.0, 1.0), (-45.0, -66.0, 1.0), (-43.0, -65.0, 0.3)]}

LOOP = {
    'drone0': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-72, -66.5, 2.0), (-72, -66.5, 0.3)],
    'drone1': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-74, -66.5, 2.0), (-74, -66.5, 0.3)],
    'drone2': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-76, -66.5, 2.0), (-76, -66.5, 0.3)],
    'drone3': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-78, -66.5, 2.0), (-78, -66.5, 0.3)],
    'drone4': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-80, -66.5, 2.0), (-80, -66.5, 0.3)],
    'drone5': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-72, -68, 2.0), (-72, -68, 0.3)],
    'drone6': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-74, -68, 2.0), (-74, -68, 0.3)],
    'drone7': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-76, -68, 2.0), (-76, -68, 0.3)],
    'drone8': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-78, -68, 2.0), (-78, -68, 0.3)],
    'drone9': [(-64.0, -65.0, 2.0), (-64.0, -59.0, 2.0), (-39.5, -59.0, 4.0), (-39.5, -65.0, 2.0), (-64.0, -65.0, 2.0), (-80, -68, 2.0), (-80, -68, 0.3)]}

BUSY_CORRIDOR = {
    'drone0': [(-60.0, -65.0, 2.0), (-55.0, -65.0, 2.0), (-50.0, -65.0, 2.0), (-45.0, -65.0, 2.0), (-40.0, -65.0, 2.0), (-35.0, -64.0, 2.0), (-35.0, -64.0, 0.3)],
    'drone2': [(-60.0, -65.0, 2.0), (-55.0, -65.0, 2.0), (-50.0, -65.0, 2.0), (-45.0, -65.0, 2.0), (-40.0, -65.0, 2.0), (-36.5, -64.0, 2.0), (-36.5, -64.0, 0.3)],
    'drone4': [(-60.0, -65.0, 2.0), (-55.0, -65.0, 2.0), (-50.0, -65.0, 2.0), (-45.0, -65.0, 2.0), (-40.0, -65.0, 2.0), (-38.0, -64.0, 2.0), (-38.0, -64.0, 0.3)],
    'drone6': [(-60.0, -65.0, 2.0), (-55.0, -65.0, 2.0), (-50.0, -65.0, 2.0), (-45.0, -65.0, 2.0), (-40.0, -65.0, 2.0), (-35.0, -66.0, 2.0), (-35.0, -66.0, 0.3)],
    'drone8': [(-60.0, -65.0, 2.0), (-55.0, -65.0, 2.0), (-50.0, -65.0, 2.0), (-45.0, -65.0, 2.0), (-40.0, -65.0, 2.0), (-36.5, -66.0, 2.0), (-36.5, -66.0, 0.3)],
    'drone1': [(-45.0, -67.0, 2.0), (-50.0, -67.0, 2.0), (-55.0, -67.0, 2.0), (-60.0, -67.0, 2.0), (-65.0, -67.0, 2.0), (-80.0, -67.0, 2.0), (-80.0, -67.0, 0.3)],
    'drone3': [(-45.0, -67.0, 2.0), (-50.0, -67.0, 2.0), (-55.0, -67.0, 2.0), (-60.0, -67.0, 2.0), (-65.0, -67.0, 2.0), (-81.5, -67.0, 2.0), (-81.5, -67.0, 0.3)],
    'drone5': [(-45.0, -67.0, 2.0), (-50.0, -67.0, 2.0), (-55.0, -67.0, 2.0), (-60.0, -67.0, 2.0), (-65.0, -67.0, 2.0), (-83.0, -67.0, 2.0), (-83.0, -67.0, 0.3)],
    'drone7': [(-45.0, -67.0, 2.0), (-50.0, -67.0, 2.0), (-55.0, -67.0, 2.0), (-60.0, -67.0, 2.0), (-65.0, -67.0, 2.0), (-80.0, -69.0, 2.0), (-80.0, -69.0, 0.3)],
    'drone9': [(-45.0, -67.0, 2.0), (-50.0, -67.0, 2.0), (-55.0, -67.0, 2.0), (-60.0, -67.0, 2.0), (-65.0, -67.0, 2.0), (-81.5, -69.0, 2.0), (-81.5, -69.0, 0.3)],
}

CITY_SIM = {
    'drone0': [(-39.0, 7.0, 7.3), (-31.0, 7.0, 7.3), (-31.0, 7.0, 10.3), (-25.0, 7.0, 10.3), (-20.0, 7.0, 10.3), (-20.0, 7.0, 7.3), (-20.0, 12.0, 7.3), (-20.0, 17.0, 7.3), (-20, 22, 7.3), (-20, 27, 7.3), (-20, 27, 11.3), (-10, 27, 11.3),(-10, 7, 11.3), (-10, 7, 5.3)],
    'drone6': [(-40.0, 7.0, 7.3), (-31.0, 7.0, 7.3), (-31.0, 7.0, 10.3), (-25.0, 7.0, 10.3), (-20.0, 7.0, 10.3), (-20.0, 7.0, 7.3), (-20.0, 12.0, 7.3), (-20.0, 17.0, 7.3), (-20, 22, 7.3), (-20, 27, 7.3), (-20, 27, 11.3), (-10, 29, 11.3),(-10, 9, 11.3), (-10, 9, 5.3)]
}

"""
The following scenarios generated through the generateScenario.py file (a set of randomly generated waypoints)

RANDOM_SCENARIO1 - 6 Drones 3 waypoints each
RANDOM_SCENARIO2 - 6 Drones 6 waypoints each
RANDOM_SCENARIO3 - 6 Drones 9 waypoints each
RANDOM_SCENARIO4 - 10 Drones 4 waypoints each
RANDOM_SCENARIO6 - 10 Drones 6 waypoints each
"""
RANDOM_SCENARIO1 = {'drone0': [(-37.61, -67.08, 0.8), (-43.26, -38.57, 2.49), (-67.98, -50.72, 0.59)],
                    'drone1': [(-40.07, -65.17, 2.14), (-17.36, -44.54, 1.97), (-26.29, -43.94, 0.99)],
                    'drone2': [(-74.86, -58.16, 0.52), (-69.47, -67.48, 2.42), (-53.15, -68.59, 2.04)],
                    'drone3': [(-49.94, -47.47, 2.14), (-64.94, -57.79, 1.89), (-61.41, -35.24, 0.89)],
                    'drone4': [(19.28, -47.31, 2.43), (24.62, -64.83, 1.25), (-12.76, -64.46, 1.11)],
                    'drone5': [(0.91, -67.55, 2.27), (-10.38, -57.05, 2.1), (-8.24, -50.28, 0.88)]}


RANDOM_SCENARIO2 = {'drone0': [(-61.7, -45.67, 0.91), (-44.19, -48.37, 1.51), (-54.86, -50.6, 0.77), (-52.15, -55.02, 1.49), (-38.22, -51.96, 2.13), (-51.19, -45.96, 0.91)],
                    'drone1': [(21.88, -46.32, 1.11), (23.46, -68.85, 1.96), (5.58, -66.55, 0.46), (20.82, -63.94, 1.81), (-41.49, -68.06, 0.6), (22.39, -63.94, 2.12)],
                    'drone2': [(-61.86, -30.86, 2.37), (-40.09, -33.2, 1.95), (-23.21, -34.59, 0.99), (-33.24, -31.34, 0.9), (-71.15, -33.02, 0.56), (-34.81, -33.43, 2.04)],
                    'drone3': [(13.38, -52.81, 1.82), (12.96, -52.68, 1.1), (10.52, -41.33, 0.41), (15.96, -41.35, 0.93), (16.76, -55.83, 0.47), (14.22, -57.34, 0.91)],
                    'drone4': [(-10.53, -37.04, 1.47), (-14.9, -38.67, 1.18), (-13.94, -39.08, 1.5), (-31.87, -46.51, 1.43), (-24.5, -42.09, 1.54), (-26.87, -51.45, 2.02)],
                    'drone5': [(-9.17, -57.87, 0.76), (-11.21, -50.24, 1.9), (-11.02, -52.84, 1.42), (-35.15, -58.84, 2.21), (-8.94, -38.62, 1.69), (-25.88, -45.82, 1.27)]}


RANDOM_SCENARIO3 = {'drone0': [(3.65, -41.35, 1.62), (6.86, -45.81, 0.96), (0.73, -47.43, 1.58), (3.22, -51.06, 1.15), (1.96, -40.35, 1.66), (0.27, -40.47, 1.63), (5.69, -58.01, 1.44), (2.17, -55.8, 2.4), (5.84, -46.63, 1.92)],
                    'drone1': [(12.43, -66.59, 0.65), (-31.62, -64.91, 0.92), (-57.34, -66.62, 1.44), (-38.95, -65.9, 1.83), (-69.98, -66.17, 0.73), (-76.58, -54.07, 1.24), (-75.67, -60.09, 1.25), (-76.24, -55.38, 1.02), (-78.71, -57.92, 1.02)],
                    'drone2': [(-67.67, -64.22, 1.42), (-39.12, -65.54, 0.99), (-30.83, -55.59, 1.0), (-15.69, -48.08, 1.59), (-23.17, -58.04, 0.55), (-35.18, -56.85, 1.48), (-27.16, -42.57, 0.64), (-23.66, -55.17, 1.33), (-18.9, -45.26, 2.39)],
                    'drone3': [(-23.41, -30.02, 1.24), (-25.69, -33.0, 2.36), (-45.85, -34.02, 1.2), (-19.97, -33.81, 2.16), (-24.48, -34.6, 1.19), (-64.86, -30.95, 2.23), (-22.19, -33.64, 1.2), (-67.25, -32.26, 2.12), (-34.08, -34.6, 1.43)],
                    'drone4': [(-25.82, -63.48, 0.84), (11.95, -63.82, 0.47), (-21.28, -64.57, 2.36), (-41.3, -64.25, 0.98), (-51.99, -68.8, 2.48), (-43.75, -67.14, 2.44), (-67.28, -65.27, 2.04), (21.42, -63.81, 2.13), (17.34, -63.13, 1.04)],
                    'drone5': [(-26.84, -51.99, 1.36), (-16.44, -53.29, 1.28), (-30.67, -39.41, 0.7), (-24.95, -56.32, 0.41), (-25.71, -54.91, 1.87), (-15.0, -43.14, 2.19), (-36.43, -49.03, 1.66), (-24.72, -54.68, 2.24), (-8.02, -41.64, 0.5)]}

RANDOM_SCENARIO4 = {
    'drone0': [(7.83, -13.41, 3.7), (-2.41, -7.79, 2.67), (10.12, 11.2, 3.22), (10.12, 11.2, 0.3)],
    'drone1': [(-0.76, -14.91, 2.07), (9.52, 8.66, 1.9), (-15.37, -16.09, 2.5), (-15.37, -16.09, 0.3)],
    'drone2': [(2.8, 4.72, 3.22), (8.45, 3.13, 3.11), (-16.83, 10.49, 3.95), (-16.83, 10.49, 0.3)],
    'drone3': [(-11.5, -7.37, 1.64), (7.47, -13.56, 3.27), (11.47, 11.76, 2.5), (11.47, 11.76, 0.3)],
    'drone4': [(-6.73, -11.83, 3.67), (-9.26, -4.72, 2.53), (10.38, -15.63, 2.93), (10.38, -15.63, 0.3)],
    'drone5': [(-8.78, -3.58, 2.09), (6.97, 5.25, 2.83), (-16.26, 11.66, 2.34), (-16.26, 11.66, 0.3)],
    'drone6': [(-2.84, -1.36, 3.87), (-5.1, -4.32, 3.25), (-15.81, 10.65, 2.22), (-15.81, 10.65, 0.3)],
    'drone7': [(-8.02, -9.48, 3.42), (0.51, -13.35, 1.81), (-16.43, -15.65, 3.42), (-16.43, -15.65, 0.3)],
    'drone8': [(7.88, 5.14, 2.6), (-0.49, -6.68, 3.54), (11.16, 10.68, 2.16), (11.16, 10.68, 0.3)],
    'drone9': [(-10.57, 2.92, 1.56), (-1.2, -1.48, 3.97), (11.52, -16.02, 3.04), (11.52, -16.02, 0.3)]}

RANDOM_SCENARIO6 = {
    'drone0': [(7.83, -13.41, 3.7), (-2.4, -6.93, 3.25), (0.47, 7.23, 1.81), (0.18, -13.67, 3.88), (-15.75, -11.55, 3.73), (-15.75, -11.55, 0.3)],
    'drone1': [(-0.76, -14.91, 2.07), (-11.11, -10.97, 3.02), (6.46, 8.98, 3.24), (2.97, -13.05, 2.68), (11.13, -13.24, 2.45), (11.13, -13.24, 0.3)],
    'drone2': [(2.8, 4.72, 3.22), (-3.58, -5.18, 3.64), (-2.89, -7.86, 2.36), (-9.97, -0.73, 2.13), (-16.5, -2.64, 2.06), (-16.5, -2.64, 0.3)],
    'drone3': [(-11.5, -7.37, 1.64), (7.15, -6.55, 2.28), (-8.84, -2.31, 3.79), (-9.01, -13.3, 3.93), (11.38, 0.84, 3.22), (11.38, 0.84, 0.3)],
    'drone4': [(-6.73, -11.83, 3.67), (-10.79, 9.86, 3.6), (-5.87, -13.72, 2.5), (8.69, -2.93, 2.29), (11.28, -2.21, 1.79), (11.28, -2.21, 0.3)],
    'drone5': [(-8.78, -3.58, 2.09), (-10.15, -11.15, 3.72), (-2.53, -10.09, 3.87), (-14.77, -10.77, 2.65), (-15.69, 5.96, 2.86), (-15.69, 5.96, 0.3)],
    'drone6': [(-2.84, -1.36, 3.87), (0.67, -13.13, 3.1), (-10.12, -14.96, 2.32), (2.88, 6.63, 3.7), (-16.07, 8.41, 3.66), (-16.07, 8.41, 0.3)],
    'drone7': [(-8.02, -9.48, 3.42), (9.34, -10.81, 3.77), (5.86, -11.17, 1.81), (8.11, 8.06, 3.83), (11.47, -8.37, 2.83), (11.47, -8.37, 0.3)],
    'drone8': [(7.88, 5.14, 2.6), (-6.79, 9.02, 2.17), (-1.22, 1.63, 3.03), (6.88, -5.36, 1.91), (10.93, 4.23, 3.97), (10.93, 4.23, 0.3)],
    'drone9': [(-10.57, 2.92, 1.56), (-11.94, -1.18, 3.27), (-12.9, 6.07, 2.39), (2.89, -2.98, 2.05), (-16.11, -0.36, 1.72), (-16.11, -0.36, 0.3)]}

SCENARIO_LIST = [SIMPLE_CORRIDOR, LOOP, BUSY_CORRIDOR, RANDOM_SCENARIO1, RANDOM_SCENARIO2, RANDOM_SCENARIO3]