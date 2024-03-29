"""
The following scenarios are generated by hand

AIRPORT - 1 ROSplane loiter loiter and descend while 6 Drones passing through 
"""

AIRPORT = {            
    'plane0': [(-50.0, 0, 50.0),(0, -50, 50.0), (50.0, 0, 50.0), (0, 50.0, 50.0),(-50.0, 0, 50.0), (0, -50.0, 50.0), (50.0, 0, 50.0), (0, 50.0, 50.0),
               (-50.0, 0, 50.0),(0, -50, 50.0), (50.0, 0, 50.0), (0, 50.0, 50.0), (-50.0, 0, 50.0), (0, -50.0, 50.0), (50.0, 0, 50.0), (0, 50.0, 50.0),
               (-50.0, 0, 50.0),(0, -50, 50.0), (50.0, 0, 50.0), (150.0, 0, 40.0), (250.0, 0, 20.0), (350.0, 0, 10.0), (450.0, 0, 0)],             
    'drone0': [(150.0, 150.0, 40.0), (150.0, -100.0, 40.0), (150.0, -100, 0.5), (150.0, -100, 40.0), (150, 150, 40.0), (150.0, 150.0, 0.5)],
    'drone1': [(155.0, 150.0, 39.0), (155.0, -100.0, 39.0), (155.0, -100, 0.5), (155.0, -100, 39.0), (155, 150, 39.0), (155.0, 150.0, 0.5)],
    'drone2': [(160.0, 150.0, 38.0), (160.0, -100.0, 38.0), (160.0, -100, 0.5), (160.0, -100, 38.0), (160, 150, 38.0), (160.0, 150.0, 0.5)],
    'drone3': [(165.0, 150.0, 37.0), (165.0, -100.0, 37.0), (165.0, -100, 0.3), (165.0, -100, 37.0), (165, 150, 37.0), (165.0, 150.0, 0.3)],
    'drone4': [(170.0, 150.0, 36.0), (170.0, -100.0, 36.0), (170.0, -100, 0.3), (170.0, -100, 36.0), (170, 150, 36.0), (170.0, 150.0, 0.3)],
    'drone5': [(175.0, 150.0, 35.0), (175.0, -100.0, 35.0), (175.0, -100, 0.5), (175.0, -100, 35.0), (175, 150, 35.0), (175.0, 150.0, 0.5)]
}



SCENARIO_LIST = [AIRPORT]
