import numpy as np
# import math as m
import yaml
from gazebo_runner import run_simulation
import pickle

grid_test_points = [{'vint': 1050, 'theta': np.pi/2, 'rho': 43736, 'vown': 954.6, 'psi': -0.4296, 'xint': -43736, 'yint': 0},
{'vint': 900, 'theta': np.pi, 'rho': 43736, 'vown': 462.6, 'psi': 0, 'xint': 0, 'yint': -43736},
{'vint': 200, 'theta': -3*np.pi/4, 'rho': 43736, 'vown': 100, 'psi': 0.3617, 'xint': 30926, 'yint': -30926},
{'vint': 600, 'theta': 3*np.pi/4, 'rho': 43736, 'vown': 204.9, 'psi': -0.5415, 'xint': -30926, 'yint': -30926},
{'vint': 300, 'theta': np.pi/4, 'rho': 43736, 'vown': 362.3, 'psi': -0.2379, 'xint': -30933, 'yint': 30933},
{'vint': 750, 'theta': -np.pi/2, 'rho': 43736, 'vown': 609.3, 'psi': 0.6226, 'xint': 43736, 'yint': 0},
{'vint': 1145, 'theta': -3*np.pi/8, 'rho': 87472, 'vown': 1145.9, 'psi': 0.7835, 'xint': 80814, 'yint': 33474},
{'vint': 450, 'theta': -1*np.pi/4, 'rho': 43736, 'vown': 636.2, 'psi': 0.7577, 'xint': 30926, 'yint': 30926},
{'vint': 60, 'theta': 0, 'rho': 43736, 'vown': 497.4, 'psi': 0, 'xint': 0, 'yint': 43736},
{'vint': 600, 'theta': 0, 'rho': 120000, 'vown': 600, 'psi': np.pi, 'xint': 0, 'yint': 120000}]


ACAS_01 = {
    'drone0': [
               (0, 0.0, 40),
               (0, 29.09656181419166, 40),
               (0, 58.19312362838332, 40),
               (0, 261.86905632772494, 40),
               (0, 290.965618, 40),
               (0, 320.965618, 40)
               ],
               'drone1': [
                          (-133.30894903682029, 0.0, 40),
                          (-119.97889106532426, 29.09622794860579, 40),
                          (-106.64883309382825, 58.19245589721158, 40),
                          (-1.0, 261.86605153745208, 40),
                          (-1.0, 323.870440711, 40),
                          ],
}
string_init_0 = '- {bot_name: "drone0", bot_type: "QUAD", init_pos: '
string_init_1 = '- {bot_name: "drone1", bot_type: "QUAD", init_pos:'
init_pos_0 = [0.0, -5.0, 3]
init_pos_1 = [-133.35894903682029, 0, 3]


# def transform_waypoint(point, transform_information):
#     x1 = point[0]  # x_i
#     y1 = point[1]  # y_i
#     translation_vector, sc = transform_information
#     x_n = translation_vector[0]
#     y_n = translation_vector[1]
#     x2 = (x1 + x_n) * math.cos(sc) - (y1 + y_n) * math.sin(sc)
#     y2 = (x1 + x_n) * math.sin(sc) + (y1 + y_n) * math.cos(sc)
#     #x2 = round(x2)
#     #y2 = round(y2)
#     return [x2, y2,point[2]]


#for i, rot_angle in enumerate(np.linspace(0, 2*np.pi, num=10)):
#transformed_dict = {}
#for agent, points in ACAS_01.items():
#transformed_dict[agent]=[]
#   for point in points:
#    transformed_dict[agent].append(transform_waypoint(point, ([0,0,0], rot_angle)))
# theta_list = [float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), np.pi]
# vint_list = [60, 150, 300, 450, 600, 750, 900, 1050, 1145]
# rho_list = [10000, 43736, 87472, 120000]

# theta_list = [np.pi/3]
# vint_list = [900,]
# rho_list = [43736,]
# h_list = [-2000]

theta_list = [-np.pi*3/4, -np.pi*3/8, np.pi/3]
vint_list = [600, 750, 900, 1050]
rho_list = [10000, 43736, 87472, ]
h_list = [-2000, 0, 2000]

scale = 0.3048 / 40
T = 100
T_wp = 150
n = 1
# for i, test_point in enumerate(grid_test_points):
for i, theta in enumerate(theta_list):
    for j, vint in enumerate(vint_list):
        for k, rho in enumerate(rho_list):
            for l, h in enumerate(h_list):
                if np.isnan(theta) or np.isnan(vint) or np.isnan(rho):
                    continue
                print(f"Run simulation for configuration {theta}, {vint}, {rho}")
                yaml_fn = f'../scenes/looping/acas_{i}_{j}_{k}_{l}.yaml'
                wp_fn = f'../scenes/looping/acas_wp_{i}_{j}_{k}_{l}'
                init_pos_0 = [0, 0, 60]
                init_yaw_0 = np.pi / 2
                # vint = vint*scale

                if theta>0 and theta<np.pi:
                    phi = -np.arcsin(rho*np.sin(abs(theta))/(T*vint))
                    psi = np.pi-abs(theta)-abs(phi)
                    vown = vint*np.sin(psi)/np.sin(abs(theta))
                elif theta>-np.pi and theta<0:
                    phi = np.arcsin(rho*np.sin(abs(theta))/(T*vint))
                    psi = np.pi-abs(theta)-abs(phi)
                    vown = vint*np.sin(psi)/np.sin(abs(theta))
                elif theta==abs(np.pi):
                    phi = 0
                    psi = 0
                    vown = (T*vint-rho)/T
                else:
                    phi = 0
                    psi = 0
                    vown = (T*vint+rho)/T 

                if np.isnan(phi) or np.isnan(psi) or np.isnan(vown):
                    print(f'Configuration {theta}, {vint}, {rho} is not available')
                    continue
                xint = init_pos_0[0]+rho*np.cos(theta+np.pi/2)
                yint = init_pos_0[1]+rho*np.sin(theta+np.pi/2)

                init_lin_vel_0 = [0, vown * scale, 0]
                init_pos_1 = [xint * scale, yint * scale, init_pos_0[2] - h*scale]
                init_yaw_1 = np.pi / 2 + phi
                init_lin_vel_1 = [vint * np.cos(init_yaw_1) * scale, vint * np.sin(init_yaw_1) * scale, h*vint/rho*scale]
                yaml_str_0 = string_init_0 + str(init_pos_0) + ', init_yaw: ' + str(init_yaw_0) + ', init_lin_vel: ' + str(init_lin_vel_0) + '}\n'
                yaml_str_1 = string_init_1 + str(init_pos_1) + ', init_yaw: ' + str(init_yaw_1) + ', init_lin_vel: ' + str(init_lin_vel_1) + '}\n'
                print(yaml_str_0)
                print(yaml_str_1)
                with open(yaml_fn,'w+') as f:
                    f.write('world_name: "city.world"\n')
                    f.write('devices:\n')
                    f.write(yaml_str_0)
                    f.write(yaml_str_1)
                waypoints_dict = {}
                waypoints_dict['drone0'] =[]
                waypoints_dict['drone1'] =[]
                for m in range(n):
                    waypoints_dict['drone0'].append((init_pos_0[0] + init_lin_vel_0[0] * (m+1) * T_wp, init_pos_0[1] + init_lin_vel_0[1] * (m+1) * T_wp, init_pos_0[2]))
                    waypoints_dict['drone1'].append((init_pos_1[0] + init_lin_vel_1[0] * (m+1) * T_wp, init_pos_1[1] + init_lin_vel_1[1] * (m+1) * T_wp, init_pos_1[2] + init_lin_vel_1[2] * (m+1) * T_wp))
                # waypoints_dict['drone1'].append((init_pos_1[0] + init_lin_vel_1[0] * (n+1) * T_wp, init_pos_1[1] + init_lin_vel_1[1] * (n+1) * T_wp, 60))
                waypoints_str = f"acas_grid_{i}_{j}_{k}_{l} = {waypoints_dict}"
                print(waypoints_str)
                with open(wp_fn, 'wb+') as f:
                    pickle.dump([waypoints_dict, {'drone0', 'drone1'}], f)
                run_simulation(f"{i}_{j}_{k}_{l}", yaml_fn, wp_fn)
# {'drone0': [(init_pos_0[0] * scale + init_lin_vel_0[0] * j * T * scale, init_pos_0[1] * scale + init_lin_vel_0[1] * T * scale, 40)], 'drone1': [(init_pos_1[0] * scale + init_lin_vel_1[0] * T * scale, init_pos_1[1] * scale + init_lin_vel_1[1] * T * scale, 40)]}
