"""Main entry point for deployment or simulation"""

import tempfile

from .konfig import Konfig


def deploy_main(app_py: str, conf: Konfig) -> None:
    from . import deploy_irl

    device_map = deploy_irl.get_device_map()
    print("Discovered devices:\n" + '\n'.join([str(d) for d in device_map]))  # TODO Logging

    # TODO Use device list to generate/update global config file

    for local_cfg in conf.gen_all_local_configs():
        device = local_cfg['device']
        if device['bot_name'] not in device_map:
            print("[WARNING] Device \"" + device['bot_name'] + "\" is not available. " +
                  "Agents on this device are not deployed.")
            continue

        # FIXME Use library to handle filesystem paths
        start_bash_path = "start_" + device['bot_name'] + ".bash"

        deploy_irl.dump_start_script(app_py, local_cfg, start_bash_path)
        deploy_irl.upload_and_exec(
            device_addr=device['ip'],
            username=device['username'],
            password=device['password'],
            local_path=start_bash_path,
            remote_path=start_bash_path)

    # TODO Wait until shutdown?
    pass


def simulate_main(app_py: str, conf: Konfig) -> None:
    import yaml

    from cym_gazebo import create_roslaunch_instance, DeviceInitInfo
    from geometry_msgs.msg import Point
    import rospy
    import src.scripts.run as middleware

    device_info_list = []
    for local_cfg in conf.gen_all_local_configs():
        agent = local_cfg['agent']
        device = local_cfg['device']
        device_info_list.append(
            DeviceInitInfo(
                device['bot_name'],
                device['bot_type'],
                Point(agent['pid'], agent['pid'], 0.3)  # FIXME Set initial location
            )
        )

    launch_gazebo = create_roslaunch_instance(device_info_list)
    agent_thread_list = []
    try:
        launch_gazebo.start()

        rospy.sleep(5.0)  # FIXME wait for launch_gazebo finish starting

        for local_cfg in conf.gen_all_local_configs():
            agent = local_cfg['agent']
            local_cfg['device']['ros_node_prefix'] = \
                local_cfg['device']['bot_name'] + '/waypoint_node'  # FIXME Handle topic names uniformly
            # FIXME Avoid creating temporary file and directly pass configs to middleware
            with tempfile.NamedTemporaryFile(mode='w') as local_cfg_file:
                yaml.dump(local_cfg, local_cfg_file, default_flow_style=True)
                print("Instantiating agent", agent['pid'])
                th = middleware.run_app(app_py, local_cfg_file.name)
                agent_thread_list.append(th)

        launch_gazebo.spin()
    except KeyboardInterrupt:
        print("Sending stop event to all agent threads...")
        for th in agent_thread_list:
            if th.is_alive():
                th.stop()
    finally:
        launch_gazebo.stop()
        # Wait for all agent threads to finish
        for th in agent_thread_list:
            th.join()


def main(argv) -> None:
    # TODO Parse arguments better
    app_krd = argv[1]
    global_cfg_filename = argv[2]

    # TODO Call Koord Compiler
    app_py = app_krd

    with open(global_cfg_filename, 'r') as f:
        conf = Konfig(f)

    # TODO Run simulation or deployment scripts
    if False:
        deploy_main(app_py, conf)
    else:  # Do simulation
        simulate_main(app_py, conf)


if __name__ == "__main__":
    import sys
    main(sys.argv)
