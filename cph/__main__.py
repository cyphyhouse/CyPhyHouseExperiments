"""Main entry point for deployment or simulation"""

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
    import multiprocessing as mp
    import yaml

    from cym_gazebo import create_roslaunch_instance, DeviceInitInfo
    from geometry_msgs.msg import Point
    import rospy
    import src.scripts.run as middleware

    mp.set_start_method('spawn')

    device_info_list = []
    agent_proc_list = []
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
        # Generate a process object for each Agent
        device['ros_node_prefix'] = \
            device['bot_name'] + '/waypoint_node'  # FIXME Handle topic names uniformly
        proc = mp.Process(target=middleware.run_app, args=(app_py, local_cfg))
        agent_proc_list.append(proc)

    launch_gazebo = create_roslaunch_instance(device_info_list)
    try:
        launch_gazebo.start()

        rospy.sleep(8.0)  # FIXME wait for launch_gazebo finish starting
        for agent in agent_proc_list:
            agent.start()

        launch_gazebo.spin()
    except KeyboardInterrupt:
        print("User sent SIGINT. Sending SIGTERM to all agent processes...")
    finally:
        for agent in agent_proc_list:
            if agent.is_alive():
                agent.terminate()
                agent.join()  # Wait for the agent to finish
        launch_gazebo.stop()


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
