import os
import signal
from subprocess import Popen
import time
import sys
from std_srvs.srv import Empty
import rospy
from test_protocol import run_test_protocol
def run(cmd, stdout, stderr, cwd = '.'):
    """Run a given `cmd` in a subprocess, write logs to stdout / stderr.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    stdout : str or subprocess.PIPE object
        Destination of stdout output.
    stderr : str or subprocess.PIPE object
        Destination of stderr output.

    Returns
    -------
    A subprocess.Popen instance.
    """
    return Popen(cmd, stdout=stdout, stderr=stderr, shell=False,
                 preexec_fn=os.setsid, cwd=cwd)


def get_stdout_stderr(typ, datetime, dir):
    """Create stdout / stderr file paths."""
    out = '%s_%s_stdout.log' % (datetime, typ)
    err = '%s_%s_stderr.log' % (datetime, typ)
    return os.path.join(dir, out), os.path.join(dir, err)


def check_files_exist(files):
    """Check if given list of files exists.

    Parameters
    ----------
    files : list of str
        Files to check for existence.

    Returns
    -------
    None if all files exist. Else raises a ValueError.
    """
    errors = []
    for f in files:
        if not os.path.exists(f):
            errors.append(f)
    if errors:
        raise ValueError('File does not exist: %s' % errors)


def start_process(cmd, typ, start_time, idx, cwd = '.'):
    """Start a subprocess with the given command `cmd`.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    typ : str
        Type of subprocess. This will be included in the logs' file names.
    start_time : str
        Datetime string, will be included in the logs' file names as well as
        the resulting bag's name.
    dpath_logs :
        Path to log direcotry.

    Returns
    -------
    A subprocess.Popen instance.
    """
    print('Starting', typ.upper())
    # stdout, stderr = get_stdout_stderr(typ, start_time, dpath_logs)
    # with open(stdout, 'wb') as out, open(stderr, 'wb') as err:
    #     return run(cmd, stdout=out, stderr=err)
    stdout = f"./logs/stdout_{typ}_{idx}_{start_time}.log"
    stderr = f"./logs/stderr_{typ}_{idx}_{start_time}.log"
    with open(stdout, 'wb+') as out, open(stderr, 'wb+') as err:
        return run(cmd, stdout=out, stderr=err, cwd = cwd)

def check_pid(pid):        
    """ Check For the existence of a unix pid. """
    print(f'check existance of {pid}')
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True

def run_simulation(idx, scene, scene_info):
    # dpath_logs = args.dpath_logs
    # script_node = args.script_node
    # check_files_exist([script_node, dpath_logs])

    start_time = time.strftime('%Y%m%d_%H%M%S')

    # p_ros_core = start_process(['roscore'],
    #                             'ros', start_time)
    p_gazebo = start_process(
        ['rosrun', 'cym_gazebo', 'cymulate.py', scene], 
        'GAZEBO', 
        start_time,
        idx
    )
    p_server = start_process(
        ['sh', 'run.sh', idx],
        'SERVER', 
        start_time,
        idx, 
        cwd = '/home/younger/work/ACAS.sXu.Version.3/Software/Integration.Resources/ACAS_multi/Multi.sXu.Server'
    )
    # session_talker_node = start_process(['/bin/bash', script_node],
    #                            'talker_node', start_time,
    #                            dpath_logs)
    time.sleep(120)

    p_manager = start_process(
        ['python3.7', 'test_protocol.py', scene, scene_info, idx],
        'MANAGER',
        start_time,
        idx
    )
    p_client = start_process(
        ['sh', 'run.sh', idx],
        'CLIENT',
        start_time,
        idx, 
        cwd = '/home/younger/work/ACAS.sXu.Version.3/Software/Integration.Resources/ACAS_multi/Multi.sXu.Server.Client.Demo'
    )
    # print pids in case something goes wrong
    print('PGID GAZEBO: ', os.getpgid(p_gazebo.pid))
    print('PGID MANAGER: ', os.getpgid(p_manager.pid))
    # print('PGID TALKER NODE: ', os.getpgid(session_talker_node.pid))
    print('PGID CLIENT: ', os.getpgid(p_client.pid))
    print('PGID SERVER: ', os.getpgid(p_server.pid))
    # run_test_protocol(idx, scene, scene_info)
    time.sleep(10)

    print('Start Simlation')
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    unpause.call()

    time.sleep(30)
    while p_manager.poll() is None:
        time.sleep(5)

    end_time = time.strftime('%Y%m%d_%H%M%S')
    print(f'Killing GAZEBO. {end_time}')
    os.killpg(os.getpgid(p_gazebo.pid), signal.SIGTERM)

    print(f'Killing SERVER. {end_time}')
    os.killpg(os.getpgid(p_server.pid), signal.SIGTERM)
    # os.killpg(os.getpgid(session_talker_node.pid), signal.SIGTERM)
    # time.sleep(60)
    
    print(f'Killing Client')
    os.killpg(os.getpgid(p_client.pid), signal.SIGKILL)
    
    time.sleep(30)
    # print(f'Killing MANAGER')
    # os.killpg(os.getpgid(p_manager.pid), signal.SIGTERM)

if __name__ == '__main__':
    """Start ROS and the talker node each as a subprocess.

    Examples
    --------

    python  start_ros.py --script_node /notebooks/workspace/talker.sh \
    -l /notebooks/workspace/src/scripts
    """
    # import argparse

    # parser = argparse.ArgumentParser()
    # # parser.add_argument('--script_node', type=str,
    # #                     default='/notebooks/workspace/talker.sh')
    # # parser.add_argument('--dpath_logs', '-l', type=str,
    # #                     default='/notebooks/workspace/src/scripts')
    # args = parser.parse_args()
    run_simulation()