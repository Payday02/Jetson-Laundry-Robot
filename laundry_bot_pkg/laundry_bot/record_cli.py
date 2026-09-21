#!/usr/bin/env python3
"""
record_cli.py — episode control from the terminal during data collection.

    ros2 run laundry_bot record_cli start "pick shirt from incoming basket"
    ros2 run laundry_bot record_cli stop              # save
    ros2 run laundry_bot record_cli stop --discard    # throw away
    ros2 run laundry_bot record_cli status

`start [task]` begins an episode (task string goes into the dataset);
`stop` saves it, `--discard` throws it away. These map 1:1 to the
record/start_episode and record/stop_episode services, so the exact same
commands work once the real Quest teleop is running.
"""
import sys

import rclpy
from rclpy.node import Node

from laundry_bot_interfaces.srv import StartEpisode, StopEpisode


def _wait(node, cli):
    if not cli.wait_for_service(timeout_sec=10.0):
        print("error: record service not available "
              "(is lerobot_recorder_node running?)")
        return False
    return True


def main():
    raw = sys.argv[1:]
    args = [a for a in raw if not a.startswith('--')]
    discard = '--discard' in raw
    if not args:
        print(__doc__)
        return
    action, task = args[0], (args[1] if len(args) > 1 else 'laundry demo')

    rclpy.init()
    node = Node('record_cli')
    try:
        if action == 'start':
            cli = node.create_client(StartEpisode, 'record/start_episode')
            if _wait(node, cli):
                res = cli.call_async(StartEpisode.Request(task=task))
                rclpy.spin_until_future_complete(node, res)
                out = res.result()
                print(f"start: success={out.success} "
                      f"episode={out.episode_index} ({out.message}) "
                      f"task='{task}'")
        elif action in ('stop', 'discard'):
            cli = node.create_client(StopEpisode, 'record/stop_episode')
            if _wait(node, cli):
                res = cli.call_async(StopEpisode.Request(discard=discard))
                rclpy.spin_until_future_complete(node, res)
                out = res.result()
                print(f"stop: success={out.success} frames={out.num_frames} "
                      f"({out.message})")
        elif action == 'status':
            print("echo the recorder status topic:")
            print("  ros2 topic echo /record/status --once")
        else:
            print(f"unknown action '{action}' (use: start|stop|status)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
