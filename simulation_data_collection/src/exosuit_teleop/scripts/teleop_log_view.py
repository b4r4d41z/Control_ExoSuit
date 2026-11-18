#!/usr/bin/env python3
"""Display one controller component from ROS1 /rosout."""

import argparse
from pathlib import Path
import re
from signal import SIGINT, SIGTERM, signal
from threading import Event, Thread
from xmlrpc.client import Error, ServerProxy, Transport

import rosgraph
import rospy
from rosgraph_msgs.msg import Log

from exosuit_teleop.teleop_diagnostics import COMPONENTS, event_line


class ProbeTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


def call(uri, method, *args):
    with ServerProxy(uri, transport=ProbeTransport()) as server:
        return getattr(server, method)(*args)


class View:
    def __init__(self, component, node, exit_file, stopped):
        self.component, self.node, self.exit_file = component, node, exit_file
        self.stopped = stopped
        self.master = rosgraph.get_master_uri()
        self.caller = "/teleop_log_view_probe"
        self.seen_controller = Event()
        self.previous = None
        self.listening = False
        self.pattern = re.compile(
            rf"^\[\d{{2}}:\d{{2}}:\d{{2}}\] \[{component}\] "
            r"\[(?:INFO|WARN|ERROR|FATAL)\] STATE=[A-Z_]+ \| EVENT=[A-Z0-9_]+"
            r"(?: \| DETAIL=[^\n]*)?$"
        )

    def status(self, level, state, event, detail=""):
        result = level, state, event, detail
        if self.previous != result:
            print(event_line(self.component, *result), flush=True)
            self.previous = result

    def message(self, msg):
        if msg.name == self.node:
            self.seen_controller.set()
            if self.pattern.fullmatch(msg.msg):
                print(msg.msg, flush=True)

    def listen(self):
        try:
            rospy.init_node("teleop_log_view", anonymous=True, disable_signals=True,
                            disable_rosout=True, disable_rostime=True)
            rospy.Subscriber("/rosout", Log, self.message, queue_size=100)
        except rospy.ROSException as error:
            if not self.stopped.is_set():
                print(event_line(self.component, "ERROR", "DISCONNECTED",
                                 "ROSOUT_SUBSCRIBE_FAILED", error), flush=True)

    def controller_unavailable(self, registered):
        if self.exit_file:
            try:
                code = int(self.exit_file.read_text())
            except (OSError, ValueError):
                pass
            else:
                self.status("WARN", "STOPPED", "LAUNCH_EXITED",
                            f"Arms launch exit status {code}. See arms.log for the node's exit reason.")
                return
        if registered:
            self.status("WARN", "DISCONNECTED", "CONTROLLER_UNREACHABLE", self.node)
        elif self.seen_controller.is_set():
            self.status("WARN", "STOPPED", "CONTROLLER_EXITED", self.node)
        else:
            self.status("INFO", "WAITING", "WAITING_FOR_CONTROLLER", self.node)

    def poll(self):
        try:
            code, _, uri = call(self.master, "lookupNode", self.caller, self.node)
        except (OSError, Error):
            # Use a stable status key so a refused/failed poll is not logged on every iteration.
            self.status("ERROR", "DISCONNECTED", "ROS_MASTER_UNAVAILABLE", self.master)
            return

        if not self.listening:
            Thread(target=self.listen, daemon=True).start()
            self.listening = True

        if code != 1:
            self.controller_unavailable(registered=False)
            return

        try:
            code, _, _ = call(uri, "getPid", self.caller)
        except (OSError, Error):
            code = 0
        if code == 1:
            self.seen_controller.set()
            self.status("INFO", "ONLINE", "CONTROLLER_CONNECTED", self.node)
        else:
            self.controller_unavailable(registered=True)

    def run(self):
        try:
            while not self.stopped.is_set():
                self.poll()
                self.stopped.wait(1)
        finally:
            rospy.signal_shutdown("Log view closed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component", required=True, choices=COMPONENTS)
    parser.add_argument("--node", default="/exosuit_teleop")
    parser.add_argument("--launcher-exit-file", type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    stopped = Event()

    def stop(_signal, _frame):
        stopped.set()

    signal(SIGINT, stop)
    signal(SIGTERM, stop)
    View(args.component, args.node, args.launcher_exit_file, stopped).run()


if __name__ == "__main__":
    main()
