# This small example demonstrates how a CoapServer can be used to receive a pointcloud
# from the sonair-evk unit without invoking ROS. The example is very simple and does not
# actually do anything with the points received.
#
# In order to run the example you should just run:
#
# cmd> python coap_example.py
#
# from the directory sonair-evk/ros/sonair-evk.

import time
from sonair_evk.coap_server import CoapServer


# The PointCloudReceiver class is a small class which purpose is to receive the pointclouds
# from the Coap server. When the Coap server has received a pointcloud from the sonair-evk it
# will call the callback method of the PointCloudReceiver.
class PointCloudReceiver:

    def __init__(self):
        self.pointcloud_list = []

    # Every time a new pointcloud is published by the sonair-evk this callback will
    # be called with the new pointcloud. The current implementation stores all the
    # the received pointclouds in a list and prints a message on stdout.
    def callback(self, points, _status, timestamp):
        self.pointcloud_list.append((timestamp, points))

        print(f"Received {len(points)} points with timestamp: {timestamp}")


# Each time a new pointcloud is received the CoapServer will call the now() method
# of this class to create a timestamp for the newly received pointcloud. When the
# Coap server is used together with ROS the timestamps will come from ROS.
class Clock:

    def now(self):
        return time.time()


def start_server():
    print("Starting coap server waiting for points - will continue until Ctrl-C")
    pointcloud_receiver = PointCloudReceiver()
    coap_server = CoapServer(pointcloud_receiver.callback, Clock())
    while True:
        time.sleep(1)


if __name__ == "__main__":
    start_server()
